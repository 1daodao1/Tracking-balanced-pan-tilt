# -*- coding: utf-8 -*-
"""
============================================================
            STM32H750 + OV5600 红球识别上位机
============================================================
功能说明：
    1. 通过串口接收 STM32H750 发送的 JPEG 图像数据；
    2. 使用 OpenCV 对图像进行红色球体识别；
    3. 实时显示红球相对图像中心点的水平 / 垂直偏移距离；
    4. 将偏移距离通过串口回传给 STM32H750 主控芯片。

坐标系定义（以图像中心为原点）：
    - 水平方向：红球在中心点右侧为正，左侧为负；
    - 垂直方向：红球在中心点上方为正，下方为负
      （注意：OpenCV 图像坐标 y 轴向下，程序内部已做翻转）。

============================================================
                    通信协议设计
============================================================
[1] 接收图像（STM32 -> PC）：
    采用标准 JPEG 流，每帧以 0xFF 0xD8（SOI）开始，
    以 0xFF 0xD9（EOI）结束。STM32 侧 OV5600 应配置为
    JPEG 输出模式，每帧完整 JPEG 字节通过串口连续发送即可。

[2] 回传数据（PC -> STM32）：共 7 字节
    -----------------------------------------------------
    | 0xAA | 0x55 | dx_L | dx_H | dy_L | dy_H | 0x5A |
    -----------------------------------------------------
    - 帧头：0xAA 0x55
    - dx：水平偏移，int16 小端（低字节在前）
    - dy：垂直偏移，int16 小端
    - 帧尾：0x5A
    STM32 侧解析时只需匹配帧头帧尾，按 int16_t 还原即可。

============================================================
                    运行环境
============================================================
    Python 版本：Python 3.8 或更高
    依赖第三方库：
        opencv-python    （图像处理与显示）
        numpy            （数组运算，opencv 依赖）
        pyserial         （串口通信）

    一键安装命令：
        pip install opencv-python numpy pyserial

    操作系统：Windows / Linux / macOS 均可（串口号写法不同）
        Windows 串口名示例： COM3、COM5
        Linux  串口名示例： /dev/ttyUSB0、/dev/ttyACM0
        macOS  串口名示例： /dev/tty.usbserial-xxxx
============================================================
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import struct
import threading
import time
from queue import Queue, Empty


# ==================== 全局配置参数 ====================

# JPEG 帧的起始和结束标志（标准 JPEG 格式）
JPEG_SOI = b'\xFF\xD8'   # Start Of Image
JPEG_EOI = b'\xFF\xD9'   # End Of Image

# 回传数据帧的帧头和帧尾
TX_HEADER = b'\xAA\x55'
TX_TAIL = b'\x5A'

# 红色在 HSV 色彩空间中跨越 0°，因此需要两段区间共同描述
# 第一段：0° ~ 10°
RED_LOWER_1 = np.array([0, 120, 70])
RED_UPPER_1 = np.array([10, 255, 255])
# 第二段：170° ~ 180°
RED_LOWER_2 = np.array([170, 120, 70])
RED_UPPER_2 = np.array([180, 255, 255])

# 形态学操作的结构元（椭圆形 5x5，用于去噪和填洞）
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# 最小有效轮廓面积（像素），小于该值视为噪声忽略
MIN_CONTOUR_AREA = 300

# 串口读写锁（防止读线程和写线程同时操作串口）
serial_lock = threading.Lock()


# ==================== 工具函数 ====================

def list_available_ports():
    """列出当前系统所有可用的串口设备名"""
    return [p.device for p in serial.tools.list_ports.comports()]


def get_user_config():
    """与用户交互，获取串口号和波特率"""
    print("=" * 55)
    print("           STM32H750 红球识别上位机")
    print("=" * 55)

    # 显示当前系统可识别到的串口列表，方便用户选择
    available = list_available_ports()
    if available:
        print("[信息] 当前系统检测到的可用串口：")
        for idx, name in enumerate(available):
            print(f"        [{idx}] {name}")
    else:
        print("[警告] 当前系统未检测到任何串口设备！")

    # ---- 输入串口号 ----
    port = input("\n请输入串口号 (如 COM3 或 /dev/ttyUSB0): ").strip()
    while not port:
        port = input("串口号不能为空，请重新输入: ").strip()

    # ---- 输入波特率 ----
    while True:
        bd_str = input("请输入波特率 (如 115200、460800、921600): ").strip()
        try:
            baudrate = int(bd_str)
            if baudrate <= 0:
                raise ValueError
            break
        except ValueError:
            print("[错误] 波特率必须为正整数，请重新输入！")

    return port, baudrate


# ==================== 串口图像接收线程 ====================

class SerialImageReceiver:
    """
    串口图像接收线程：
        持续从串口读取字节流，按 JPEG 的 SOI / EOI 标记
        切分出一帧完整图像，放入线程安全队列中供主线程消费。

    设计要点：
        - 使用守护线程后台运行，主程序退出时自动结束；
        - 队列容量限制为 2，保证主线程始终处理最新帧（实时性）；
        - 当缓冲区无 SOI 时，仅保留最后 1 字节（避免分包导致 SOI 丢失）。
    """

    def __init__(self, ser):
        self.ser = ser
        self.frame_queue = Queue(maxsize=2)
        self.running = False
        self.thread = None
        self.buffer = bytearray()       # 接收缓冲区

    def start(self):
        """启动后台接收线程"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """停止接收线程"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _receive_loop(self):
        """接收主循环：读取串口数据 -> 解析 JPEG -> 入队"""
        while self.running:
            try:
                # 读取串口当前缓冲区中所有可用字节
                with serial_lock:
                    n = self.ser.in_waiting
                    data = self.ser.read(n) if n > 0 else b''

                if not data:
                    # 没有数据时让出 CPU，避免 100% 占用
                    time.sleep(0.001)
                    continue

                # 把新数据追加到缓冲区
                self.buffer.extend(data)

                # 在缓冲区中持续解析完整 JPEG 帧
                while True:
                    soi_idx = self.buffer.find(JPEG_SOI)
                    if soi_idx < 0:
                        # 没找到 SOI：丢弃旧数据，保留最后 1 字节（可能是 SOI 的前半部分）
                        if len(self.buffer) > 1:
                            self.buffer = self.buffer[-1:]
                        break

                    # 从 SOI 之后查找 EOI
                    eoi_idx = self.buffer.find(JPEG_EOI, soi_idx + 2)
                    if eoi_idx < 0:
                        # 没找到 EOI：丢弃 SOI 之前的无效数据，等待后续字节
                        if soi_idx > 0:
                            self.buffer = self.buffer[soi_idx:]
                        break

                    # 提取完整 JPEG 帧
                    jpeg_bytes = bytes(self.buffer[soi_idx:eoi_idx + 2])
                    # 从缓冲区中删除已处理部分
                    self.buffer = self.buffer[eoi_idx + 2:]

                    # 队列满则丢弃最旧帧，保证实时性
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except Empty:
                            pass
                    self.frame_queue.put(jpeg_bytes)

            except Exception as e:
                print(f"[接收线程] 异常：{e}")
                time.sleep(0.1)

    def get_frame(self, timeout=1.0):
        """从队列中阻塞获取一帧 JPEG 数据，超时返回 None"""
        try:
            return self.frame_queue.get(timeout=timeout)
        except Empty:
            return None


# ==================== 红球检测算法 ====================

def detect_red_ball(frame_bgr):
    """
    检测图像中最显著的红球。

    参数：
        frame_bgr : BGR 格式图像 (np.ndarray)
    返回：
        若检测到红球：返回 (cx, cy, radius)
        若未检测到：返回 None
    """
    # 1) 将图像转换到 HSV 空间（HSV 对光照变化更鲁棒）
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # 2) 分别取红色的两段范围并合并掩膜
    mask1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
    mask2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 3) 形态学处理：开运算去除孤立噪点，闭运算填补球体内部空洞
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    # 4) 寻找所有外部轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 5) 选择面积最大的轮廓（默认场景中红球为主要红色目标）
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None

    # 6) 用最小外接圆拟合，得到球心和半径
    (cx, cy), radius = cv2.minEnclosingCircle(largest)
    return int(cx), int(cy), int(radius)


# ==================== 串口数据发送 ====================

def send_offset(ser, dx, dy):
    """
    将红球的水平 / 垂直偏移距离通过串口发送给 STM32。

    数据帧格式（共 7 字节）：
        0xAA 0x55 | dx(int16 小端) | dy(int16 小端) | 0x5A
    """
    # 限幅到 int16 有效范围，防止 struct.pack 越界
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))

    # '<hh' 表示：小端、2 个有符号 16 位整数
    payload = struct.pack('<hh', dx, dy)
    frame = TX_HEADER + payload + TX_TAIL

    try:
        with serial_lock:
            ser.write(frame)
    except Exception as e:
        print(f"[发送] 串口写入异常：{e}")


# ==================== 主程序 ====================

def main():
    # 1. 获取用户输入的串口配置
    port, baudrate = get_user_config()

    # 2. 打开串口
    try:
        ser = serial.Serial(port=port,
                            baudrate=baudrate,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=0.1)
        print(f"\n[串口] {port} 已成功打开，波特率 {baudrate}")
    except Exception as e:
        print(f"[串口] 打开失败：{e}")
        return

    # 3. 启动后台图像接收线程
    receiver = SerialImageReceiver(ser)
    receiver.start()
    print("[系统] 开始接收图像数据，按 'q' 键退出窗口\n")

    # FPS 统计变量
    fps_t0 = time.time()
    fps_count = 0
    fps_value = 0.0

    try:
        while True:
            # 4. 从队列取出一帧 JPEG
            jpeg_data = receiver.get_frame(timeout=2.0)
            if jpeg_data is None:
                print("[警告] 2 秒内未收到完整图像帧，请检查 STM32 端发送是否正常")
                continue

            # 5. 解码 JPEG -> BGR 图像
            np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                # JPEG 数据损坏（可能是串口丢字节），丢弃该帧
                print("[警告] JPEG 解码失败，跳过该帧")
                continue

            # 6. 计算图像中心坐标
            h, w = frame.shape[:2]
            cx_img, cy_img = w // 2, h // 2

            # 7. 调用红球检测
            result = detect_red_ball(frame)

            if result is not None:
                bx, by, r = result

                # 计算偏移量
                # 水平：球在图像中心右侧时 (bx > cx_img) 为正
                dx = bx - cx_img
                # 垂直：球在图像中心上方时 (by < cy_img) 为正
                # OpenCV 图像 y 轴向下，因此用 (cy_img - by) 实现“上正下负”
                dy = cy_img - by

                # 绘制可视化效果
                cv2.circle(frame, (bx, by), r, (0, 255, 0), 2)           # 绿色：外接圆
                cv2.circle(frame, (bx, by), 4, (0, 0, 255), -1)          # 红点：球心
                cv2.line(frame, (cx_img, cy_img), (bx, by), (255, 255, 0), 1)  # 黄线：中心->球心
                cv2.putText(frame, f"dx={dx:+d}  dy={dy:+d}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # 8. 通过串口回传偏移量
                send_offset(ser, dx, dy)

                print(f"[识别] 球心=({bx:3d},{by:3d})  偏移 dx={dx:+5d}  dy={dy:+5d}  半径={r}")

            else:
                # 未检测到红球：界面提示并发送 (0, 0)
                cv2.putText(frame, "No red ball", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                send_offset(ser, 0, 0)

            # 9. 绘制图像中心十字参考线
            cv2.line(frame, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (255, 255, 255), 1)
            cv2.line(frame, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (255, 255, 255), 1)

            # 10. 计算并显示 FPS
            fps_count += 1
            elapsed = time.time() - fps_t0
            if elapsed >= 1.0:
                fps_value = fps_count / elapsed
                fps_count = 0
                fps_t0 = time.time()
            cv2.putText(frame, f"FPS: {fps_value:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 11. 显示图像窗口
            cv2.imshow("Red Ball Detection - press 'q' to quit", frame)

            # 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[系统] 检测到 Ctrl+C，准备退出...")
    finally:
        # 12. 资源清理
        receiver.stop()
        if ser.is_open:
            ser.close()
        cv2.destroyAllWindows()
        print("[系统] 程序已安全退出")


if __name__ == "__main__":
    main()
