#include "vision.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/USART4/usart4.h"
#include "stdio.h"

/* 跟踪控制参数 */
//中心死区，控制中心点抖动
#define TRACK_DEAD_ZONE_X          10      /* X方向死区，单位：像素 */
#define TRACK_DEAD_ZONE_Y          10      /* Y方向死区，单位：像素 */

//单轴变差容差，越大越不容易反向
#define TRACK_AXIS_WORSE_MARGIN    4       /* 单轴变差容差，防止轻微抖动导致频繁反向 */

//整体距离变差容差，越大越稳定但反应慢
#define TRACK_DIST_WORSE_MARGIN    100     /* 距离平方变差容差，100约等于10像素误差 */

//越小发送越快
#define TRACK_SEND_PERIOD_MS       50      /* 串口发送周期，单位ms */

/* 上一时刻的偏移量 */
static int16_t s_last_dx = 0;
static int16_t s_last_dy = 0;
static uint32_t s_last_dist2 = 0;

/* 当前方向修正系数
 *  1 表示保持当前方向
 * -1 表示该方向取反
 */
static int8_t s_dir_x = 1;
static int8_t s_dir_y = 1;

static uint8_t s_has_last = 0;
static uint32_t s_last_send_tick = 0;


/**
 * @brief 清空跟踪控制历史数据
 * @note  进入跟踪模式前、目标丢失时可以调用
 */
void TrackControl_Reset(void)
{
    s_last_dx = 0;
    s_last_dy = 0;
    s_last_dist2 = 0;

    s_dir_x = 1;
    s_dir_y = 1;

    s_has_last = 0;
    s_last_send_tick = 0;
}


/**
 * @brief       根据当前小球偏移量判断控制方向，并通过UART4发送给子机
 * @param       ball: 小球识别结果
 * @note        发送格式: "@dx,dy\r\n"
 */
void TrackControl_UpdateAndSend(BallResult_t *ball)
{
    char buf[32];

    uint32_t now_tick;
    int16_t dx;
    int16_t dy;
    int16_t send_dx;
    int16_t send_dy;

    int16_t abs_dx;
    int16_t abs_dy;
    int16_t last_abs_dx;
    int16_t last_abs_dy;

    uint32_t dist2;
    uint8_t x_worse = 0;
    uint8_t y_worse = 0;

    now_tick = HAL_GetTick();

    /* 限制串口发送频率，避免发送太快 */
    if (now_tick - s_last_send_tick < TRACK_SEND_PERIOD_MS)
    {
        return;
    }

    s_last_send_tick = now_tick;

    /* 没找到球：发送停止指令，并清空历史 */
    if (ball == 0 || ball->found == 0)
    {
        sprintf(buf, "@0,0\r\n");
        UART4_SendString(buf);

        TrackControl_Reset();
        return;
    }

    dx = ball->dx;
    dy = ball->dy;

    /* 计算当前dx绝对值 */
    if (dx < 0)
    {
        abs_dx = -dx;
    }
    else
    {
        abs_dx = dx;
    }

    /* 计算当前dy绝对值 */
    if (dy < 0)
    {
        abs_dy = -dy;
    }
    else
    {
        abs_dy = dy;
    }

    /* X方向死区 */
    if (abs_dx <= TRACK_DEAD_ZONE_X)
    {
        dx = 0;
        abs_dx = 0;
    }

    /* Y方向死区 */
    if (abs_dy <= TRACK_DEAD_ZONE_Y)
    {
        dy = 0;
        abs_dy = 0;
    }

    /* 已经在中心附近：发送停止 */
    if (dx == 0 && dy == 0)
    {
        sprintf(buf, "@0,0\r\n");
        UART4_SendString(buf);

        s_last_dx = 0;
        s_last_dy = 0;
        s_last_dist2 = 0;
        s_has_last = 1;
        return;
    }

    /* 当前距离中心点的距离平方 */
    dist2 = (uint32_t)((int32_t)dx * dx + (int32_t)dy * dy);

    /*
     * 第一次识别到球：
     * 不做变好/变坏判断，直接发送当前偏移量
     */
    if (s_has_last == 0)
    {
        s_has_last = 1;

        s_last_dx = dx;
        s_last_dy = dy;
        s_last_dist2 = dist2;

        send_dx = dx * s_dir_x;
        send_dy = dy * s_dir_y;

        sprintf(buf, "@%d,%d\r\n", send_dx, send_dy);
        UART4_SendString(buf);
        return;
    }

    /*
     * 如果当前距离中心点变大，说明上一次控制方向可能错了
     * 根据X/Y哪个方向变差，分别反向
     */
    if (dist2 > s_last_dist2 + TRACK_DIST_WORSE_MARGIN)
    {
        if (s_last_dx < 0)
        {
            last_abs_dx = -s_last_dx;
        }
        else
        {
            last_abs_dx = s_last_dx;
        }

        if (s_last_dy < 0)
        {
            last_abs_dy = -s_last_dy;
        }
        else
        {
            last_abs_dy = s_last_dy;
        }

        if (abs_dx > last_abs_dx + TRACK_AXIS_WORSE_MARGIN)
        {
            x_worse = 1;
        }

        if (abs_dy > last_abs_dy + TRACK_AXIS_WORSE_MARGIN)
        {
            y_worse = 1;
        }

        /* X方向变差，X方向取反 */
        if (x_worse)
        {
            s_dir_x = -s_dir_x;
        }

        /* Y方向变差，Y方向取反 */
        if (y_worse)
        {
            s_dir_y = -s_dir_y;
        }

        /*
         * 整体距离变差，但单轴没有明显变差：
         * 说明可能两个方向都有轻微问题，两个方向同时取反
         */
        if (x_worse == 0 && y_worse == 0)
        {
            s_dir_x = -s_dir_x;
            s_dir_y = -s_dir_y;
        }
    }

    /*
     * 应用方向修正系数
     * 发送格式：@dx,dy\r\n
     */
    send_dx = dx * s_dir_x;
    send_dy = dy * s_dir_y;

    sprintf(buf, "@%d,%d\r\n", send_dx, send_dy);
    UART4_SendString(buf);

    /* 更新上一时刻数据 */
    s_last_dx = dx;
    s_last_dy = dy;
    s_last_dist2 = dist2;
}


/**
 * @brief       在 RGB565 图像中查找黄色球的位置
 * @param       framebuffer: 图像缓冲区指针。如果为 NULL，则通过底层驱动直接从 LCD 显存读取像素。
 * @param       width:  图像宽度
 * @param       height: 图像高度
 * @retval      BallResult_t 结构体，包含是否找到、球心坐标及中心偏移量
 */
BallResult_t FindBall_RGB565(uint16_t *framebuffer, uint16_t width, uint16_t height)
{
    BallResult_t result = {0, 0, 0, 0, 0};
    uint32_t sum_x = 0; // 所有红色像素点 X 坐标之和
    uint32_t sum_y = 0; // 所有红色像素点 Y 坐标之和
    uint32_t count = 0; // 红色像素点的总个数

    /* 
     * 遍历图像像素
     * 步长设置为 2 (+= 2) 可以跳过部分像素，大幅减少 CPU 运算量，提高帧率。
     * 对于 320x240 分辨率，处理像素从 76800 降至 19200。
     */
    for (uint16_t y = 0; y < height; y += 2) 
    {
        for (uint16_t x = 0; x < width; x += 2) 
        {
            uint16_t color;
            
            // 数据源判断：从内存读还是从 LCD 硬件读
            if (framebuffer != NULL)
            {
                color = framebuffer[y * width + x];
            }
            else
            {
                /* 
                 * 直接从 LCD 显存读取当前点的颜色值。
                 * 注意：此操作依赖底层驱动支持从 GRAM 读取数据，且 DCMI 传输必须处于暂停状态以防冲突。
                 */
                color = lcd_read_point(x, y);
            }

            /* 
             * RGB565 解码
             * R: bit[15:11] (5位)
             * G: bit[10:5]  (6位)
             * B: bit[4:0]   (5位)
             */
            uint8_t r = (color >> 11) & 0x1F;
            uint8_t g = (color >> 5) & 0x3F;
            uint8_t b = color & 0x1F;

            /* 
             * 归一化到 8 位颜色空间 (0-255) 以便设定阈值。
             * 使用左移补位法模拟真实的 8 位色深。
             */
            r = (r << 3) | (r >> 2);
            g = (g << 2) | (g >> 4);
            b = (b << 3) | (b >> 2);

            /* 
             * 黄色球判定条件：
             * 1. 红色分量较高
					   * 2. 绿色分量较高
						 * 3. 蓝色分量较低
             * 
             * 这是一个基础的静态阈值。在复杂光照下，建议根据实际表现调整这些数值。
             */
            if (r > 150 && g > 120 && b < 100 && r > b + 50 &&  g > b + 40)
            {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    /* 
     * 计算结果处理
     * 设置一个最小像素数 count > 5，用于过滤摄像头噪点。
     */
    if (count > 5) 
    {
        result.found = 1;
        // 计算重心坐标
        result.ball_x = sum_x / count;
        result.ball_y = sum_y / count;

        // 画面中心参考点
        uint16_t center_x = width / 2;
        uint16_t center_y = height / 2;

        /* 
         * 计算偏差 dx, dy
         * dx > 0 表示球在画面右侧，dx < 0 表示球在画面左侧。
         * dy > 0 表示球在画面上方，dy < 0 表示球在画面下方。
         */
        result.dx = (int16_t)result.ball_x - (int16_t)center_x;
        result.dy = (int16_t)center_y - (int16_t)result.ball_y;
    }

    return result;
}

/**
 * @brief       在 LCD 上实时绘制识别结果叠加层
 * @param       ball:   识别结果结构体指针
 * @param       width:  画面宽度
 * @param       height: 画面高度
 */
void LCD_DrawBallOverlay(BallResult_t *ball, uint16_t width, uint16_t height)
{
    uint16_t center_x = width / 2;
    uint16_t center_y = height / 2;

    /* 
     * 1. 绘制静态参考十字 (画面正中心，绿色)
     */
    lcd_draw_hline(center_x - 10, center_y, 20, GREEN);
    lcd_draw_line(center_x, center_y - 10, center_x, center_y + 10, GREEN);

    char buf[64];

    if (ball->found)
    {
        /* 
         * 2. 绘制动态球心十字 (随球移动，红色)
         */
        lcd_draw_hline(ball->ball_x - 10, ball->ball_y, 20, RED);
        lcd_draw_line(ball->ball_x, ball->ball_y - 10, ball->ball_x, ball->ball_y + 10, RED);

        /* 
         * 3. 实时刷新数值显示
         */
        // 显示绝对坐标
        sprintf(buf, "Ball: %3d, %3d", ball->ball_x, ball->ball_y);
        lcd_show_string(10, height - 40, 200, 16, 16, buf, RED);
        
        // 显示相对偏差 (主要用于控制算法)
        sprintf(buf, "dx: %4d, dy: %4d", ball->dx, ball->dy);
        lcd_show_string(10, height - 20, 200, 16, 16, buf, RED);
    }
    else
    {
        /* 
         * 4. 未找到目标时的状态提示
         * 使用空格填充尾部，以清除上一帧残留的数字。
         */
        lcd_show_string(10, height - 40, 200, 16, 16, "Ball: Not Found    ", RED);
        lcd_show_string(10, height - 20, 200, 16, 16, "dx:    0, dy:    0 ", RED);
    }
}


//void UART_SendBallData(BallResult_t *ball)
//{
//    char buf[64];
//    if (ball->found)
//    {
//        // 格式化数据字符串
//        sprintf(buf, "BALL:%d,%d,%d,%d\r\n", ball->ball_x, ball->ball_y, ball->dx, ball->dy);
//    }
//    else
//    {
//        // 发送未找到标志，让子机知道目标丢失
//        sprintf(buf, "BALL:NONE\r\n");
//    }
//    
//    /* 
//     * 调用工程中已有的串口发送函数。
//     * 该函数应封装了 HAL_UART_Transmit 或直接操作寄存器。
//     */
//    UART4_SendString(buf);
//}
