/*
	上位机发送数据的格式为"@dx,dy\r\n"
	一定要是英文模式！！！！！
	使用OLED测试模式下，使用对应OLED测试传输函数
	操控云台前，一定要先对其进行验证，验证后，暂时先选用合适的FOC控制，设置速度，角度
*/

#include "stm32f10x.h"                  // Device header
#include "DFOC.h"

#define    PI    3.14159265359f
//下述定义用于AS5600 的磁铁状态、自动增益、磁场强度，
//在测试时候使用
#define AS5600_STATUS_REG      0x0B
#define AS5600_AGC_REG         0x1A
#define AS5600_MAGNITUDE_H     0x1B
#define AS5600_MAGNITUDE_L     0x1C

int M0_PP = 7, M0_DIR = 1;
int M1_PP = 7, M1_DIR = -1;
char axis;
float angle,dx,dy;
extern Motor_t M0;
extern Motor_t M1;
extern float M0_zero_elc_Angle;
extern float M1_zero_elc_Angle;
extern volatile uint8_t g_ctrl_tick;

/*
 * KEY0 跟踪模式协议（主机 vision.c）：
 *   "@dx,dy\r\n"  — dx/dy 为像素偏差（主机已做 ±10px 死区与方向修正）
 *   "@0,0\r\n"    — 丢球或已居中，云台回初始角
 *
 * 像素 → 弧度：100px ≈ MAX_TRACK_OFFSET rad，现场可改 PIXEL_TO_RAD
 */
#define PIXEL_TO_RAD         0.0045f   /* 100px -> 0.45rad(约26°)，可按实际调小 */
#define MAX_TRACK_OFFSET     0.45f     /* 相对初始角的最大偏转 */


//下述为电机驱控主要操控模块
int main(void)
{
	int16_t raw_dx, raw_dy;
	static float target_m1 = 0.0f;   /* 水平轴 M1 */
	static float target_m0 = 0.0f;   /* 垂直轴 M0 */
	static float M0_home_angle = 0.0f;
	static float M1_home_angle = 0.0f;


	
	/*模块初始化*/
	LED_Init();			//LED初始化
	Serial_Init();		//串口初始化
	Motor_en(); //电机使能
	FOC_Init(12.6);
	Systick_CountMode();
	Ctrl_Timer_Init();          /* 放在 FOC_Init(12.6) 之后 */
	

	/* 记录上电初始角，跟踪目标 = 初始角 + 像素映射偏移 */
	M0_home_angle = (float)M0_DIR * GetAngle(&Angle_Sensor0);
	M1_home_angle = (float)M1_DIR * GetAngle(&Angle_Sensor1);
	target_m0 = M0_home_angle;
	target_m1 = M1_home_angle;

	
	while (1)
	{
		if (Serial_RxFlag == 1) 
		{
			if (sscanf((char *)Serial_RxPacket, "%hd,%hd", &raw_dx, &raw_dy) == 2) 
			{
				if (raw_dx == 0 && raw_dy == 0)
				{
					/* 丢球或主机判定已居中：回到初始角，消除累加漂移 */
					target_m1 = M1_home_angle;
					target_m0 = M0_home_angle;
				}
				else
				{
					float off_x = constrain((float)raw_dx * PIXEL_TO_RAD,
					                        -MAX_TRACK_OFFSET, MAX_TRACK_OFFSET);
					float off_y = constrain((float)raw_dy * PIXEL_TO_RAD,
					                        -MAX_TRACK_OFFSET, MAX_TRACK_OFFSET);
					target_m1 = M1_home_angle + off_x;
					target_m0 = M0_home_angle + off_y;
				}
			}
			Serial_RxFlag = 0;
		}
		
		/* FOC 1kHz，与串口解耦 */
		if (g_ctrl_tick) 
		{
			g_ctrl_tick = 0;
			M1_Set_Velocity_Angle(target_m1);
			M0_Set_Velocity_Angle(target_m0);
		}

	}
}



