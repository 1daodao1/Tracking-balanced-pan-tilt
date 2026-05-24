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


//测试用函数
//uint16_t AS5600_ReadMagnitude(void)
//{
//    uint8_t high = AS5600_ReadReg(AS5600_MAGNITUDE_H);
//    uint8_t low  = AS5600_ReadReg(AS5600_MAGNITUDE_L);

//    return ((high & 0x0F) << 8) | low;
//}

////等 AS5600 有效，再启动电机，测试用函数
//uint8_t AS5600_Magnet_OK(void)
//{
//		Set_Ang_Sensor(1);
//	
//    uint8_t status = AS5600_ReadReg(0x0B);

//    uint8_t md = (status & 0x20) ? 1 : 0;
//    uint8_t ml = (status & 0x10) ? 1 : 0;
//    uint8_t mh = (status & 0x08) ? 1 : 0;

//    if(md == 1 && ml == 0 && mh == 0)
//        return 1;
//    else
//        return 0;
//}

////状态打印函数，测试用函数
//void AS5600_PrintStatus_Mot(int mot)
//{
//    Set_Ang_Sensor(mot);

//    uint8_t status = AS5600_ReadReg(0x0B);
//    uint8_t agc = AS5600_ReadReg(0x1A);
//    uint16_t magnitude = AS5600_ReadMagnitude();

//    uint16_t raw = AS5600_GetRawData();
//    float angle = raw / 4096.0f * 6.2831853f;

//    uint8_t md = (status & 0x20) ? 1 : 0;
//    uint8_t ml = (status & 0x10) ? 1 : 0;
//    uint8_t mh = (status & 0x08) ? 1 : 0;

//    Serial_Printf(
//        "M%d Raw:%d Angle:%.3f STATUS:0x%02X MD:%d ML:%d MH:%d AGC:%d MAG:%d\r\n",
//        mot,
//        raw,
//        angle,
//        status,
//        md,
//        ml,
//        mh,
//        agc,
//        magnitude
//    );
//}

//OLED测试显示用函数
void Format_SignedDecimal1(char *buf, int16_t raw)
{
    int32_t value = raw;
    char sign = '+';

    if (value < 0)
    {
        sign = '-';
			value = -value;//将负数转化为正数
    }

    sprintf(buf, "%c%ld.%ld", sign, value / 10, value % 10);//将格式化的字符串送入buf
}

//下述为电机驱控主要操控模块
//int main(void)
//{
//	/*模块初始化*/
//	LED_Init();			//LED初始化
//	Serial_Init();		//串口初始化
//	Motor_en(); //电机使能
//	FOC_Init(12.6);
//	Systick_CountMode();
	
//	int16_t raw_dy,raw_dx;
//	float dx = 0;
//	float dy = 0;

//	while (1)
//	{
//			
//		if (Serial_RxFlag == 1)		//如果接收到数据包
//		{
//			if (sscanf((char *)Serial_RxPacket, "%hd,%hd", &raw_dx, &raw_dy) == 2)//接收两个短整型成功
//					{
//						           
//
//                // 驱动 dx
//                Format_SignedDecimal1(dx, raw_dx);
//								M1_Set_Velocity(dx);
//								//驱动dy
//                Format_SignedDecimal1(dy, raw_dy);
//                M0_Set_Velocity(dy);
//						
//					}	
//			Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
//		}

//	}
//}

//测试OLED对于字符的显示
int main(void)
{
	/*模块初始化*/
	LED_Init();			//LED初始化
	OLED_Init();
	Serial_Init();		//串口初始化
	int16_t raw_dy,raw_dx;
	float dx = 0;
  float dy = 0;
	char OLED_Buf[16];
	
	OLED_ShowString(1, 1, "H:");
	OLED_ShowString(3, 1, "V:");
	
	while (1)
	{
			
		if (Serial_RxFlag == 1)		//如果接收到数据包
		{
				if (sscanf((char *)Serial_RxPacket, "%hd,%hd", &raw_dx, &raw_dy) == 2)//接收两个短整型成功
            {
                dx = raw_dx / 10.0f;
                dy = raw_dy / 10.0f;

                // 显示 dx
                OLED_ShowString(1, 4, "        ");     // 清除旧数据
                Format_SignedDecimal1(OLED_Buf, raw_dx);
                OLED_ShowString(1, 4, OLED_Buf);

                // 显示 dy
                OLED_ShowString(3, 4, "        ");     // 清除旧数据
                Format_SignedDecimal1(OLED_Buf, raw_dy);
                OLED_ShowString(3, 4, OLED_Buf);
						}
		
			Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
		}

	}
}

/*测试部分*/
//测试as5600是否安装正常
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    AS5600_Init();

//    while(1)
//    {
//        AS5600_PrintStatus_Mot(1);   // 重点：这里是 1，表示 M1

//        Delay_ms(200);
//    }
//}

//测试三相PWM和电机时序是否正常，&测试上电角度读取是否正常
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    Motor_en();
//    FOC_Init(12.6);

//    float open_angle = 0.0f;

//    while(1)
//    {
//        SetPhaseVoltage(&M1, 1.0f, open_angle);

//        open_angle += 0.02f;
//        if(open_angle > 6.2831853f)
//        {
//            open_angle -= 6.2831853f;
//        }

//        //Set_Ang_Sensor(0);
//        uint16_t raw = AS5600_GetRawData();

//        //Serial_Printf("open:%.3f raw:%d\r\n", open_angle, raw);

//        Delay_ms(5);
//    }
//}

//测试M1是否能完成电压测试
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    Motor_en();
//    FOC_Init(12.6);

//    float open_angle = 0.0f;

//    while(1)
//    {
//				
//			SetPhaseVoltage(&M1, 1.5f, M1_electricAngle());
//     
//    }
//}

////低速测试
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    AS5600_Init();

//    while(AS5600_Magnet_OK() == 0)
//    {
//        Serial_SendString("AS5600 magnet error\r\n");
//        Delay_ms(100);
//    }

//    Motor_en();
//    FOC_Init(12.6);
//    Systick_CountMode();
//		
//		

//    while (1)
//    {
//        M0_Set_Velocity(0.1f);


//    }
//}

////直流电压测试
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    AS5600_Init();

//    while(AS5600_Magnet_OK() == 0)
//    {
//        Serial_SendString("AS5600 magnet error\r\n");
//        Delay_ms(100);
//    }

//    Motor_en();
//    FOC_Init(12.6);
//    Systick_CountMode();
//		
//		float open_angle = 0.0f;
//		uint16_t cnt = 0;
//		
//    while (1)
//    {
//			//M0_Set_Velocity_Voltage(1.2f);
//			M1_Set_Velocity(1);
//   

////			open_angle += 0.01f;

////			if(open_angle > 6.2831853f)
////			{
////					open_angle = 0.0f;
////			}
////		
////			    cnt++;
////    if(cnt >= 100)
////    {
////        cnt = 0;

////        Set_Ang_Sensor(0);
////        uint16_t raw = AS5600_GetRawData();

////        float raw_el = M0_rawElectricAngle();
////        float closed_el = M0_electricAngle();

////        Serial_Printf(
////            "open:%.3f raw:%d raw_el:%.3f closed_el:%.3f zero:%.3f\r\n",
////            open_angle,
////            raw,
////            raw_el,
////            closed_el,
////            M0_zero_elc_Angle
////        );
////    }
//			
//			Delay_ms(1);

//    }
//}
