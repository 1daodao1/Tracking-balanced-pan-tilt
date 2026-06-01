/*
	上位机发送数据的格式为"@dx,dy\r\n"
	一定要是英文模式！！！！！
	使用OLED测试模式下，使用对应OLED测试传输函数
	操控云台前，一定要先对其进行验证，验证后，暂时先选用合适的FOC控制，设置速度，角度
*/

#include "stm32f10x.h"                  // Device header
#include "DFOC.h"

#define    PI    3.14159265359f
#define    _2PI  6.28318530718f

//下述定义用于AS5600 的磁铁状态、自动增益、磁场强度，
//在测试时候使用
#define AS5600_STATUS_REG      0x0B
#define AS5600_AGC_REG         0x1A
#define AS5600_MAGNITUDE_H     0x1B
#define AS5600_MAGNITUDE_L     0x1C

int M0_PP = 7, M0_DIR = -1;
int M1_PP = 7, M1_DIR = -1;
char axis;
float angle,dx,dy;
extern Motor_t M0;
extern Motor_t M1;
extern float M0_zero_elc_Angle;
extern float M1_zero_elc_Angle;
extern PID_t M0_ANG_PID;
extern PID_t M0_VEL_PID;
extern PID_t M0_CUR_PID;
extern PID_t M1_ANG_PID;
extern PID_t M1_VEL_PID;
extern PID_t M1_CUR_PID;
extern struct AS5600_Sensor Angle_Sensor0;


//测试用函数
uint16_t AS5600_ReadMagnitude(void)
{
    uint8_t high = AS5600_ReadReg(AS5600_MAGNITUDE_H);
    uint8_t low  = AS5600_ReadReg(AS5600_MAGNITUDE_L);

    return ((high & 0x0F) << 8) | low;
}

////等 AS5600 有效，再启动电机，测试用函数
uint8_t AS5600_Magnet_OK(void)
{
		Set_Ang_Sensor(0);
	
    uint8_t status = AS5600_ReadReg(0x0B);

    uint8_t md = (status & 0x20) ? 1 : 0;
    uint8_t ml = (status & 0x10) ? 1 : 0;
    uint8_t mh = (status & 0x08) ? 1 : 0;

    if(md == 1 && ml == 0 && mh == 0)
        return 1;
    else
        return 0;
}

////状态打印函数，测试用函数
void AS5600_PrintStatus_Mot(int mot)
{
    Set_Ang_Sensor(mot);

    uint8_t status = AS5600_ReadReg(0x0B);
    uint8_t agc = AS5600_ReadReg(0x1A);
    uint16_t magnitude = AS5600_ReadMagnitude();

    uint16_t raw = AS5600_GetRawData();
    float angle = raw / 4096.0f * 6.2831853f;

    uint8_t md = (status & 0x20) ? 1 : 0;
    uint8_t ml = (status & 0x10) ? 1 : 0;
    uint8_t mh = (status & 0x08) ? 1 : 0;

    Serial_Printf(
        "M%d Raw:%d Angle:%.3f STATUS:0x%02X MD:%d ML:%d MH:%d AGC:%d MAG:%d\r\n",
        mot,
        raw,
        angle,
        status,
        md,
        ml,
        mh,
        agc,
        magnitude
    );
}

//磁通量测试函数
void AS5600_OneTurn_MagTest(int mot, float voltage, uint16_t sample_points)
{
    uint16_t i;
    uint16_t bad_count = 0;

    uint16_t raw_min = 4095;
    uint16_t raw_max = 0;

    uint8_t agc_min = 255;
    uint8_t agc_max = 0;

    uint16_t mag_min = 4095;
    uint16_t mag_max = 0;

    int pp;
    int dir;
    Motor_t *motor;

    if(mot == 0)
    {
        pp = M0_PP;
        dir = M0_DIR;
        motor = &M0;
    }
    else
    {
        pp = M1_PP;
        dir = M1_DIR;
        motor = &M1;
    }

    Serial_Printf("\r\n===== AS5600 M%d One Turn Magnet Test Start =====\r\n", mot);

    for(i = 0; i <= sample_points; i++)
    {
        float mech_angle = _2PI * ((float)i / sample_points);
        float elec_angle = mech_angle * pp * dir;

        // 开环拖动电机到指定机械角度
        SetPhaseVoltage(motor, voltage, elec_angle);

        // 等电机稳定一下
        Delay_ms(20);

        Set_Ang_Sensor(mot);

        uint8_t status = AS5600_ReadReg(AS5600_STATUS_REG);
        uint8_t agc = AS5600_ReadReg(AS5600_AGC_REG);
        uint16_t mag = AS5600_ReadMagnitude();
        uint16_t raw = AS5600_GetRawData();

        uint8_t md = (status & 0x20) ? 1 : 0;
        uint8_t ml = (status & 0x10) ? 1 : 0;
        uint8_t mh = (status & 0x08) ? 1 : 0;

        uint8_t ok = 1;

        if(md == 0) ok = 0;   // 没检测到磁铁
        if(ml == 1) ok = 0;   // 磁场太弱
        if(mh == 1) ok = 0;   // 磁场太强

        // 经验判断：AGC 太接近 0 或 255 都不理想
        if(agc < 10 || agc > 245) ok = 0;

        if(ok == 0)
        {
            bad_count++;
        }

        if(raw < raw_min) raw_min = raw;
        if(raw > raw_max) raw_max = raw;

        if(agc < agc_min) agc_min = agc;
        if(agc > agc_max) agc_max = agc;

        if(mag < mag_min) mag_min = mag;
        if(mag > mag_max) mag_max = mag;

        Serial_Printf(
            "idx:%3d mech:%6.3f raw:%4d status:0x%02X MD:%d ML:%d MH:%d AGC:%3d MAG:%4d %s\r\n",
            i,
            mech_angle,
            raw,
            status,
            md,
            ml,
            mh,
            agc,
            mag,
            ok ? "OK" : "BAD"
        );
    }

    // 测试结束，关闭输出
    SetPhaseVoltage(motor, 0.0f, 0.0f);

    Serial_Printf("===== AS5600 M%d One Turn Magnet Test Result =====\r\n", mot);
    Serial_Printf("RAW min:%d max:%d span:%d\r\n", raw_min, raw_max, raw_max - raw_min);
    Serial_Printf("AGC min:%d max:%d span:%d\r\n", agc_min, agc_max, agc_max - agc_min);
    Serial_Printf("MAG min:%d max:%d span:%d\r\n", mag_min, mag_max, mag_max - mag_min);
    Serial_Printf("BAD count:%d / %d\r\n", bad_count, sample_points + 1);

    if(bad_count == 0)
    {
        Serial_SendString("RESULT: PASS\r\n");
    }
    else
    {
        Serial_SendString("RESULT: FAIL, check magnet distance/centering/strength\r\n");
    }

    Serial_SendString("==============================================\r\n\r\n");
}


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

////测试OLED对于字符的显示
//int main(void)
//{
//	/*模块初始化*/
//	LED_Init();			//LED初始化
//	OLED_Init();
//	Serial_Init();		//串口初始化
//	int16_t raw_dy,raw_dx;
//	float dx = 0;
//  float dy = 0;
//	char OLED_Buf[16];
//	
//	OLED_ShowString(1, 1, "H:");
//	OLED_ShowString(3, 1, "V:");
//	
//	while (1)
//	{
//			
//		if (Serial_RxFlag == 1)		//如果接收到数据包
//		{
//				if (sscanf((char *)Serial_RxPacket, "%hd,%hd", &raw_dx, &raw_dy) == 2)//接收两个短整型成功
//            {
//                dx = raw_dx / 10.0f;
//                dy = raw_dy / 10.0f;

//                // 显示 dx
//                OLED_ShowString(1, 4, "        ");     // 清除旧数据
//                Format_SignedDecimal1(OLED_Buf, raw_dx);
//                OLED_ShowString(1, 4, OLED_Buf);

//                // 显示 dy
//                OLED_ShowString(3, 4, "        ");     // 清除旧数据
//                Format_SignedDecimal1(OLED_Buf, raw_dy);
//                OLED_ShowString(3, 4, OLED_Buf);
//						}
//		
//			Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
//		}

//	}
//}

/*测试部分*/
//第一步
//测试as5600是否安装正常
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    AS5600_Init();

//    while(1)
//    {
//        AS5600_PrintStatus_Mot(0);   // 重点：这里是 1，表示 M1

//        Delay_ms(200);
//    }
//}

//磁通量测试主函数
//第二步
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    AS5600_Init();

//    Motor_en();
//    FOC_Init(12.6);

//    Delay_ms(1000);

//    while(1)
//    {
//        // 测试 M0：
//        // 参数含义：电机编号，开环电压，采样点数
//        // 72 表示一圈采 72 个点，也就是每 5 度采一次
//        AS5600_OneTurn_MagTest(0, 1.0f, 72);

//        Delay_ms(3000);
//    }
//}


//测试三相PWM和电机时序是否正常，&测试上电角度读取是否正常
//第三步
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    Motor_en();
//    FOC_Init(12.6);

//    float open_angle = 0.0f;

//    while(1)
//    {
//        SetPhaseVoltage(&M0, 1.0f, open_angle);

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

//第四步，验证AS5600 方向是否和电机一致
/*
	diff > 0  => M0_DIR =  1;
	diff < 0  => M0_DIR = -1;
*/
int16_t RawDiff(uint16_t now, uint16_t last)
{
    int16_t diff = (int16_t)now - (int16_t)last;

    if(diff > 2048) diff -= 4096;
    if(diff < -2048) diff += 4096;

    return diff;
}

//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    Motor_en();
//    FOC_Init(12.6);     // 注意：FOC_Init 里先注释 Check_Sensor_M1()

//    while(1)
//    {
//        uint16_t raw1, raw2;
//        int16_t diff;

//        SetPhaseVoltage(&M0, 1.5f, 0.0f);
//        Delay_ms(1000);
//        Set_Ang_Sensor(0);
//        raw1 = AS5600_GetRawData();

//        SetPhaseVoltage(&M0, 1.5f, 1.0f);   // 电角度正方向跳变
//        Delay_ms(1000);
//        Set_Ang_Sensor(0);
//        raw2 = AS5600_GetRawData();

//        diff = RawDiff(raw2, raw1);

//        Serial_Printf("raw1:%d raw2:%d diff:%d DIR:%d\r\n",
//                      raw1,
//                      raw2,
//                      diff,
//                      diff >= 0 ? 1 : -1);

//        SetPhaseVoltage(&M0, 0.0f, 0.0f);
//        Delay_ms(2000);
//    }
//}

////第五步，低速测试
//int main(void)
//{
//    LED_Init();
//    Serial_Init();

//    Motor_en();
//    FOC_Init(12.6);          // 里面先注释 Check_Sensor_M1()
//    Systick_CountMode();

//    while(1)
//    {
//        M0_Set_Velocity_Voltage(0.5f);   // 先低速
//        Delay_ms(1);
//    }
//}

//第六步，M0 正反转低速测试
//void M0_RunVelocity(float target, uint32_t time_ms)
//{
//    uint32_t i;

//    for(i = 0; i < time_ms; i++)
//    {
//        M0_Set_Velocity(target);
//        Delay_ms(1);
//    }
//}

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

//    while(1)
//    {
//        M0_RunVelocity(1.0f, 3000);

//        M0_RunVelocity(0.0f, 1000);

//        M0_RunVelocity(-1.0f, 3000);

//        M0_RunVelocity(0.0f, 1000);
//    }
//}

//第七步，M1 角度环测试。
void M1_AngleLoop_Print(uint16_t period_ms)
{
    static uint16_t cnt = 0;

    if(period_ms == 0)
    {
        period_ms = 500;
    }

    cnt++;
    if(cnt < period_ms)
    {
        return;
    }
    cnt = 0;

    Serial_Printf(
        "T:%.3f A:%.3f E:%.3f V:%.3f\r\n",
        M1_ANG_PID.Target,
        M1_ANG_PID.Actual,
        M1_ANG_PID.Target - M0_ANG_PID.Actual,
        M1_ANG_PID.Out
    );
}


void M1_RunAngle(float target, uint32_t time_ms)
{
    uint32_t i;

    for(i = 0; i < time_ms; i++)
    {
        M1_Set_Velocity_Angle(target);

        // 每 500ms 打印一次，降低串口压力
        M1_AngleLoop_Print(500);

        Delay_ms(1);
    }
}


int main(void)
{
    LED_Init();
    Serial_Init();

    AS5600_Init();

    while(AS5600_Magnet_OK() == 0)
    {
        Serial_SendString("AS5600 magnet error\r\n");
        Delay_ms(100);
    }

    Motor_en();
    FOC_Init(12.6);
    Systick_CountMode();
		
		// 关键：FOC_Init 里面 Check_Sensor 已经拖动过电机
    // 所以这里必须重新初始化角度跟踪
    AS5600_Sensor_Init(&Angle_Sensor1);
		
    Delay_ms(500);

    float center = M1_DIR * GetAngle(&Angle_Sensor1);

		Serial_Printf("center:%.3f\r\n", center);
		
		while(1)
		{
			//这里的每一步测试是为了看看是否是由于卡进串口当中
				Serial_SendString("STEP 1\r\n");
				M1_RunAngle(center, 1000);

				Serial_SendString("STEP 2\r\n");
				M1_RunAngle(center + 0.4f, 3000);

				Serial_SendString("STEP 3\r\n");
				M1_RunAngle(center, 3000);

				Serial_SendString("STEP 4\r\n");
				M1_RunAngle(center - 0.4f, 3000);

				Serial_SendString("STEP 5\r\n");
				M1_RunAngle(center, 3000);
		}

}






















//测试M0是否能完成电压测试
//测试第四步
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
//			SetPhaseVoltage(&M0, 1.5f, M0_electricAngle());
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
//        M0_Set_Velocity(1);


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
