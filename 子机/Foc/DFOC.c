#include "stm32f10x.h"
#include "DFOC.h"


#define PI         3.14159265359f
#define _3PI_2     4.71238898f
#define _1_SQRT3   0.57735026919f
#define _2_SQRT3   1.15470053838f
#define _SQRT3     1.73205080757f // 增加预计算常数，替代 sqrt(3) 库函数调用
volatile uint8_t g_ctrl_tick = 0;

//对于结构体，如果是首元素，则可以如此定义
Motor_t M0 = {0};  
Motor_t M1 = {1}; 

// 角度环 (输出通常为目标速度，需根据 velocity_limit 设定限幅)
PID_t M0_ANG_PID = {
    .Kp = 0.15f, 
	  .Ki = 0.0f,
  	.Kd = 0.002f,
    .OutMax = 1.0f,
    .OutMin = -1.0f,    // 替换为你的速度上限
    .ErrorIntMax = 0, 
	  .ErrorIntMin = 0     // 纯P环无需积分限幅
};
PID_t M1_ANG_PID = {
    .Kp = 0.15f, 
  	.Ki = 0.0f,
  	.Kd = 0.002f,
    .OutMax = 1.0f,
    .OutMin = -1.0f, 
    .ErrorIntMax = 0, 
  	.ErrorIntMin = 0
};

// 速度环 (输出通常为目标电流/交轴电压，需设定电流或电压限幅)
PID_t M0_VEL_PID = {
    .Kp = 0.1f,
  	.Ki = 0.2f,
  	.Kd = 0.0f,
    .OutMax = 1.0f,
  	.OutMin = -1.0f,     // 替换为目标电流/扭矩上限
    .ErrorIntMax = 1.0f, 
   	.ErrorIntMin = -1.0f // 积分限幅通常与输出限幅保持一致或略小
};
PID_t M1_VEL_PID = {
    .Kp = 0.1f,
  	.Ki = 0.2f,
  	.Kd = 0.0f,
    .OutMax = 1.0f,
  	.OutMin = -1.0f,
    .ErrorIntMax = 1.0f,
  	.ErrorIntMin = -1.0f
};

// 电流环 (输出为相电压，绝对不能超过母线电压 voltage_limit)
PID_t M0_CUR_PID = {
    .Kp = 2.0f,
  	.Ki = 50.0f,
  	.Kd = 0.0f,
    .OutMax = 12.0f,
    .OutMin = -12.0f,    
    .ErrorIntMax = 12.0f,
  	.ErrorIntMin = -12.0f 
};
PID_t M1_CUR_PID = {
    .Kp = 2.0f,
  	.Ki = 50.0f,
  	.Kd = 0.0f,
    .OutMax = 12.0f,
    .OutMin = -12.0f,
    .ErrorIntMax = 12.0f,
  	.ErrorIntMin = -12.0f 
};

LowPass_t M0_CUR_Filter = {.Tf = 0.01f};  
LowPass_t M0_VEL_Filter = {.Tf = 0.1f};
LowPass_t M1_CUR_Filter = {.Tf = 0.01f};  
LowPass_t M1_VEL_Filter = {.Tf = 0.1f};

struct AS5600_Sensor Angle_Sensor0 = {0};
struct AS5600_Sensor Angle_Sensor1 = {1};

struct Current_Sensor Current_Sensor0 = {0};
struct Current_Sensor Current_Sensor1 = {1};
  
float voltage_limit = 12.0f;
float voltage_power_supply = 0.0f;
float M0_zero_elc_Angle = 0.0f, M1_zero_elc_Angle = 0.0f;

extern int M0_PP , M0_DIR ;
extern int M1_PP , M1_DIR ;

float velocity_limit = 10.0f;
#define ANG_DEADBAND_RAD  0.03f
//相当于启动电机的总开关
void Motor_en()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

// 限制幅值
float constrain(float amt, float low, float high)
{
     return ((amt < low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// 将角度归化到0-2PI
float normalizeAngle(float angle)
{
    float a = fmod(angle, 2.0f * PI);
    return ((a >= 0.0f) ? a : (a + 2.0f * PI));
}

//原始电角度函数
float M0_rawElectricAngle(void)
{
    return normalizeAngle(GetAngle_NoTrack(&Angle_Sensor0) * M0_PP * M0_DIR);
}

float M1_rawElectricAngle(void)
{
    return normalizeAngle(GetAngle_NoTrack(&Angle_Sensor1) * M1_PP * M1_DIR);
}

float M0_electricAngle(void)
{
    return normalizeAngle(M0_rawElectricAngle() - M0_zero_elc_Angle);
}

float M1_electricAngle(void)
{
     return normalizeAngle(M1_rawElectricAngle() - M1_zero_elc_Angle);
}



void SetPwm(int Mot_num, float Ua, float Ub, float Uc)
{
    float U_a = constrain(Ua, 0.0f, voltage_limit);
    float U_b = constrain(Ub, 0.0f, voltage_limit);
    float U_c = constrain(Uc, 0.0f, voltage_limit);
    
    // 防范除以 0 的风险
    float v_supply_safe = (voltage_power_supply > 0.1f) ? voltage_power_supply : 0.1f;

    if(Mot_num == 0)
    {
        M0.dc_a = constrain(U_a / v_supply_safe, 0.0f, 1.0f);
        M0.dc_b = constrain(U_b / v_supply_safe, 0.0f, 1.0f);
        M0.dc_c = constrain(U_c / v_supply_safe, 0.0f, 1.0f);
        
        M0_PWM_A(M0.dc_a * 4800.0f);  // 频率15k
        M0_PWM_B(M0.dc_b * 4800.0f);
        M0_PWM_C(M0.dc_c * 4800.0f);
    }
    else if(Mot_num == 1)
    {
        M1.dc_a = constrain(U_a / v_supply_safe, 0.0f, 1.0f);
        M1.dc_b = constrain(U_b / v_supply_safe, 0.0f, 1.0f);
        M1.dc_c = constrain(U_c / v_supply_safe, 0.0f, 1.0f);
        
        M1_PWM_A(M1.dc_a * 4800.0f);  // 频率15k
        M1_PWM_B(M1.dc_b * 4800.0f);
        M1_PWM_C(M1.dc_c * 4800.0f);
    }
}

// FOC核心算法，克拉克逆变换/帕克逆变换
void SetPhaseVoltage(Motor_t *Motor, float Uq, float angle_el)
{
    Motor->Ualpha = -Uq * sin(angle_el);
    Motor->Ubeta  =  Uq * cos(angle_el);
    
    // 修正：使用预定义的宏 _SQRT3 替代实时的开方运算，消灭浮点算力瓶颈
    Motor->Ua = Motor->Ualpha + voltage_power_supply / 2.0f;
    Motor->Ub = (_SQRT3 * Motor->Ubeta - Motor->Ualpha) / 2.0f + voltage_power_supply / 2.0f;
    Motor->Uc = -(Motor->Ualpha + _SQRT3 * Motor->Ubeta) / 2.0f + voltage_power_supply / 2.0f;
    
    SetPwm(Motor->Mot_num, Motor->Ua, Motor->Ub, Motor->Uc);
}

//将获取电流值转变为Iq
float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
    float I_alpha = current_a;
    float I_beta  = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    float ct = cos(angle_el);
    float st = sin(angle_el);
    float I_q = I_beta * ct - I_alpha * st;
    
    return I_q;
}

void Check_Sensor(void)
{
    M0_zero_elc_Angle = 0.0f;

    SetPhaseVoltage(&M0, 3.0f, _3PI_2);
    Delay_ms(1000);

    M0_zero_elc_Angle = M0_rawElectricAngle();

    Serial_Printf("M0_zero:%.3f\r\n", M0_zero_elc_Angle);

    SetPhaseVoltage(&M0, 0.0f, _3PI_2);
    Delay_ms(500);
    

}

void Check_Sensor_M1(void)
{
    M1_zero_elc_Angle = 0.0f;

    SetPhaseVoltage(&M1, 3.0f, _3PI_2);
    Delay_ms(1000);

    M1_zero_elc_Angle = M1_rawElectricAngle();

    Serial_Printf("M1_zero:%.3f\r\n", M1_zero_elc_Angle);

    SetPhaseVoltage(&M1, 0.0f, _3PI_2);
    Delay_ms(500);
}


void FOC_Init(float power)
{
    voltage_power_supply = power;
    PWM_Init();
    AD_Init();
    CurrSense_Init(&Current_Sensor0);
    CurrSense_Init(&Current_Sensor1);
    AS5600_Init();
    
		//初始化角度传感器，这里修改
	  Angle_Sensor0.Mot_num = 0;
    Angle_Sensor1.Mot_num = 1;
    AS5600_Sensor_Init(&Angle_Sensor0);
    AS5600_Sensor_Init(&Angle_Sensor1);
	
	  Check_Sensor();
    Check_Sensor_M1();
}

void Ctrl_Timer_Init(void)            /* TIM4 @ 1kHz */
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_Prescaler         = 72 - 1;     /* 72MHz/72 = 1MHz */
    t.TIM_Period            = 1000 - 1;   /* 1MHz/1000 = 1kHz */
    t.TIM_ClockDivision     = TIM_CKD_DIV1;
    t.TIM_CounterMode       = TIM_CounterMode_Up;
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &t);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef n;
    n.NVIC_IRQChannel                   = TIM4_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        g_ctrl_tick = 1;
    }
}


// 单电流环
void M0_Set_CurTorque(float Target)
{
    GetPhaseCurrent(&Current_Sensor0);
    //SetPhaseVoltage(&M0, PID_Controller(&M0_CUR_PID, (Target - Lowpassfilter(&M0_CUR_Filter, cal_Iq_Id(Current_Sensor0.I_a, Current_Sensor0.I_b, M0_electricAngle())))), M0_electricAngle());
		/*获取现在的电角度和三相电流转变为Iq*/
		float el_angle = M0_electricAngle();
    float iq = cal_Iq_Id(Current_Sensor0.I_a, Current_Sensor0.I_b, el_angle);
    float iq_filtered = Lowpassfilter(&M0_CUR_Filter, iq);
	
		/*对于电流值PID变换将其转变为电压值*/
		M0_CUR_PID.Target = Target;
    M0_CUR_PID.Actual = iq_filtered;
    PID_Controller(&M0_CUR_PID);
	
    /*FOC的核心变换*/
    SetPhaseVoltage(&M0, M0_CUR_PID.Out, el_angle);

}

void M1_Set_CurTorque(float Target)
{
    GetPhaseCurrent(&Current_Sensor1);
    //SetPhaseVoltage(&M1, PID_Controller(&M1_CUR_PID, (Target - Lowpassfilter(&M1_CUR_Filter, cal_Iq_Id(Current_Sensor1.I_a, Current_Sensor1.I_b, M1_electricAngle())))), M1_electricAngle());
		float el_angle = M1_electricAngle();
    float iq = cal_Iq_Id(Current_Sensor1.I_a, Current_Sensor1.I_b, el_angle);
    float iq_filtered = Lowpassfilter(&M1_CUR_Filter, iq);
    
    M1_CUR_PID.Target = Target;
    M1_CUR_PID.Actual = iq_filtered;
    PID_Controller(&M1_CUR_PID);
    
    SetPhaseVoltage(&M1, M1_CUR_PID.Out, el_angle);
}

uint16_t print_cnt = 0;

// 速度环+电流环
void M0_Set_Velocity(float Target)
{
    float raw_vel = GetVelocity(&Angle_Sensor0);

    // 速度反馈统一到 M0 正方向
    float vel = M0_DIR * raw_vel;

    Angle_Sensor0.velocity = Lowpassfilter(&M0_VEL_Filter, vel);

    M0_VEL_PID.Target = constrain(Target, -velocity_limit, velocity_limit);
    M0_VEL_PID.Actual = Angle_Sensor0.velocity;
    PID_Controller(&M0_VEL_PID);

    // 这里不要再乘 M0_DIR
    M0_Set_CurTorque(M0_VEL_PID.Out);
}


void M1_Set_Velocity(float Target)
{
    float raw_vel = GetVelocity(&Angle_Sensor1);

    // 关键：速度反馈统一到 M1 正方向
    float vel = M1_DIR * raw_vel;

    Angle_Sensor1.velocity = Lowpassfilter(&M1_VEL_Filter, vel);

    M1_VEL_PID.Target = constrain(Target, -velocity_limit, velocity_limit);
    M1_VEL_PID.Actual = Angle_Sensor1.velocity;
    PID_Controller(&M1_VEL_PID);

    // 关键：这里不要再乘 M1_DIR
    M1_Set_CurTorque(M1_VEL_PID.Out);
}

//速度电压模式，跳过电流环
void M0_Set_Velocity_Voltage(float Target)
{
    float raw_vel = GetVelocity(&Angle_Sensor0);

    // 关键：把 AS5600 原始速度转换成电机正方向速度
    float vel = M0_DIR * raw_vel;

    Angle_Sensor0.velocity = Lowpassfilter(&M0_VEL_Filter, vel);

    M0_VEL_PID.Target = constrain(Target, -velocity_limit, velocity_limit);
    M0_VEL_PID.Actual = Angle_Sensor0.velocity;
    PID_Controller(&M0_VEL_PID);

    float Uq = constrain(M0_VEL_PID.Out, -2.0f, 2.0f);

    SetPhaseVoltage(&M0, Uq, M0_electricAngle());
}


void M1_Set_Velocity_Voltage(float Target)
{
    float raw_vel = GetVelocity(&Angle_Sensor1);

    // 关键：速度反馈统一到 M1 正方向
    float vel = M1_DIR * raw_vel;

    Angle_Sensor1.velocity = Lowpassfilter(&M1_VEL_Filter, vel);

    M1_VEL_PID.Target = constrain(Target, -velocity_limit, velocity_limit);
    M1_VEL_PID.Actual = Angle_Sensor1.velocity;
    PID_Controller(&M1_VEL_PID);

    float Uq = constrain(M1_VEL_PID.Out, -2.0f, 2.0f);

    SetPhaseVoltage(&M1, Uq, M1_electricAngle());
}


//三环
void M0_Set_Velocity_Angle(float Target) 
{
    //M0_Set_Velocity(PID_Controller(&M0_ANG_PID, (Target - GetAngle(&Angle_Sensor0))));
    float actual_angle;
    float err;
	
    // 关键：角度方向也统一到电机正方向
    actual_angle = M0_DIR * GetAngle(&Angle_Sensor0);

		err = Target - actual_angle;

    if (err > -ANG_DEADBAND_RAD && err < ANG_DEADBAND_RAD)
    {
        M0_Set_Velocity(0.0f);
        return;
    }
		
    M0_ANG_PID.Target = Target;
    M0_ANG_PID.Actual = actual_angle;
    PID_Controller(&M0_ANG_PID);

    M0_Set_Velocity(M0_ANG_PID.Out);
/*
	三环含义
	->控制角度（输出速度） 
	-> 控制速度（输出电流） 
	-> 控制电流（输出电压）
	每一环的输出，都是下一环的输入。
	由于下面两个在速度环中以及写了，所以耦合关系
	*/
}

void M1_Set_Velocity_Angle(float Target)
{
    float actual_angle;
		float err;
	
    // 关键：角度反馈统一到 M1 正方向
    actual_angle = M1_DIR * GetAngle(&Angle_Sensor1);
    err = Target - actual_angle;

    if (err > -ANG_DEADBAND_RAD && err < ANG_DEADBAND_RAD)
    {
        M1_Set_Velocity(0.0f);
        return;
    }
    M1_ANG_PID.Target = Target;
    M1_ANG_PID.Actual = actual_angle;
    PID_Controller(&M1_ANG_PID);

    M1_Set_Velocity(M1_ANG_PID.Out);
}

void DataPrint(void)
{
    /*********三相电流波形***********/
    // 修正：将 Current.I_a 修改为合法的 Current_SensorX.I_a 以防取消注释时报错
//  Serial_SendFloatNumber(Current_Sensor0.I_a, 3, 2);
//  Serial_SendString(",");
//  Serial_SendFloatNumber(Current_Sensor0.I_b, 3, 2);  
//  Serial_SendString(",");
//  Serial_SendFloatNumber(Current_Sensor0.I_b + Current_Sensor0.I_a, 3, 2);  

//  Serial_SendFloatNumber(Current_Sensor1.I_a, 3, 2);
//  Serial_SendString(",");
//  Serial_SendFloatNumber(Current_Sensor1.I_b, 3, 2);  
//  Serial_SendString(",");
//  Serial_SendFloatNumber(Current_Sensor1.I_b + Current_Sensor1.I_a, 3, 2);  
    
    /*********实际角度值***********/
    Serial_SendFloatNumber(Angle_Sensor0.Angle, 3, 2);  
    Serial_SendString(",");
    Serial_SendFloatNumber(Angle_Sensor1.Angle, 3, 2);  
    
    /*********实际速度值***********/
//  Serial_SendFloatNumber(Angle_Sensor0.velocity, 3, 2);   
//  Serial_SendString(",");
//  Serial_SendFloatNumber(Angle_Sensor1.velocity, 3, 2);

    Serial_SendString("\n");
}

