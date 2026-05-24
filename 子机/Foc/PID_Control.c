#include "stm32f10x.h"
#include "Serial.h"
#include "PID_Control.h"

#define SYSTICK_RELOAD_VALUE 0xFFFFFF

void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Actual1 = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	p->Timestamp_Last = SysTick->VAL; // 锚定初始时间
}

void PID_Controller(PID_t *p)
{
    float dt = 0.0f;
    uint32_t current_time = SysTick->VAL;
    
    // 1. 动态时间步长计算 (向下计数逻辑)
    if (current_time <= p->Timestamp_Last) 
    {
        dt = (float)(p->Timestamp_Last - current_time) / 9.0f * 1e-6f;
    }
    else 
    {
        // 发生溢出重载的补偿计算
        dt = (float)(p->Timestamp_Last + (SYSTICK_RELOAD_VALUE - current_time)) / 9.0f * 1e-6f;
    }

    // 2. 异常步长阻断 (规避除零风险与系统级阻塞)
    if (dt <= 0.0f || dt > 0.05f) 
    {
        dt = 0.001f; // 提供安全的默认步长推演
    }
    
    p->Timestamp_Last = current_time; // 更新时间戳

    // 3. 核心误差推导
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    
    // 4. 带时间系数的积分项运算 (标准矩形积分)
    if (p->Ki != 0.0f)
    {
        // 逻辑修正：严格使用 dt 乘以 当前误差
        p->ErrorInt += p->Ki * p->Error0 * dt; 
        
        if(p->ErrorInt > p->ErrorIntMax) { p->ErrorInt = p->ErrorIntMax; }
        if(p->ErrorInt < p->ErrorIntMin) { p->ErrorInt = p->ErrorIntMin; }
    }
    else
    {
        p->ErrorInt = 0.0f;
    }
    
    // 5. 控制律合成
    // 比例项：不变
    // 积分项：已在上方计算并限幅，直接代入
    // 微分项：微分先行策略 + 引入时间分母
    p->Out = (p->Kp * p->Error0)
           + (p->ErrorInt)
           - (p->Kd * (p->Actual - p->Actual1) / dt);
    
    // 6. 输出死区偏移
    if (p->Out > 0.0f) { p->Out += p->OutOffset; }
    else if (p->Out < 0.0f) { p->Out -= p->OutOffset; }
    
    // 7. 整体输出限幅
    if (p->Out > p->OutMax) { p->Out = p->OutMax; }
    if (p->Out < p->OutMin) { p->Out = p->OutMin; }
    
    // 8. 状态迭代
    p->Actual1 = p->Actual;
}

