#include "LowPass_Filter.h"
#include "stm32f10x.h" // 确保 SysTick 可用

// 初始化滤波器，传入时间常数
void LowPassFilter_Init(LowPass_t *p, float time_constant)
{
    p->Tf = time_constant;
    p->Last_Timesamp = SysTick->VAL; // 初始化为当前时间，避免第一次计算得到巨大的 dt
    p->Last_y = 0.0f;
}

// 使用公式: y[n] = alpha * y[n-1] + (1 - alpha) * x[n]
float Lowpassfilter_sim(float x, float *prev_out, float alpha)
{
    float out = alpha * (*prev_out) + (1.0f - alpha) * x;
    *prev_out = out;
    return out;
}

// 基于 SysTick 动态计算时间步长的实装函数
float Lowpassfilter(LowPass_t *LowPass, float x)
{
    float dt = 0.0f;
    uint32_t Timesamp = SysTick->VAL;
    
    // Cortex-M SysTick 是向下计数 (Down-counter)
    if(Timesamp <= LowPass->Last_Timesamp) 
    {
        dt = (float)(LowPass->Last_Timesamp - Timesamp) / 9.0f * 1e-6f;
    }
    else // 发生溢出
    {
        // 实际消耗的时间 = 上次剩余的计数值 + (重载值 - 当前计数值)
        dt = (float)(LowPass->Last_Timesamp + (SYSTICK_RELOAD_VAL - Timesamp)) / 9.0f * 1e-6f;
    }

    // 异常步长处理
    if(dt <= 0.0f) 
    {
        dt = 0.0015f; 
    }
    else if(dt > 0.005f)
    {
        // 如果步长过大，认为系统状态不连续，直接透传数据并重置状态
        LowPass->Last_y = x;
        LowPass->Last_Timesamp = Timesamp;
        return x;
    }
    
    // 计算动态滤波系数
    float alpha = LowPass->Tf / (LowPass->Tf + dt);
    
    // 一阶 IIR 滤波计算y_n = \alpha y_{n-1} + (1-\alpha) x_n
    float y = alpha * LowPass->Last_y + (1.0f - alpha) * x;
    
    LowPass->Last_y = y;
    LowPass->Last_Timesamp = Timesamp;

    return y;
}
