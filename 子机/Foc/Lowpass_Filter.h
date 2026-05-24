#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

#include <stdint.h>

/* 必须根据实际情况修改此宏。若是满量程则为 0xFFFFFF */
#define SYSTICK_RELOAD_VAL 0xFFFFFF 

typedef struct
{
    float Tf;               /* 时间常数，等于 1/(2*pi*fc) */
    uint32_t Last_Timesamp; /* 上次计算的系统时钟戳，必须为 uint32_t */
    float Last_y;           /* 上一次的输出值 */
} LowPass_t;

void LowPassFilter_Init(LowPass_t *p, float time_constant);
float Lowpassfilter_sim(float x, float *prev_out, float alpha);
float Lowpassfilter(LowPass_t *LowPass, float x);

#endif
