#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	//积分限幅
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;//输出偏移值
	
	uint32_t Timestamp_Last; // 必须使用无符号整型避免溢出截断
	
} PID_t;

void PID_Controller(PID_t *p);
void PID_Init(PID_t *p);


#endif


