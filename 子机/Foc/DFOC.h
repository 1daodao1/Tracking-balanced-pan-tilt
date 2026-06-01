#ifndef __DFOC_H
#define __DFOC_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pwm.h"
#include "AS5600.h"
#include "Delay.h"
#include "Serial.h"
#include "Lowpass_Filter.h"
#include "PID_Control.h"
#include "Inlinecurrent.h"
#include "OLED.h"
#include "LED.h"

typedef struct{
	int Mot_num;
	float Ua;
	float Ub;
	float Uc;
	float Ubeta;
	float Ualpha;
	float dc_a;
	float dc_b;
	float dc_c;
}Motor_t;


void Motor_en(void);
float constrain(float amt, float low, float high);
void SetPwm(int Mot_num, float Ua, float Ub, float Uc);
float normalizeAngle(float angle);
void SetPhaseVoltage(Motor_t *Motor, float Uq, float angle_el);
float cal_Iq_Id(float current_a,float current_b,float angle_el);
void Check_Sensor(void);
void Check_Sensor_M1(void);
void FOC_Init(float power);
float M0_electricAngle(void);
float M1_electricAngle(void);
void M0_Set_Velocity(float Target);
void M0_Set_CurTorque(float Target);
void M1_Set_Velocity(float Target);
void M1_Set_CurTorque(float Target);
void M0_Set_Velocity_Angle(float target);
void M1_Set_Velocity_Angle(float Target);
void DataPrint(void);
float M0_rawElectricAngle(void);
float M1_rawElectricAngle(void);
void M0_Set_Velocity_Voltage(float Target);
void M1_Set_Velocity_Voltage(float Target);
void Ctrl_Timer_Init(void);

extern struct AS5600_Sensor Angle_Sensor0;
extern struct AS5600_Sensor Angle_Sensor1;

#endif


