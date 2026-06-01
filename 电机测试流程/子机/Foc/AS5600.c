#include "stm32f10x.h"
#include "Serial.h"
#include "Delay.h"
#include <math.h>
#include "Lowpass_Filter.h"
#include "AS5600.h"

#define AS5600_ADDRESS        0x6C //加上读写位（1位1/0）
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D

#define PI       3.14159265359f
#define _2PI     6.28318530718f

uint16_t SCL_PIN, SDA_PIN;

void Set_Ang_Sensor(int Mot)
{
	if(Mot == 0)
	{
		SCL_PIN = GPIO_Pin_10;
		SDA_PIN = GPIO_Pin_11;
	}
	else if(Mot == 1)
	{
		SCL_PIN = GPIO_Pin_6;
		SDA_PIN = GPIO_Pin_7;
	}
}

void I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SCL_PIN, (BitAction)BitValue);
}

void I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SDA_PIN, (BitAction)BitValue);
}

//读取时钟线数据
uint8_t I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, SDA_PIN);

	return BitValue;
}

void MyI2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_6|GPIO_Pin_7);
}
	
void I2C_Start(void)
{
	I2C_W_SCL(1);
	I2C_W_SDA(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C_Stop(void)
{
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);
}

void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		I2C_W_SDA(Byte & (0x80>>i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}
}

uint8_t I2C_RecviveData(void)
{
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(1);
	for(i=0;i<8;i++)
	{
		I2C_W_SCL(1);
		if(I2C_R_SDA() == 1) {Byte |= (0x80 >> i);}
		I2C_W_SCL(0);
	}
	return Byte;
}

void I2C_SendAck(uint8_t AckBit)
{
		I2C_W_SDA(AckBit);
		I2C_W_SCL(1);
		I2C_W_SCL(0);
}

uint8_t I2C_RecviveAck(void)
{
	uint8_t AckBit;

	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);
	
	return AckBit;
}

uint8_t AS5600_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	I2C_RecviveAck();
	I2C_SendByte(RegAddress);
	I2C_RecviveAck();
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();
	
	return Data;
}

uint16_t AS5600_GetRawData(void)
{
    uint8_t Data_H;
    uint8_t Data_L;
    uint16_t Raw_Data;

    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS);          // 0x6C write
    I2C_RecviveAck();

    I2C_SendByte(AS5600_RAW_ANGLE_H);      // 0x0C
    I2C_RecviveAck();

    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS | 0x01);   // 0x6D read
    I2C_RecviveAck();

    Data_H = I2C_RecviveData();
    I2C_SendAck(0);                        // ACK，继续读

    Data_L = I2C_RecviveData();
    I2C_SendAck(1);                        // NACK，结束

    I2C_Stop();

    Raw_Data = ((Data_H & 0x0F) << 8) | Data_L;

    return Raw_Data;
}

void AS5600_Init(void)
{
	MyI2C_Init();
}


float GetAngle_NoTrack(struct AS5600_Sensor *AS5600)
{
	Set_Ang_Sensor(AS5600->Mot_num);
	float Angle = 0.0;
 
//	uint8_t DataH = 0;
//	uint8_t DataL = 0;
//	DataH = AS5600_ReadReg(AS5600_RAW_ANGLE_H);
//	DataL = AS5600_ReadReg(AS5600_RAW_ANGLE_L);
	
//	Angle = (DataH << 8) | DataL;
	//是否需要-2派看角度问题，修改与5.23
	Angle = ((AS5600_GetRawData() / 4096.0f) * _2PI) ;
	//	Angle = (Angle/4096) * 360;
	
//	Angle = (AS5600_GetRawData()/4096) * _2PI;
	
	return Angle;
}

float GetAngle(struct AS5600_Sensor *AS5600)
{
//	float D_Angle = 0.0;
	AS5600->Angle = GetAngle_NoTrack(AS5600);
	float D_Angle = AS5600->Angle - AS5600->Last_Angle;

	if( fabs(D_Angle) > (0.8f*2*PI) )
	{
		AS5600->full_rotations = AS5600->full_rotations + ((D_Angle > 0) ? -1 :1);
	}
	
	AS5600->Last_Angle = AS5600->Angle;
	
	AS5600->Angle = (AS5600->full_rotations * _2PI + AS5600->Last_Angle);
	
	return AS5600->Angle;
}

float GetVelocity(struct AS5600_Sensor *AS5600_Vel)
{
	float dt = 0.0;
	float Vel_ts = SysTick -> VAL;
	if(Vel_ts < AS5600_Vel->Last_Vel_ts) dt = (AS5600_Vel->Last_Vel_ts - Vel_ts)/9*1e-6f;
	else dt = (0xFFFFFF - Vel_ts + AS5600_Vel->Last_Vel_ts)/9*1e-6f;
	
	//此处替换了if(dt < 0.0001) dt = 10000;
		if(dt <= 0.0f || dt > 0.05f)
		{
			dt = 0.001f;
		}
		else if(dt < 0.0001f)
		{
			return AS5600_Vel->velocity;   // 时间太短，直接返回上一次速度
		}
	
	float Vel_Angle = GetAngle(AS5600_Vel);
	
	float dv = Vel_Angle - AS5600_Vel->Vel_Last_Angle;

	AS5600_Vel->velocity = (Vel_Angle - AS5600_Vel->Vel_Last_Angle)/dt;

	AS5600_Vel->Last_Vel_ts = Vel_ts;
  AS5600_Vel->Vel_Last_Angle = Vel_Angle;
  
  return AS5600_Vel->velocity;
}

////记录初始角度
void AS5600_Sensor_Init(struct AS5600_Sensor *AS5600)
{
    Set_Ang_Sensor(AS5600->Mot_num);

    AS5600->Angle = GetAngle_NoTrack(AS5600);
    AS5600->Last_Angle = AS5600->Angle;
    AS5600->Vel_Last_Angle = AS5600->Angle;
    AS5600->full_rotations = 0;
    AS5600->velocity = 0.0f;
}
