#ifndef __AS5600_H
#define __AS5600_H

struct AS5600_Sensor{
	int Mot_num;
	float Angle;
	float velocity;
	float full_rotations;
	float Last_Angle;
	float Last_Vel_ts;
	float Vel_Last_Angle;
};


void Set_Ang_Sensor(int Mot);
void Set_Sensor(int Mot);
void I2C_W_SCL(uint8_t BitValue);
void I2C_W_SDA(uint8_t BitValue);
uint8_t I2C_R_SDA(void);
void MyI2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t Byte);
uint8_t I2C_RecviveData(void);
void I2C_SendAck(uint8_t AckBit);
uint8_t I2C_RecviveAck(void);
uint8_t  AS5600_ReadReg(uint8_t RegAddress);
void AS5600_Init(void);
uint16_t AS5600_GetRawData(void);
float GetAngle(struct AS5600_Sensor *AS5600);
float GetAngle_NoTrack(struct AS5600_Sensor *AS5600);
float GetVelocity(struct AS5600_Sensor *AS5600_Vel);
void AS5600_Sensor_Init(struct AS5600_Sensor *AS5600);
	

#endif
