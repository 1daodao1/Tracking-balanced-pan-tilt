#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern uint8_t Serial_RxPacket[100];				//定义接收数据包数组
extern uint8_t Serial_RxFlag;					//定义接收数据包标志位

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void Serial_SendFloatNumber(float FloatNumber,uint8_t Int_Length, uint8_t Float_Length);

#endif
