#ifndef __MYTASK_H
#define __MYTASK_H

#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./USMART/usmart.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/btim.h"
#include "./BSP/DCMI/dcmi.h"
#include "./BSP/OV5640/ov5640.h"
#include "./BSP/USART2/usart2.h"
#include "./BSP/USART4/usart4.h"
#include "./BSP/VISION/vision.h"
#include "./BSP/MPU6050/MPU6050.h"
#include <stdio.h>
#include <math.h>


extern uint8_t g_ov_mode;                  /* bit0: 0,RGB565模式;  1,JPEG模式 */
extern uint16_t g_curline;                 /* 摄像头输出数据,当前行编号 */
extern uint16_t g_yoffset;                 /* y方向的偏移量 */


void jpeg_data_process(void);
void jpeg_dcmi_rx_callback(void);
void balance_test(void);
void rgb565_test(void);



#endif
