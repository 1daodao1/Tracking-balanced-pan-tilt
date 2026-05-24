#ifndef __VISION_H
#define __VISION_H

#include "stdint.h"
#include <stddef.h>

// 识别结果结构体
typedef struct {
    uint8_t found;     // 1: 找到球, 0: 未找到
    uint16_t ball_x;   // 球在图像中的 X 坐标
    uint16_t ball_y;   // 球在图像中的 Y 坐标
    int16_t dx;        // 球心与画面中心的 X 偏差 (ball_x - center_x)
    int16_t dy;        // 球心与画面中心的 Y 偏差 (center_y - ball_y)
} BallResult_t;

// 球位置识别函数 (如果 framebuffer 为 NULL，则直接从 LCD 读取像素)
BallResult_t FindBall_RGB565(uint16_t *framebuffer, uint16_t width, uint16_t height);

// 在 LCD 上叠加显示中心十字、球十字以及坐标信息
void LCD_DrawBallOverlay(BallResult_t *ball, uint16_t width, uint16_t height);

// 通过 UART 发送球的数据给子机
void UART_SendBallData(BallResult_t *ball);

void TrackControl_Reset(void);
void TrackControl_UpdateAndSend(BallResult_t *ball);


#endif /* __VISION_H */
