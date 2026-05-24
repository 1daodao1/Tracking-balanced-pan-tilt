/**
 ****************************************************************************************************
 * @file        dcmi.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-04-25
 * @brief       DCMI驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M100Z-M7最小系统板STM32H750版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#ifndef _DCMI_H
#define _DCMI_H

#include "./SYSTEM/sys/sys.h"

/* DCMI DMA接收回调函数,需要用户实现该函数 */
extern void (*dcmi_rx_callback)(void);
extern DCMI_HandleTypeDef g_dcmi_handle;    /* DCMI句柄 */
extern DMA_HandleTypeDef g_dma_dcmi_handle; /* DMA句柄 */

/* 函数声明 */
void dcmi_init(void);                                                                                       /* 初始化DCI */
void dcmi_dma_init(uint32_t mem0addr,uint32_t mem1addr,uint16_t memsize,uint32_t memblen,uint32_t meminc);  /* 配置DCMI DMA */
void dcmi_start(void);                                                                                      /* 启动DCMI传输 */
void dcmi_stop(void);                                                                                       /* 停止DCMI传输 */
void dcmi_set_window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);                               /* 设置DCMI显示窗口 */
void dcmi_cr_set(uint8_t pclk,uint8_t hsync,uint8_t vsync);                                                 /* DCMI调试 */

#endif
