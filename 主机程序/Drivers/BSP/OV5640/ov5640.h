/**
 ****************************************************************************************************
 * @file        ov5640.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-04-25
 * @brief       OV5640驱动代码
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

#ifndef _OV5640_H
#define _OV5640_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/OV5640/sccb.h"

/* 引脚定义 */
#define OV_PWDN_GPIO_PORT           GPIOC
#define OV_PWDN_GPIO_PIN            GPIO_PIN_4
#define OV_PWDN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define OV_RESET_GPIO_PORT          GPIOA
#define OV_RESET_GPIO_PIN           GPIO_PIN_7
#define OV_RESET_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* IO操作 */
#define OV5640_PWDN(x)              do{ x ? \
                                        HAL_GPIO_WritePin(OV_PWDN_GPIO_PORT, OV_PWDN_GPIO_PIN, GPIO_PIN_SET) : \
                                        HAL_GPIO_WritePin(OV_PWDN_GPIO_PORT, OV_PWDN_GPIO_PIN, GPIO_PIN_RESET); \
                                    }while(0)

#define OV5640_RST(x)               do{ x ? \
                                        HAL_GPIO_WritePin(OV_RESET_GPIO_PORT, OV_RESET_GPIO_PIN, GPIO_PIN_SET) : \
                                        HAL_GPIO_WritePin(OV_RESET_GPIO_PORT, OV_RESET_GPIO_PIN, GPIO_PIN_RESET); \
                                    }while(0)

/* OV5640的ID和访问地址 */
#define OV5640_ID                   0X5640  /* OV5640的芯片ID */
#define OV5640_ADDR                 0X78    /* OV5640的IIC地址 */
 
/* OV5640相关寄存器定义 */
#define OV5640_CHIPIDH              0X300A  /* OV5640芯片ID高字节 */
#define OV5640_CHIPIDL              0X300B  /* OV5640芯片ID低字节 */
 

/* 函数声明 */
uint8_t ov5640_read_reg(uint16_t reg);                                                          /* 读OV5640寄存器 */
uint8_t ov5640_write_reg(uint16_t reg,uint8_t data);                                            /* 写OV5640寄存器 */
uint8_t ov5640_init(void);                                                                      /* 初始化OV5640 */
void ov5640_jpeg_mode(void);                                                                    /* 配置OV5640为JPEG模式 */
void ov5640_rgb565_mode(void);                                                                  /* 配置OV5640为RGB565模式 */
uint8_t ov5640_focus_init(void);                                                                /* 初始化OV5640自动对焦 */
uint8_t ov5640_focus_single(void);                                                              /* OV5640执行一次自动对焦 */
uint8_t ov5640_focus_constant(void);                                                            /* OV5640持续自动对焦 */
void ov5640_flash_ctrl(uint8_t sw);                                                             /* OV5640闪光灯控制 */
void ov5640_test_pattern(uint8_t mode);                                                         /* OV5640测试序列 */
void ov5640_sharpness(uint8_t sharp);                                                           /* 配置OV5640锐度 */
void ov5640_brightness(uint8_t bright);                                                         /* 配置OV5640亮度 */
void ov5640_contrast(uint8_t contrast);                                                         /* 配置OV5640对比度 */
void ov5640_exposure(uint8_t exposure);                                                         /* 配置OV5640 EV曝光补偿 */
void ov5640_light_mode(uint8_t mode);                                                           /* 配置OV5640白平衡 */
void ov5640_special_effects(uint8_t eft);                                                       /* 配置OV5640特效 */
void ov5640_color_saturation(uint8_t sat);                                                      /* 配置OV5640色彩饱和度 */
uint8_t ov5640_outsize_set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);         /* 配置图像输出大小 */
uint8_t ov5640_image_window_set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);    /* 配置图像开窗大小(ISP大小) */

#endif
