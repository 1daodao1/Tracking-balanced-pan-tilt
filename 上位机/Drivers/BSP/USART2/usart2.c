/**
 ****************************************************************************************************
 * @file        usart2.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-04-25
 * @brief       USART2驱动代码
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

#include "./BSP/USART2/usart2.h"
#include "./BSP/LCD/lcd.h"
#include <stdio.h>

UART_HandleTypeDef g_uart2_handle;  /* USART2句柄 */

/**
 * @brief       串口2初始化函数
 * @note        注意:串口2的时钟源频率在sys已经设置过了
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 */
void usart2_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    /* 使能时钟 */
    __HAL_RCC_USART2_CLK_ENABLE();                          /* 使能USART2时钟 */
    USART2_TX_GPIO_CLK_ENABLE();                            /* 使能串口TX脚时钟 */
    USART2_RX_GPIO_CLK_ENABLE();                            /* 使能串口RX脚时钟 */
    
    /* 配置USART2_TX引脚 */
    gpio_init_struct.Pin = USART2_TX_GPIO_PIN;              /* USART2_TX引脚 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速 */
    gpio_init_struct.Alternate = USART2_TX_GPIO_AF;         /* 复用为USART2 */
    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &gpio_init_struct);  /* 初始化串口TX引脚 */
    
    /* 配置USART2_RX引脚 */
    gpio_init_struct.Pin = USART2_RX_GPIO_PIN;              /* USART2_RX引脚 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速 */
    gpio_init_struct.Alternate = USART2_RX_GPIO_AF;         /* 复用为USART2 */
    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &gpio_init_struct);  /* 初始化串口RX引脚 */
    
    /* 初始化串口 */
    g_uart2_handle.Instance = USART2;                       /* USART2 */
    g_uart2_handle.Init.BaudRate = baudrate;                /* 波特率 */
    g_uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;    /* 字长为8位数据格式 */
    g_uart2_handle.Init.StopBits = UART_STOPBITS_1;         /* 一个停止位 */
    g_uart2_handle.Init.Parity = UART_PARITY_NONE;          /* 无奇偶校验位 */
    g_uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;    /* 无硬件流控 */
    g_uart2_handle.Init.Mode = UART_MODE_TX_RX;                /* 发模式 */
    HAL_UART_Init(&g_uart2_handle);                         /* 使能USART2 */
		
		/* 开启接收中断 */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_RXNE);
}

/* 接收缓冲区和状态变量 */
uint8_t g_usart2_rx_buf[7];
uint8_t g_usart2_rx_cnt = 0;

/**
 * @brief       USART2中断服务函数
 */
void USART2_IRQHandler(void)
{
    uint8_t res;
    uint32_t isrflags = USART2->ISR;
    
    // 处理接收中断
    if ((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
    {
        res = (uint8_t)(USART2->RDR); // 直接读取寄存器，避免HAL状态冲突
        
        // 解析数据帧: 0xAA 0x55 dx_L dx_H dy_L dy_H 0x5A
        if (g_usart2_rx_cnt == 0 && res == 0xAA) {
            g_usart2_rx_buf[g_usart2_rx_cnt++] = res;
        } else if (g_usart2_rx_cnt == 1 && res == 0x55) {
            g_usart2_rx_buf[g_usart2_rx_cnt++] = res;
        } else if (g_usart2_rx_cnt >= 2 && g_usart2_rx_cnt < 6) {
            g_usart2_rx_buf[g_usart2_rx_cnt++] = res;
        } else if (g_usart2_rx_cnt == 6 && res == 0x5A) {
            g_usart2_rx_buf[g_usart2_rx_cnt++] = res;
            
            // 接收完成，转发给UART4
            extern UART_HandleTypeDef huart4;
            HAL_UART_Transmit(&huart4, g_usart2_rx_buf, 7, 10); // 设置较短超时
							
						//extern UART_HandleTypeDef g_uart1_handle;
						//HAL_UART_Transmit(&g_uart1_handle, g_usart2_rx_buf, 7, 10);
					  // 3. 在LCD屏幕上显示
            int16_t dx = (int16_t)((g_usart2_rx_buf[3] << 8) | g_usart2_rx_buf[2]);
            int16_t dy = (int16_t)((g_usart2_rx_buf[5] << 8) | g_usart2_rx_buf[4]);
            char lcd_buf[32];
            sprintf(lcd_buf, "dx: %-5d dy: %-5d", dx, dy);
            lcd_show_string(30, 260, 200, 16, 16, lcd_buf, RED);
					
            g_usart2_rx_cnt = 0;
        } else {
            g_usart2_rx_cnt = 0;
            if (res == 0xAA) {
                g_usart2_rx_buf[g_usart2_rx_cnt++] = res;
            }
        }
    }
    
    // 清除错误标志 (ORE, FE, NE, PE)
    if ((isrflags & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) != RESET)
    {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }
}
