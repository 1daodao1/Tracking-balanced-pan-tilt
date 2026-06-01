#ifndef __UART4_H
#define __UART4_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include <string.h>
#include "stm32h7xx_hal.h"

/* PA12 -> UART4_TX */
#define UART4_TX_GPIO_PORT          GPIOA
#define UART4_TX_GPIO_PIN           GPIO_PIN_12
#define UART4_TX_GPIO_AF            GPIO_AF6_UART4
#define UART4_TX_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* PA11 -> UART4_RX */
#define UART4_RX_GPIO_PORT          GPIOA
#define UART4_RX_GPIO_PIN           GPIO_PIN_11
#define UART4_RX_GPIO_AF            GPIO_AF6_UART4
#define UART4_RX_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define UART4_RX_BUF_SIZE 128
//extern UART_HandleTypeDef g_uart4_handle;
extern uint8_t uart4_rx_byte;
extern uint8_t uart4_rx_buf[UART4_RX_BUF_SIZE];
extern uint16_t uart4_rx_index;
extern uint8_t uart4_rx_finish_flag;
extern UART_HandleTypeDef huart4;

void uart4_init(uint32_t baudrate);
void UART4_SendByte(uint8_t data);
void UART4_SendArray(uint8_t *array, uint16_t len);
void UART4_SendString(char *str);
void UART4_StartReceive_IT(void);
void UART4_ClearRxBuffer(void);

#endif
