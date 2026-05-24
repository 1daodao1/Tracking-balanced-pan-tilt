#include "./BSP/USART4/usart4.h"

//UART_HandleTypeDef g_uart4_handle;
UART_HandleTypeDef huart4; // 统一使用 huart4 作为主要句柄

uint8_t uart4_rx_byte = 0;
uint8_t uart4_rx_buf[UART4_RX_BUF_SIZE] = {0};
uint16_t uart4_rx_index = 0;
uint8_t uart4_rx_finish_flag = 0;

/**
 * @brief       UART4初始化函数
 * @note        PA12 -> UART4_TX
 *              PA11 -> UART4_RX
 * @param       baudrate: 波特率
 */
void uart4_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init_struct;

    /* 1. 使能 UART4 和 GPIOA 时钟 */
    __HAL_RCC_UART4_CLK_ENABLE();
    UART4_TX_GPIO_CLK_ENABLE();
    UART4_RX_GPIO_CLK_ENABLE();

    /* 2. 配置 PA12 为 UART4_TX */
    gpio_init_struct.Pin = UART4_TX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = UART4_TX_GPIO_AF;
    HAL_GPIO_Init(UART4_TX_GPIO_PORT, &gpio_init_struct);

    /* 3. 配置 PA11 为 UART4_RX */
    gpio_init_struct.Pin = UART4_RX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = UART4_RX_GPIO_AF;
    HAL_GPIO_Init(UART4_RX_GPIO_PORT, &gpio_init_struct);

    /* 4. 配置 UART4 参数 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = baudrate;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		
    HAL_UART_Init(&huart4);
}

/**
 * @brief UART4 发送一个字节
 */
void UART4_SendByte(uint8_t data)
{
    HAL_UART_Transmit(&huart4, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief UART4 发送数组
 */
void UART4_SendArray(uint8_t *array, uint16_t len)
{
    HAL_UART_Transmit(&huart4, array, len, HAL_MAX_DELAY);
}

/**
 * @brief UART4 发送字符串
 */
void UART4_SendString(char *str)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief 开启 UART4 中断接收
 */
void UART4_StartReceive_IT(void)
{
    HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
}

/**
 * @brief 清空 UART4 接收缓存
 */
void UART4_ClearRxBuffer(void)
{
    memset(uart4_rx_buf, 0, UART4_RX_BUF_SIZE);
    uart4_rx_index = 0;
    uart4_rx_finish_flag = 0;
}
