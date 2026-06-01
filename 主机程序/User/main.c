#include "MyTask.h"


int main(void)
{
	//定义按键变量和计时变量
    uint8_t key = 0;
    uint16_t t = 0;
    
		//开启超高速小容量静态内存，用于提升性能
    sys_cache_enable();                         /* 打开L1-Cache */
    
		//基本时钟初始化
		HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(240, 2, 2, 4);         /* 配置系统时钟, 480Mhz */
    delay_init(480);                            /* 初始化延时功能 */
    
		//串口部分初始化
		usart_init(115200);                         /* 初始化串口 */
//    usmart_dev.init(240);                       /* 初始化USMART */
		//初始化 USART 串口调试组件，并告知该组件当前系统（或相关定时器）的工作频率为 240MHz
//    usart2_init(921600);                       /* 初始化串口2波特率为1500000 */
    mpu_memory_protection();                    /* 保护相关存储区域 */
		//通过牺牲一部分内存访问速度（关闭特定区域的 Cache），来换取 CPU 与硬件外设（DMA）之间的数据一致性
		
		//初始化uart4
		uart4_init(9600); 
		
    led_init();                                 /* 初始化LED */
    lcd_init();                                 /* 初始化LCD */
    key_init();                                 /* 初始化按键 */
    MPU6050_Init();
		btim_timx_int_init(10000 - 1, 24000 - 1);   /*初始化基本定时器， 10Khz计数, 1秒钟中断一次, 用于统计帧率 */
    
		//加入LCD显示，格式为。x,y,显示宽度,高度,字体大小,内容，颜色
    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "OV5640 TEST", RED);
    lcd_show_string(30, 90, 200, 16, 16, "ATOM@ALIENTEK", RED);
    
//		//检查6050的ID号
//		if (MPU6050_GetID() != 0x70)
//		{
//			lcd_show_string(30, 170, 240, 16, 16, "MPU6050 ERROR", RED);
//		}	
//		else
//		{
//			lcd_show_string(30, 170, 240, 16, 16, "MPU6050 OK", GREEN);
//		}

		
		//给子机发送
		//UART4_SendString("UART4 test start\r\n");
    UART4_StartReceive_IT();
		
		//反复初始化OV5600，防止摄像头模块可能接触不良、上电未稳定、SCCB/I2C 通信失败
    while (ov5640_init() != 0)                                                      /* 初始化OV5640 */
    {
        lcd_show_string(30, 130, 240, 16, 16, "OV5640 ERROR", RED);
        delay_ms(200);
        lcd_fill(30, 130, 239, 170, WHITE);
        delay_ms(200);
        LED0_TOGGLE();
    }    
    lcd_show_string(30, 130, 200, 16, 16, "OV5640 OK", RED);
    
    while (1)
    {
				//检测按键
        key = key_scan(0);
        
        if (key == KEY0_PRES)
        {
            g_ov_mode = 0; //规定OV的模式                                                         /* RGB565模式 */
						/* bit0: 0,RGB565模式;  1,平衡模式 */  
						break;
        }
        else if (key == WKUP_PRES)
        {
            g_ov_mode = 1;                                                          /* JPEG模式 */
            break; 
        }
        
        t++;
        
				//这里表明闪烁提示
        if (t == 100)
        {
            lcd_show_string(30, 150, 230, 16, 16, "KEY0:RGB565  WK_UP:JPEG", RED);  /* 闪烁显示提示信息 */
        }
        
        if (t == 200)
        {
            lcd_fill(30, 150, 210, 150 + 16, WHITE);
            t = 0;
            LED0_TOGGLE();
        }
        
        delay_ms(5);
    }
    
		//根据按键，进入不同模式
    if (g_ov_mode == 1)
    {
        balance_test();                                                                /* JPEG模式测试 */
    }
    else 
    {
        rgb565_test();                                                              /* RGB565模式测试 */
    }
		

}
