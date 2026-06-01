#include "MyTask.h"

//2. 摄像头模式、JPEG缓存、状态标志等
uint8_t g_ov_mode = 0;                  /* bit0: 0,RGB565模式;  1,JPEG模式 */
uint16_t g_curline = 0;                 /* 摄像头输出数据,当前行编号 */
uint16_t g_yoffset = 0;                 /* y方向的偏移量 */

static uint32_t last_mpu_tick = 0;
//static uint8_t mpu_overlay_enable = 0;



/* ================= 简单 MPU6050 Pitch 角度 ================= */

#define PI_F                3.1415926f
#define RAD_TO_DEG          (180.0f / PI_F)

/*
 * 角度死区。
 * PitchAngle 在 ±1° 内认为已经接近稳定，不控制电机。
 */
#define PITCH_DEADBAND_DEG      1.0f

/*
 * MPU6050 当前陀螺仪量程是 ±2000°/s
 * 所以：原始值 / 32768 * 2000 = °/s
 */
#define GYRO_TO_DPS         (2000.0f / 32768.0f)

/*
 * 互补滤波参数
 * 0.01 表示：
 * 1%  使用加速度计角度
 * 99% 使用陀螺仪积分角度
 */
#define SIMPLE_ALPHA        0.05f

/*
 * 你的 balance_test() 主循环最后是 delay_ms(10)
 * 所以这里先写 0.010f
 */
#define SIMPLE_DT           0.010f

/* ================= 简单 Pitch 电机控制参数 ================= */

/*
 * 角度死区。
 * PitchAngle 在 ±1° 内认为已经接近稳定，不控制电机。
 */
#define PITCH_DEADBAND_DEG      1.0f

/*
 * 比例系数。
 * 角度越大，发送给电机的控制量越大。
 * 先从小值开始，防止电机动作太猛。
 */
#define PITCH_KP                1.0f

/*
 * 输出限幅。
 * 先限制小一点，安全测试。
 */
#define PITCH_CMD_MAX           15

/*
 * 电机方向。
 * 如果发现电机越调越偏，把 1 改成 -1。
 */
#define PITCH_DIR               1

/*
 * 控制发送周期，单位 ms。
 * 20ms = 50Hz。
 */
#define PITCH_SEND_PERIOD_MS    20

//发送计时变量
static uint32_t PitchSend_LastTick = 0;



static float PitchAngle = 0.0f;      /* 最终 Pitch 角 */
static float GyroY_Offset = 0.0f;    /* Y 轴陀螺仪零偏 */
static float Pitch_Mid = 0.0f;       /* 初始机械中值 */
static uint32_t SimplePitch_LastTick = 0;


const char *EFFECTS_TBL[6] = {"Normal", "Cool", "Warm", "Yellowish ", "Inverse", "Greenish"};                                       /* 6种特效 */


#define MPU_OVERLAY_X      10
#define MPU_OVERLAY_Y      10
#define MPU_OVERLAY_W      210
#define MPU_OVERLAY_H      80

/**
 * @brief  简单限幅
 */
static float Simple_Limit(float value, float min, float max)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }

    return value;
}


/**
 * @brief  简单 Pitch 角初始化
 * @note   调用时 MPU6050 必须保持静止
 */
static void SimplePitch_Init(void)
{
    int16_t AX, AY, AZ;
    int16_t GX, GY, GZ;

    int i;
    float sum_gyro_y = 0.0f;
    float sum_angle_acc = 0.0f;

    for (i = 0; i < 200; i++)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

        /*
         * 记录 Y 轴陀螺仪零偏
         */
        sum_gyro_y += GY;

        /*
         * 使用加速度计计算 Pitch 角
         * 这里采用简单写法：
         * Pitch = -atan2(AX, AZ)
         */
        sum_angle_acc += -atan2f((float)AX, (float)AZ) * RAD_TO_DEG;

        delay_ms(5);
    }

    GyroY_Offset = sum_gyro_y / 200.0f;

    /*
     * 初始 Pitch 角
     */
    Pitch_Mid = sum_angle_acc / 200.0f;   // 关键

    /*
		 * PitchAngle 表示相对初始姿态的角度。
		 * 所以初始化完成后，应该直接等于 0。
		 */
		PitchAngle = 0.0f;

		/*
		 * 记录初始化完成时刻，后面计算真实 dt。
		 */
		SimplePitch_LastTick = HAL_GetTick();
		PitchSend_LastTick = HAL_GetTick();
}

/**
 * @brief  简单 Pitch 角更新
 */
static void SimplePitch_Update(void)
{
    int16_t AX, AY, AZ;
    int16_t GX, GY, GZ;

    uint32_t now_tick;
    float dt;

    float AngleAcc;
    float AngleGyro;
    float GyroY_Dps;

    /*
     * 计算真实 dt，单位：秒
     */
    now_tick = HAL_GetTick();
    dt = (float)(now_tick - SimplePitch_LastTick) / 1000.0f;
    SimplePitch_LastTick = now_tick;

    /*
     * 防止第一次进入、调试暂停、LCD阻塞导致 dt 异常
     */
    if ((dt <= 0.0f) || (dt > 0.1f))
    {
        dt = 0.01f;
    }

    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

    /*
     * 1. 加速度计计算 Pitch 角
     */
    AngleAcc = -atan2f((float)AX, (float)AZ) * RAD_TO_DEG;

    /*
     * 2. 减去初始机械中值
     * 得到相对于初始姿态的角度
     */
    AngleAcc -= Pitch_Mid;

    /*
     * 3. 陀螺仪 Y 轴转角速度
     */
    GyroY_Dps = ((float)GY - GyroY_Offset) * GYRO_TO_DPS;

    /*
     * 4. 陀螺仪死区
     * 如果静止时仍然慢慢增加，把 0.5f 改成 1.0f 或 2.0f
     */
    if (GyroY_Dps > -1.0f && GyroY_Dps < 1.0f)
    {
        GyroY_Dps = 0.0f;
    }

    /*
     * 5. 陀螺仪积分预测
     */
    AngleGyro = PitchAngle + GyroY_Dps * dt;

    /*
     * 6. 互补滤波
     */
    PitchAngle = SIMPLE_ALPHA * AngleAcc +
                 (1.0f - SIMPLE_ALPHA) * AngleGyro;
}



static void lcd_show_mpu6050_overlay(void)
{

    char buf[32];


    /*
     * 先画一个白色背景框，避免旧数字残留。
     * 注意：lcd_show_char 在非叠加模式下会使用 g_back_color 作为字符背景。
     */
    g_back_color = WHITE;


		lcd_show_string(MPU_OVERLAY_X + 6,
										MPU_OVERLAY_Y + 6,
										200,
										16,
										16,
										"Simple Pitch",
										YELLOW);

		    int32_t p100;

    p100 = (int32_t)(PitchAngle * 100.0f);

    if (p100 < 0)
			{
        p100 = -p100;
        sprintf(buf, "P:-%ld.%02ld", p100 / 100, p100 % 100);
			}	
    else
			{
        sprintf(buf, "P: %ld.%02ld", p100 / 100, p100 % 100);
			}

    lcd_show_string(MPU_OVERLAY_X + 6,
                    MPU_OVERLAY_Y + 24,
                    200,
                    16,
                    16,
                    buf,
                    GREEN);

//    sprintf(buf, "R:%7.2f", g_mpu_attitude.roll_err_deg);
		lcd_show_string(MPU_OVERLAY_X + 6,
										MPU_OVERLAY_Y + 24,
										200,
										16,
										16,
										buf,
										GREEN);
	
//		sprintf(buf, "P:%7.2f", g_mpu_attitude.pitch_err_deg);
		lcd_show_string(MPU_OVERLAY_X + 6,
										MPU_OVERLAY_Y + 42,
										200,
										16,
										16,
										buf,
										GREEN);

//		sprintf(buf, "Y:%7.2f", g_mpu_attitude.yaw_err_deg);
		lcd_show_string(MPU_OVERLAY_X + 6,
										MPU_OVERLAY_Y + 60,
										200,
										16,
										16,
										buf,
										GREEN);

}


// 3. DCMI帧中断里处理一帧JPEG数据
/**
 * @brief       处理JPEG数据
 * @ntoe        在DCMI_IRQHandler中断服务函数里面被调用
 *              当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
 * @param       无
 * @retval      无
 */
void jpeg_data_process(void)
{
//    uint16_t i;
//    uint16_t rlen;                                                              /* 剩余数据长度 */
//    uint32_t *pbuf;
//    g_curline = g_yoffset;                                                      /* 行数复位 */
//    
//	//在JPEG模式下
//    if (g_ov_mode & 0x01)                                                       /* 只有在JPEG格式下,才需要做处理. */
//    {
//			//首先判断是否正在采集
//			/*
//			 * 0,数据没有采集完;
//			 * 1,数据采集完了,但是还没处理;
//			 * 2,数据已经处理完成了,可以开始下一帧接收
//			*/
//        if (g_jpeg_data_ok == 0)                                                /* jpeg数据还未采集完? */
//        {
//            __HAL_DMA_DISABLE(&g_dma_dcmi_handle);                              /* 关闭DMA，避免继续接收下一帧，覆盖数据 */
//						
//					//rlen表示已经接收到的数据量
//						rlen = jpeg_line_size - __HAL_DMA_GET_COUNTER(&g_dma_dcmi_handle);  /* 得到剩余数据长度 */
//            //__HAL_DMA_GET_COUNTER() 返回 DMA 还剩多少没传。
//					  
//						//把最后剩余数据也复制到总缓存
//						pbuf = g_jpeg_data_buf + g_jpeg_data_len;                           /* 偏移到有效数据末尾,继续添加 */
//            
//					//判断当前DMA在哪个区
//            if (DMA1_Stream1->CR & (1 << 19))
//            {
//							//如果说当前DMA切换到0空间，说明1空间满了，于是复制1空间
//                for (i = 0; i < rlen; i++)
//                {
//                    pbuf[i] = g_dcmi_line_buf[1][i];                            /* 读取buf1里面的剩余数据 */
//                }
//            }
//            else 
//            {
//                for (i = 0; i < rlen; i++)
//                {
//                    pbuf[i] = g_dcmi_line_buf[0][i];                            /* 读取buf0里面的剩余数据 */
//                }
//            }
//            
//						//标记 JPEG 采集完成
//            g_jpeg_data_len += rlen;                                            /* 加上剩余长度 */
//            g_jpeg_data_ok = 1;                                                 /* 标记JPEG数据采集完成,等待主函数处理 */
//        }
//        //判断是否开始下一帧的接收，重新开启DMA
//        if (g_jpeg_data_ok == 2)                                                /* 上一次的jpeg数据已经被处理了 */
//        {
//            __HAL_DMA_SET_COUNTER(&g_dma_dcmi_handle, jpeg_line_size);          /* 传输长度为jpeg_buf_size*4字节 */
//            __HAL_DMA_ENABLE(&g_dma_dcmi_handle);                               /* 重新传输 */
//            g_jpeg_data_ok = 0;                                                 /* 标记数据未采集 */
//            g_jpeg_data_len = 0;                                                /* 数据重新开始 */
//        }
//    }
//		
//		//在RGB565模式下，一帧结束后，把LCD写入光标重新设置
//    else
//    {
        lcd_set_cursor(0, 0);
        lcd_write_ram_prepare();                                                /* 开始写入GRAM */
//    }
}

//4. DMA接收半包/整包数据时的回调
/**
 * @brief       JPEG数据接收回调函数
								DMA 每接收满一个缓冲区，就把这个缓冲区的数据复制到总 JPEG 缓冲区
 * @ntoe        在DMA1_Stream1_IRQHandler中断服务函数里面被调用
 * @param       无
 * @retval      无
 */
void jpeg_dcmi_rx_callback(void)
{
//    uint16_t i;
//    volatile uint32_t *pbuf;
//    pbuf = g_jpeg_data_buf + g_jpeg_data_len;   /* 偏移到有效数据末尾 */
//    //即找到总缓存的追加位置
//	
//	//这里CR寄存器表示当前使用是哪个区
//    if (DMA1_Stream1->CR & (1 << 19))           /* buf0已满,正常处理buf1 */
//    {
//        for (i = 0; i < jpeg_line_size; i++)
//        {
//            pbuf[i] = g_dcmi_line_buf[0][i];    /* 读取buf0里面的数据 */
//        }
//        
//        g_jpeg_data_len += jpeg_line_size;      /* 偏移 */
//    }
//    else                                        /* buf1已满,正常处理buf0 */
//    {
//        for (i = 0; i < jpeg_line_size; i++)
//        {
//            pbuf[i] = g_dcmi_line_buf[1][i];    /* 读取buf1里面的数据 */
//        }
//        
//        g_jpeg_data_len += jpeg_line_size;      /* 偏移 *///更新数据长度
//    }
//    
//		//避免CPU读取旧数据
//    SCB_CleanInvalidateDCache();                /* 清除无效化DCache */
}


/**
 * @brief  根据 PitchAngle 计算电机控制量，并通过 UART4 发送
 */
static void SimplePitch_ControlSend(void)
{
    uint32_t now_tick;
    float error;
    float output;
    int16_t pitch_cmd;
    char txbuf[32];

    /*
     * 控制发送限频。
     * 不要每次 while 都发，避免串口发送太频繁。
     */
    now_tick = HAL_GetTick();

    if (now_tick - PitchSend_LastTick < PITCH_SEND_PERIOD_MS)
    {
        return;
    }

    PitchSend_LastTick = now_tick;

    /*
     * 目标是让 PitchAngle 回到 0。
     *
     * 当前角度是 PitchAngle，
     * 目标角度是 0，
     * 所以误差 = 0 - PitchAngle。
     */
    error = 0.0f - PitchAngle;

    /*
     * 死区处理。
     * 小角度不控制，避免电机一直抖。
     */
    if (error > -PITCH_DEADBAND_DEG && error < PITCH_DEADBAND_DEG)
    {
        pitch_cmd = 0;
    }
    else
    {
        /*
         * 比例控制：
         * 输出 = 误差 × Kp
         */
        output = error * PITCH_KP;

        /*
         * 根据实际电机方向修正正负号。
         */
        output = output * PITCH_DIR;

        /*
         * 限幅，防止输出过大。
         */
        output = Simple_Limit(output,
                              -(float)PITCH_CMD_MAX,
                              (float)PITCH_CMD_MAX);

        pitch_cmd = (int16_t)output;
    }

    /*
     * 发送给子机。
     *
     * 当前格式：
     * G,pitch,roll
     *
     * 现在只控制 Pitch，Roll 先发 0。
     */
		sprintf(txbuf, "@0,%d\r\n", pitch_cmd);
		UART4_SendString(txbuf);
}


/** 5. 平衡模式测试：采集6050并使用串口发送
 * @brief       平衡测试
 * @ntoe        6050数据,通过串口4发送给子机，并且再LCD屏幕上显示图像和现在6050三轴偏移情况
 * @param       无
 * @retval      无
 */
void balance_test(void)
{
    uint8_t key;
    uint8_t effect = 0;                                                                         /* 默认是全尺寸缩放 */
    uint8_t msgbuf[15];                                                                         /* 消息缓存区 */
    uint16_t outputheight = 0;
    
    lcd_clear(WHITE);
    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "OV5640 RGB565 Mode", RED);
    lcd_show_string(30, 100, 200, 16, 16, "KEY0:Contrast", RED);                                /* 对比度 */
    lcd_show_string(30, 160, 200, 16, 16, "WK_UP:FullSize/Scale", RED);                         /* 1:1尺寸(显示真实尺寸)/全尺寸缩放 */
    
    ov5640_rgb565_mode();  //输出RGB565格式数据	/* RGB565模式 */
    
		//图像效果相关设置
		ov5640_focus_init();                                                                        /* 自动对焦初始化 */
    ov5640_brightness(0);                                                                       /* 自动模式，状态复位 */
    ov5640_color_saturation(3);                                                                 /* 色彩饱和度0 */
    ov5640_brightness(4);                                                                       /* 亮度0 */
    ov5640_contrast(3);                                                                         /* 对比度0 */
    ov5640_sharpness(33);                                                                       /* 自动锐度 */
    ov5640_focus_constant();                                                                    /* 启动持续对焦 */
    
		//初始化DCMI，使其能接收摄像头数据
		dcmi_init();                                                                                /* DCMI配置 */
    //初始化 DMA，把数据直接送到 LCD
		dcmi_dma_init((uint32_t)&LCD->LCD_RAM, //DMA 目标地址是 LCD 的显存写入口。
										0, //第二个缓存区不用
										1, //一次搬运单位长度
										DMA_MDATAALIGN_HALFWORD, //内存数据宽度是半字
										DMA_MINC_DISABLE);//内存地址不自增，硬件自动自增    /* DCMI DMA配置,MCU屏,竖屏 */
    //DCMI 接收到的数据，通过 DMA，直接写入 LCD 的数据寄存器 LCD_RAM
		
    if (lcddev.height >= 800)
    {
        g_yoffset = (lcddev.height - 800) / 2;
        outputheight = 800;
        ov5640_write_reg(0x3035, 0x51);                                                         /* 降低输出帧率，否则可能抖动 */
    }
    else
    {
        g_yoffset = 0;
        outputheight = lcddev.height;
    }
    g_curline = g_yoffset;                                                                      /* 行数复位 */
    
		//摄像头输出适配 LCD 的宽高。
		ov5640_outsize_set(4, 0, lcddev.width, outputheight);                                       /* 满屏缩放显示 */
		
		
		lcd_show_string(30, 190, 240, 16, 16, "Keep MPU Still...", RED);

		/*
		 * 简单 Pitch 初始化
		 * 这里必须保持 MPU6050 静止
		 */
		SimplePitch_Init();

		lcd_show_string(30, 190, 240, 16, 16, "Pitch Init OK    ", GREEN);

		delay_ms(500);
		
		dcmi_start();                                                                               /* 启动传输 */
		lcd_clear(BLACK);
    
    while (1)
    {
			  SimplePitch_Update();
			
				SimplePitch_ControlSend();	
			
        key = key_scan(0);
        
        if (key)
        {
            if (key != WKUP_PRES)
            {
                dcmi_stop();                                                                    /* 非KEY1按下,停止显示 */
            }
            
            switch (key)
            {
							//每按一次 KEY0，切换一种图像特效
                case WKUP_PRES:                                                                 /* 特效设置 */
                {
                    effect++;
                    
                    if (effect >= 6)
                    {
                        effect = 0;
                    }
                    ov5640_special_effects(effect);                                             /* 设置特效 */
                    sprintf((char *)msgbuf, "%s", EFFECTS_TBL[effect]);
                    break;
                }
								
            }
            
            if (key != WKUP_PRES)                                                               /* 非KEY0按下 */
            {
                lcd_show_string(30, 50, 210, 16, 16, (char*)msgbuf, RED);                       /* 显示提示内容 */
                delay_ms(800);
                dcmi_start();                                                                   /* 重新开始传输 */
            }
        }
				
				/*
     * 每 100ms 更新一次 MPU6050 数据叠加
     * 注意：画LCD前暂停DCMI，画完后恢复DCMI
     */
    if (HAL_GetTick() - last_mpu_tick >= 100)
    {
        last_mpu_tick = HAL_GetTick();

        dcmi_stop();

        lcd_show_mpu6050_overlay();

        /*
         * 关键：因为刚才 lcd_fill/lcd_show_string 改变了 LCD GRAM 写入位置，
         * 所以恢复摄像头DMA前，必须重新设置LCD写入起点。
         */
        lcd_set_cursor(0, g_yoffset);
        lcd_write_ram_prepare();

        dcmi_start();
    }
		
        delay_ms(10);
    }
}

/**6. RGB565模式测试：摄像头实时显示到LCD
 * @brief       RGB565测试
 * @ntoe        RGB数据直接显示在LCD上面
 * @param       无
 * @retval      无
 */
void rgb565_test(void)
{
    uint8_t key;
    float fac = 0;
    uint8_t effect = 0;
    uint8_t scale = 1;                                                                          /* 默认是全尺寸缩放 */
    uint8_t msgbuf[15];                                                                         /* 消息缓存区 */
    uint16_t outputheight = 0;
    
    lcd_clear(WHITE);
    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "OV5640 RGB565 Mode", RED);
    lcd_show_string(30, 100, 200, 16, 16, "KEY0:Contrast", RED);                                /* 对比度 */
    lcd_show_string(30, 160, 200, 16, 16, "WK_UP:FullSize/Scale", RED);                         /* 1:1尺寸(显示真实尺寸)/全尺寸缩放 */
    
    ov5640_rgb565_mode();  //输出RGB565格式数据	/* RGB565模式 */
    
		//图像效果相关设置
		ov5640_focus_init();                                                                        /* 自动对焦初始化 */
    ov5640_brightness(0);                                                                       /* 自动模式，状态复位 */
    ov5640_color_saturation(3);                                                                 /* 色彩饱和度0 */
    ov5640_brightness(4);                                                                       /* 亮度0 */
    ov5640_contrast(3);                                                                         /* 对比度0 */
    ov5640_sharpness(33);                                                                       /* 自动锐度 */
    ov5640_focus_constant();                                                                    /* 启动持续对焦 */
    
		//初始化DCMI，使其能接收摄像头数据
		dcmi_init();                                                                                /* DCMI配置 */
    //初始化 DMA，把数据直接送到 LCD
		dcmi_dma_init((uint32_t)&LCD->LCD_RAM, //DMA 目标地址是 LCD 的显存写入口。
										0, //第二个缓存区不用
										1, //一次搬运单位长度
										DMA_MDATAALIGN_HALFWORD, //内存数据宽度是半字
										DMA_MINC_DISABLE);//内存地址不自增，硬件自动自增    /* DCMI DMA配置,MCU屏,竖屏 */
    //DCMI 接收到的数据，通过 DMA，直接写入 LCD 的数据寄存器 LCD_RAM
		
    if (lcddev.height >= 800)
    {
        g_yoffset = (lcddev.height - 800) / 2;
        outputheight = 800;
        ov5640_write_reg(0x3035, 0x51);                                                         /* 降低输出帧率，否则可能抖动 */
    }
    else
    {
        g_yoffset = 0;
        outputheight = lcddev.height;
    }
    g_curline = g_yoffset;                                                                      /* 行数复位 */
    
		//摄像头输出适配 LCD 的宽高。
		ov5640_outsize_set(4, 0, lcddev.width, outputheight);                                       /* 满屏缩放显示 */
    //启动DCMI
		dcmi_start();                                                                               /* 启动传输 */
    lcd_clear(BLACK);
    TrackControl_Reset();
		
    while (1)
    {
        key = key_scan(0);
        
        if (key)
        {
            if (key != KEY0_PRES)
            {
                dcmi_stop();                                                                    /* 非KEY1按下,停止显示 */
            }
            
            switch (key)
            {
							//每按一次 KEY0，切换一种图像特效
                case KEY0_PRES:                                                                 /* 特效设置 */
                {
                    effect++;
                    
                    if (effect >= 6)
                    {
                        effect = 0;
                    }
                    ov5640_special_effects(effect);                                             /* 设置特效 */
                    sprintf((char *)msgbuf, "%s", EFFECTS_TBL[effect]);
                    break;
                }
								
								//RGB565 的全尺寸 / 缩放显示
                //scale = 1：把摄像头图像缩放到 LCD 大小
								//scale = 0：尽量显示摄像头原始比例的中间区域
                case WKUP_PRES:                                                                 /* 1:1尺寸(显示真实尺寸)/缩放 */
                {
                    scale = !scale;
                    
                    if (scale == 0)
                    {
                        fac = (float)800 / outputheight;                                        /* 得到比例因子 */
                        ov5640_outsize_set((1280 - fac * lcddev.width) / 2, (800 - fac * outputheight) / 2, lcddev.width, outputheight);
                        sprintf((char *)msgbuf, "Full Size 1:1");
                    }
                    else
                    {
                        ov5640_outsize_set(4, 0, lcddev.width, outputheight);
                        sprintf((char *)msgbuf, "Scale");
                    }
                    break;
                }
            }
            
            if (key != KEY0_PRES)                                                               /* 非KEY0按下 */
            {
                lcd_show_string(30, 50, 210, 16, 16, (char*)msgbuf, RED);                       /* 显示提示内容 */
                delay_ms(800);
                dcmi_start();                                                                   /* 重新开始传输 */
            }
        }
				
        // 为防止 CPU 读写 LCD 寄存器时与 DMA 发生冲突导致花屏，
        // 在进行图像处理和叠加显示前，先暂停 DCMI 传输。
        dcmi_stop();

        // 传入 NULL，让函数内部使用 lcd_read_point 从 LCD 显存中读取像素进行识别
        BallResult_t ball = FindBall_RGB565(NULL, lcddev.width, outputheight);
        
        // 在 LCD 上叠加十字和坐标信息
        LCD_DrawBallOverlay(&ball, lcddev.width, outputheight);
        
        // 恢复 DCMI 传输，继续采集下一帧
        dcmi_start();

        // 串口发送数据给子机
        TrackControl_UpdateAndSend(&ball);			
								
        delay_ms(10);
    }
}
