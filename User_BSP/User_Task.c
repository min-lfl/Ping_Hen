#include "User_Task.H"

/**
	* @brief	任务初始化函数,在main函数中调用,用于初始化所有任务,包括硬件初始化和软件初始化
	* @note		目前的设计思路,所有模块的初始化方式不同,下面会定义所有模块的独特初始化方式封装成函数(轮询,等待,多少次读取,校准逻辑)
				再由本函数统一调用,这样可以保证所有模块的初始化都在main函数中调用,并且可以保证所有模块的初始化顺序,避免模块之间的初始化冲突
	* @param	无
	* @retval	无
	*/
void User_Task_Init(void){
	
	// 初始化所有任务
	User_Task_MPU6050_Init();	//初始化陀螺仪
	User_Task_OLED_Init();		//初始化OLED

	/**********************************************************
	***	上电延时500毫秒等待闭环初始化完毕
	**********************************************************/	
	HAL_Delay(500);
}

//##################################################################################################
//********关于陀螺仪模块任务**************************************************************************
//##################################################################################################
/**
	* @brief	陀螺仪任务初始化函数
	* @note		该函数用于初始化陀螺仪相关的硬件和软件资源
	* @param	无
	* @retval	无
	*/
static uint8_t mpu_ready = 0;
void User_Task_MPU6050_Init(void){
	//初始化并且上电静止校准陀螺仪
	if (MPU6050_Init(&hi2c2) == 0)
	{
		printf_dma("Keep MPU still: calibrating gyro...\r\n");
		if (MPU6050_CalibrateGyro(&hi2c2, 300) == 0)
			mpu_ready = 1;
		else
			printf_dma("MPU gyro calibration failed\r\n");
	}
	else
	{
		printf_dma("MPU WHO_AM_I or I2C failed\r\n");
	}
}


// 定义数据缓冲区(局部静态变量,避免频繁分配和释放内存)
static MPU6050_t mpu6050_date = {0};
/**
	* @brief	陀螺仪任务函数,在main函数的while循环中调用,用于处理陀螺仪数据采集和姿态解算
	* @note		该函数会在主循环中被调用,用于读取 MPU6050 的数据并进行姿态解算,目前函数只是用来触发采样,和解算
	* @param	无
	* @retval	无
	*/
void User_Task_MPU6050_Update(void){
	// 处理 MPU6050 数据
	if(mpu_ready)
		MPU6050_Read_All(&hi2c2, &mpu6050_date, 5.0); // 读取 MPU6050 数据并进行姿态解算，采样频率为 200Hz
	else
		printf_dma("MPU not ready, please check initialization and calibration\r\n");
}

/**
	* @brief	陀螺仪任务函数,用于获取陀螺仪数据
	* @note		该函数用于获取陀螺仪数据,并将数据传递给外部使用
	* @param	mpu6050_data: 指向 MPU6050_t 结构体的指针
	* @retval	无
	*/
void User_Task_MPU6050_Get(MPU6050_t* data){
	if (data != NULL) {
		//把 mpu6050_date 的数据复制一份给外部使用
		*data = mpu6050_date;
	}
}

//##################################################################################################
//**********************关于OLED模块任务*************************************************************
//##################################################################################################

/**
	* @brief	OLED任务初始化函数
	* @note		该函数用于初始化OLED相关的硬件和软件资源
	* @param	无
	* @retval	无
	*/
void User_Task_OLED_Init(void){
//初始化 OLED 屏幕
ssd1306_Init();
//清空屏幕显存 (全部填充满黑色)
ssd1306_Fill(Black);

}


/**
	* @brief	OLED任务函数,在main函数的while循环中调用,用于更新OLED屏幕显示
	* @note		该函数会在主循环中被调用,用于更新OLED屏幕显示,目前函数只是用来触发更新,和显示
	* @param	无
	* @retval	无
	*/
void User_Task_OLED_Update(void){
	static int my_count = 0;       // 自加的数字
	static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	my_count++;					   // 自加
	sprintf(str_buff, "Count: %d", my_count);			//组合打印内容,这里是把数字变成字符
	ssd1306_Fill(Black);								//清空屏幕缓冲区
	ssd1306_SetCursor(10, 20);							//锁定打印位置
	ssd1306_WriteString(str_buff, Font_11x18, White);	//确定打印字符,大小,颜色
	ssd1306_UpdateScreen_DMA();							//刷新屏幕显示,使用DMA搬运数据,不会阻塞主循环
}


//##################################################################################################
//********关于串口模块任务***************************************************************************
//##################################################################################################
/**
	* @brief	串口任务函数,在main函数的while循环中调用,用于更新串口数据传输
	* @note		该函数会在主循环中被调用,用于更新串口数据传输,目前函数只是用来触发发送,和发送数据
	* @param	无
	* @retval	无
	*/
void User_Task_UART_Update(void){
	MPU6050_t mpu6050_date = {0};			//声明局部结构体
	User_Task_MPU6050_Get(&mpu6050_date);	//获取陀螺仪数据

	printf_dma("x轴加速度%d x轴角速度%d \r\n", mpu6050_date.Accel_X_RAW, mpu6050_date.Gyro_X_RAW);
	printf_dma("y轴加速度%d y轴角速度%d \r\n", mpu6050_date.Accel_Y_RAW, mpu6050_date.Gyro_Y_RAW);
	printf_dma("z轴加速度%d z轴角速度%d \r\n", mpu6050_date.Accel_Z_RAW, mpu6050_date.Gyro_Z_RAW);
	printf_dma(" \r\n");
	printf_dma("转化后x轴加速度%f g 转化后x轴角速度%f 度/s \r\n", mpu6050_date.Ax, mpu6050_date.Gx);
	printf_dma("转化后y轴加速度%f g 转化后y轴角速度%f 度/s \r\n", mpu6050_date.Ay, mpu6050_date.Gy);
	printf_dma("转化后z轴加速度%f g 转化后z轴角速度%f 度/s \r\n", mpu6050_date.Az, mpu6050_date.Gz);
	printf_dma(" \r\n");
	printf_dma("当前x轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleX);
	printf_dma("当前y轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleY);
	printf_dma(" \r\n");
}


//##################################################################################################
//********关于按键模块任务***************************************************************************
//##################################################################################################
/**
	* @brief	按键任务函数,在main函数的while循环中调用,用于处理按键事件
	* @note		该函数会扫描三个按键,分别对应GPIOB的13,14,15引脚,按下时会翻转对应的LED灯以及电机位置模式正反转的功能
	* @param	无
	* @retval	无
	*/
void User_Task_key(void){
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) 
	{
			/* 软件消抖延时（根据实际情况决定是否保留） */
			HAL_Delay(20); 
			
			/* 再次确认是否依然为低电平 */
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) 
			{
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        			BSP_Emm_V5_Pos_Control(2000);   //实际收到的01 FD 00 03 E8 00 00 00 07 D0 02 00 6B 
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) 
	{
			/* 软件消抖延时（根据实际情况决定是否保留） */
			HAL_Delay(20); 
			
			/* 再次确认是否依然为低电平 */
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) 
			{
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
        			BSP_Emm_V5_Pos_Control(0);
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}
	
	
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) 
	{
			/* 软件消抖延时（根据实际情况决定是否保留） */
			HAL_Delay(20); 
			
			/* 再次确认是否依然为低电平 */
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) 
			{
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
					BSP_Emm_V5_Pos_Control(-2000);
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}
}


