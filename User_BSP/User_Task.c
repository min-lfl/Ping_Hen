#include "User_Task.H"
#include "Control_Config.h"
#include <math.h>

#define CONTROL_RAD_TO_DEG (57.2957795f)

typedef struct {
	float rod_angle_deg;
	int32_t motor_pulse;
} LinkageCalPoint_t;

typedef struct {
	float stage_1;
	float stage_2;
	uint8_t initialized;
} ControlLowPass_t;

typedef struct {
	float position;
	float velocity;
	float acceleration;
} ControlMotion_t;

/*
 * LINKAGE CALIBRATION TABLE
 * Keep rod_angle_deg strictly increasing. Replace each angle with the
 * measured rod angle at the corresponding motor pulse.
 */
static const LinkageCalPoint_t linkage_cal_table[] = {
	{-3.975f, -250},
	{-2.683f, -167},
	{-1.832f,  -83},
	{ 0.000f,    0},
	{ 2.826f,   83},
	{ 4.383f,  167},
	{ 5.975f,  250},
};

volatile float control_debug_ay_filtered_g = 0.0f;
volatile float control_debug_az_filtered_g = 1.0f;
volatile float control_debug_target_angle_deg = 0.0f;
volatile float control_debug_target_pulse = 0.0f;
volatile float control_debug_command_pulse = 0.0f;
volatile float control_debug_profile_speed = 0.0f;
volatile uint8_t control_automatic_enabled = CONTROL_AUTOMATIC_ENABLE;

static ControlLowPass_t control_ay_filter = {0};
static ControlLowPass_t control_az_filter = {0};
static ControlMotion_t control_motion = {0};
static volatile float control_target_pulse = (float)CONTROL_LEVEL_PULSE;
static volatile uint8_t control_target_valid = 0;
static int32_t control_last_sent_pulse = CONTROL_LEVEL_PULSE;
static uint8_t control_command_sent = 0;

static void Control_Init(void);
static void Control_Input_Update(float ay_g, float az_g);
static float Control_Motion_Update(float target_pulse);

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

	BSP_Emm_V5_Pos_Init();
	HAL_Delay(5);
	Control_Init();
}

//##################################################################################################
//********关于参数更新模块任务***************************************************************************
//##################################################################################################
/**
	* @brief	参数更新任务函数,在main函数的while循环中调用,用于更新系统参数
	* @note		无
	* @param	无
	* @retval	无
	*/
volatile uint16_t UPdate_Speed_RPM = CONTROL_MOTOR_SPEED_RPM;
volatile uint8_t UPdate_Accel_Param = CONTROL_MOTOR_ACCEL_PARAM;
void User_Task_Param_Update(void){	
	Emm_V5_Set_QPos_Params(1,
	                   UPdate_Speed_RPM,
	                   UPdate_Accel_Param,
	                   0x01,
	                   EMM_V5_SNF);
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
	if(mpu_ready) {
		MPU6050_Read_All(&hi2c2, &mpu6050_date,300); // 读取 MPU6050 数据并进行姿态解算，采样频率为 300Hz
		Control_Input_Update((float)mpu6050_date.Ay, (float)mpu6050_date.Az);
	}
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
	// static int my_count = 0;       // 自加的数字
	// static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	// my_count++;					   // 自加
	// sprintf(str_buff, "Count: %d", my_count);			//组合打印内容,这里是把数字变成字符
	// ssd1306_Fill(Black);									//清空屏幕缓冲区
	// ssd1306_SetCursor(10, 20);							//锁定打印位置
	// ssd1306_WriteString(str_buff, Font_11x18, White);	//确定打印字符,大小,颜色
	// ssd1306_UpdateScreen_DMA();							//刷新屏幕显示,使用DMA搬运数据,不会阻塞主循环
	
	//现在想要显示参数,分别是CONTROL_MOTOR_SPEED_RPM和CONTROL_MOTOR_ACCEL_PARAM两个宏定义的值
	static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	ssd1306_Fill(Black);			//清空屏幕缓冲区

	ssd1306_SetCursor(10, 20);									//锁定打印位置
	sprintf(str_buff, "Count: %d", UPdate_Speed_RPM);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色


	ssd1306_SetCursor(10, 40);									//锁定打印位置
	sprintf(str_buff, "Count: %d", UPdate_Accel_Param);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色
	ssd1306_UpdateScreen_DMA();	

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
					/* 翻转 PB12 的电平 */
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
					BSP_Emm_V5_Pos_Control(CONTROL_LEVEL_PULSE + CONTROL_MANUAL_TEST_PULSE);
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
					/* 翻转 PA8 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
					BSP_Emm_V5_Pos_Control(CONTROL_LEVEL_PULSE);
				
					//自动控制切换开关
					if(control_automatic_enabled==1){
						control_automatic_enabled=0;		//暂时禁用自动控制
					}else{
						control_automatic_enabled=1;
					}
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
					/* 翻转 PA11 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
					BSP_Emm_V5_Pos_Control(CONTROL_LEVEL_PULSE - CONTROL_MANUAL_TEST_PULSE);
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}
}



//##################################################################################################
//********关于控制模块任务***************************************************************************
//##################################################################################################
static float Control_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}

static float Control_Approach(float current, float target, float maximum_step)
{
	if (target > current + maximum_step)
		return current + maximum_step;
	if (target < current - maximum_step)
		return current - maximum_step;
	return target;
}

static float Control_LowPass_Update(ControlLowPass_t *filter, float input)
{
	const float dt = 1.0f / CONTROL_IMU_SAMPLE_HZ;
	const float rc = 1.0f / (6.2831853f * CONTROL_ACCEL_LPF_POLE_HZ);
	const float alpha = dt / (rc + dt);

	if (!filter->initialized) {
		filter->stage_1 = input;
		filter->stage_2 = input;
		filter->initialized = 1;
		return input;
	}

	filter->stage_1 += alpha * (input - filter->stage_1);
	filter->stage_2 += alpha * (filter->stage_1 - filter->stage_2);
	return filter->stage_2;
}

static float Control_Linkage_Angle_To_Pulse(float rod_angle_deg)
{
	const uint32_t point_count = sizeof(linkage_cal_table) / sizeof(linkage_cal_table[0]);
	uint32_t index;

	if (rod_angle_deg <= linkage_cal_table[0].rod_angle_deg)
		return (float)linkage_cal_table[0].motor_pulse;

	for (index = 1; index < point_count; index++) {
		if (rod_angle_deg <= linkage_cal_table[index].rod_angle_deg) {
			const LinkageCalPoint_t *left = &linkage_cal_table[index - 1];
			const LinkageCalPoint_t *right = &linkage_cal_table[index];
			const float ratio = (rod_angle_deg - left->rod_angle_deg) /
				(right->rod_angle_deg - left->rod_angle_deg);

			return (float)left->motor_pulse +
				ratio * (float)(right->motor_pulse - left->motor_pulse);
		}
	}

	return (float)linkage_cal_table[point_count - 1].motor_pulse;
}

static void Control_Init(void)
{
	control_ay_filter.initialized = 0;
	control_az_filter.initialized = 0;
	control_motion.position = (float)CONTROL_LEVEL_PULSE;
	control_motion.velocity = 0.0f;
	control_motion.acceleration = 0.0f;
	control_target_pulse = (float)CONTROL_LEVEL_PULSE;
	control_target_valid = 0;
	control_last_sent_pulse = CONTROL_LEVEL_PULSE;
	control_command_sent = 0;
}

static void Control_Input_Update(float ay_g, float az_g)
{
	float ay_corrected;
	float az_corrected;
	float ay_filtered;
	float az_filtered;
	float vertical_g;
	float target_angle_rad;
	float target_angle_deg;
	float relative_pulse;
	float absolute_pulse;
	const float minimum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MIN_RELATIVE_PULSE);
	const float maximum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MAX_RELATIVE_PULSE);

	ay_corrected = ay_g - CONTROL_ACCEL_Y_BIAS_G;
	ay_corrected = Control_Clamp(ay_corrected,
		-CONTROL_ACCEL_INPUT_LIMIT_G,
		 CONTROL_ACCEL_INPUT_LIMIT_G);
	az_corrected = az_g - CONTROL_ACCEL_Z_BIAS_G;

	ay_filtered = Control_LowPass_Update(&control_ay_filter, ay_corrected);
	az_filtered = Control_LowPass_Update(&control_az_filter, az_corrected);
	vertical_g = fabsf(az_filtered);
	if (vertical_g < 0.25f)
		vertical_g = 1.0f;

	target_angle_rad = CONTROL_ACCEL_TO_ROD_SIGN * atan2f(ay_filtered, vertical_g);
	target_angle_deg = target_angle_rad * CONTROL_RAD_TO_DEG;
	relative_pulse = Control_Linkage_Angle_To_Pulse(target_angle_deg);
	absolute_pulse = (float)CONTROL_LEVEL_PULSE + relative_pulse;
	absolute_pulse = Control_Clamp(absolute_pulse, minimum_pulse, maximum_pulse);

	control_debug_ay_filtered_g = ay_filtered;
	control_debug_az_filtered_g = az_filtered;
	control_debug_target_angle_deg = target_angle_deg;
	control_debug_target_pulse = absolute_pulse;
	control_target_pulse = absolute_pulse;
	control_target_valid = 1;
}

static float Control_Motion_Update(float target_pulse)
{
	const float dt = 1.0f / CONTROL_LOOP_HZ;
	const float natural_omega = 6.2831853f * CONTROL_PROFILE_NATURAL_HZ;
	const float minimum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MIN_RELATIVE_PULSE);
	const float maximum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MAX_RELATIVE_PULSE);
	float error = target_pulse - control_motion.position;
	float desired_acceleration;
	float next_position;

	if ((fabsf(error) <= CONTROL_SETTLE_POSITION_PULSE) &&
		(fabsf(control_motion.velocity) <= CONTROL_SETTLE_SPEED_PULSE_S) &&
		(fabsf(control_motion.acceleration) <= CONTROL_SETTLE_ACCEL_PULSE_S2)) {
		control_motion.position = target_pulse;
		control_motion.velocity = 0.0f;
		control_motion.acceleration = 0.0f;
		return control_motion.position;
	}

	desired_acceleration = natural_omega * natural_omega * error -
		2.0f * CONTROL_PROFILE_DAMPING * natural_omega * control_motion.velocity;
	desired_acceleration = Control_Clamp(desired_acceleration,
		-CONTROL_MAX_PULSE_ACCEL,
		 CONTROL_MAX_PULSE_ACCEL);
	control_motion.acceleration = Control_Approach(control_motion.acceleration,
		desired_acceleration,
		CONTROL_MAX_PULSE_JERK * dt);

	control_motion.velocity += control_motion.acceleration * dt;
	control_motion.velocity = Control_Clamp(control_motion.velocity,
		-CONTROL_MAX_PULSE_SPEED,
		 CONTROL_MAX_PULSE_SPEED);
	next_position = control_motion.position + control_motion.velocity * dt;

	if (next_position <= minimum_pulse) {
		control_motion.position = minimum_pulse;
		control_motion.velocity = 0.0f;
		control_motion.acceleration = 0.0f;
	}
	else if (next_position >= maximum_pulse) {
		control_motion.position = maximum_pulse;
		control_motion.velocity = 0.0f;
		control_motion.acceleration = 0.0f;
	}
	else
		control_motion.position = next_position;

	return control_motion.position;
}

/**
	* @brief	控制任务函数,在main函数的while循环中调用,用于处理控制逻辑,包括PID控制和电机控制
	* @note		该函数会在主循环中被调用,用于处理控制逻辑,目前函数只是用来触发控制逻辑,和执行控制
	* @param	无
	* @retval	无
	*/
void User_Task_Control(void){
	float target_pulse;
	float profiled_pulse;
	int32_t command_pulse;
	int32_t command_delta;
	const int32_t minimum_pulse = CONTROL_LEVEL_PULSE + CONTROL_MIN_RELATIVE_PULSE;
	const int32_t maximum_pulse = CONTROL_LEVEL_PULSE + CONTROL_MAX_RELATIVE_PULSE;

	if (!control_automatic_enabled)
		return;

	target_pulse = control_target_valid ? control_target_pulse : (float)CONTROL_LEVEL_PULSE;
	profiled_pulse = Control_Motion_Update(target_pulse);
	command_pulse = (profiled_pulse >= 0.0f) ?
		(int32_t)(profiled_pulse + 0.5f) : (int32_t)(profiled_pulse - 0.5f);
	if (command_pulse < minimum_pulse)
		command_pulse = minimum_pulse;
	if (command_pulse > maximum_pulse)
		command_pulse = maximum_pulse;

	control_debug_command_pulse = (float)command_pulse;
	control_debug_profile_speed = control_motion.velocity;
	command_delta = command_pulse - control_last_sent_pulse;
	if (command_delta < 0)
		command_delta = -command_delta;

	if ((!control_command_sent) ||
		(command_delta >= CONTROL_SEND_HYSTERESIS_PULSE)) {
		BSP_Emm_V5_Pos_Control(command_pulse);
		control_last_sent_pulse = command_pulse;
		control_command_sent = 1;
	}
}




