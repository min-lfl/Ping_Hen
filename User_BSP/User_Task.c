#include "User_Task.H"
#include "Control_Config.h"
#include <math.h>

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
	
	User_Task_Laser_UART_Init();
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
//**********************关于激光串口模块任务**********************************************************
//##################################################################################################
/**
	* @brief	激光串口任务初始化函数
	* @note		该函数用于初始化激光串口相关的硬件和软件资源
	* @param	无
	* @retval	无
	*/
void User_Task_Laser_UART_Init(void){
	Laser_UART_Init(&huart2);
}


/**
	* @brief	获取激光后台缓存的最新位置，正数在原点右侧，负数在原点左侧。
	* @note		本函数只获取缓存，不会主动读取串口；有效状态见 ball_control_laser_valid。
	* @param	distance_mm 输出相对 BALL_CONTROL_LASER_ORIGIN_MM 的位置，单位 mm。
	* @retval	无
	*/
void User_Task_Laser_UART_Get(float *distance_mm){
	if (distance_mm != NULL) {
		if (Laser_UART_GetDistance(distance_mm)) {
			*distance_mm -= BALL_CONTROL_LASER_ORIGIN_MM;
			__DMB();
			ball_control_laser_valid = 1U;
		} else {
			ball_control_laser_valid = 0U;
			*distance_mm = 0.0f;
		}
	}
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
volatile uint16_t UPdate_Manual_PULSE = CONTROL_MANUAL_TEST_PULSE;
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
					BSP_Emm_V5_Pos_Control(CONTROL_LEVEL_PULSE + UPdate_Manual_PULSE);
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
					BSP_Emm_V5_Pos_Control(CONTROL_LEVEL_PULSE - UPdate_Manual_PULSE);
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}
}



//##################################################################################################
//********关于加速度补偿控制模块任务(方案1已弃用)***************************************************************************
//##################################################################################################
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
	{-4.525f, -250},
	{-3.123f, -167},
	{-2.432f,  -83},
	{ 0.000f,    0},
	{ 2.826f,   83},
	{ 4.783f,  167},
	{ 6.475f,  250},
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
	* @brief	加速度补偿控制任务函数,在main函数的while循环中调用,用于处理控制逻辑,包括PID控制和电机控制
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



//##################################################################################################
//********关于速度环+位置环控制模块任务*******************************************************
//##################################################################################################
/*
 * 方案 2 对外可观察状态。
 * 两个 PID 各自只由对应的定时任务修改；32 位 float 在 Cortex-M3 上可原子读写。
 */
volatile uint8_t ball_control_enabled = BALL_CONTROL_ENABLE_DEFAULT;
volatile uint8_t ball_control_laser_valid = 0U;
volatile float ball_control_target_position_mm = BALL_CONTROL_TARGET_POSITION_MM;
volatile float ball_control_position_mm = 0.0f;
volatile float ball_control_speed_mm_s = 0.0f;
volatile float ball_control_target_speed_mm_s = 0.0f;
volatile float ball_control_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE;

/* 位置外环输出目标速度；速度内环输出相对水平位置的电机脉冲。 */
volatile PID_t ball_position_pid = {
	.Kp = BALL_POSITION_KP,
	.Ki = BALL_POSITION_KI,
	.Kd = BALL_POSITION_KD,
	.integral_max = BALL_POSITION_INTEGRAL_LIMIT_MM_SEC,
	.output_max = BALL_POSITION_MAX_TARGET_SPEED_MM_S
};

volatile PID_t ball_speed_pid = {
	.Kp = BALL_SPEED_KP,
	.Ki = BALL_SPEED_KI,
	.Kd = BALL_SPEED_KD,
	.integral_max = BALL_SPEED_INTEGRAL_LIMIT_MM,
	.output_max = BALL_MOTOR_MAX_CONTROL_PULSE
};

/* 激光约 10 Hz，而速度环更快。只有位置确实变化时才进行一次差分。 */
static uint8_t ball_speed_estimator_initialized = 0U;
static float ball_speed_reference_position_mm = 0.0f;
static uint32_t ball_speed_reference_tick_ms = 0U;
static uint32_t ball_speed_last_motion_tick_ms = 0U;

/* 电机命令状态用于限速和抑制重复 DMA 发送。 */
static int32_t ball_motor_last_sent_pulse = BALL_MOTOR_LEVEL_PULSE;
static uint32_t ball_motor_last_send_tick_ms = 0U;
static uint8_t ball_motor_command_sent = 0U;

static float Ball_Control_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}

static float Ball_Control_Approach(float current, float target, float maximum_step)
{
	if (target > current + maximum_step)
		return current + maximum_step;
	if (target < current - maximum_step)
		return current - maximum_step;
	return target;
}

static int32_t Ball_Control_Round_To_Int32(float value)
{
	return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

/** 清除 PID 历史，防止停用或进入死区后残留积分继续驱动电机。 */
static void Ball_Control_Reset_PID(PID_t *pid)
{
	pid->error = 0.0f;
	pid->last_error = 0.0f;
	pid->integral = 0.0f;
	pid->differential = 0.0f;
	pid->output = 0.0f;
}

/**
 * @brief 根据激光位置变化估算小球速度，并对差分结果进行一阶低通。
 * @note  重复取得同一后台缓存值时不会再次差分，避免把 10 Hz 数据误当成 100 Hz 数据。
 */
static float Ball_Control_Update_Speed_Estimate(float position_mm, uint32_t now_ms)
{
	float position_delta_mm;
	uint32_t elapsed_ms;

	if (!ball_speed_estimator_initialized) {
		ball_speed_reference_position_mm = position_mm;
		ball_speed_reference_tick_ms = now_ms;
		ball_speed_last_motion_tick_ms = now_ms;
		ball_speed_estimator_initialized = 1U;
		ball_control_speed_mm_s = 0.0f;
		return 0.0f;
	}

	position_delta_mm = position_mm - ball_speed_reference_position_mm;
	if (fabsf(position_delta_mm) >= BALL_CONTROL_NEW_SAMPLE_EPSILON_MM) {
		elapsed_ms = now_ms - ball_speed_reference_tick_ms;
		if (elapsed_ms > 0U) {
			const float dt = (float)elapsed_ms * 0.001f;
			const float rc = 1.0f / (6.2831853f * BALL_CONTROL_SPEED_FILTER_HZ);
			const float alpha = dt / (rc + dt);
			float raw_speed_mm_s = position_delta_mm / dt;

			raw_speed_mm_s = Ball_Control_Clamp(raw_speed_mm_s,
				-BALL_CONTROL_SPEED_ESTIMATE_LIMIT_MM_S,
				 BALL_CONTROL_SPEED_ESTIMATE_LIMIT_MM_S);
			ball_control_speed_mm_s +=
				alpha * (raw_speed_mm_s - ball_control_speed_mm_s);
		}

		ball_speed_reference_position_mm = position_mm;
		ball_speed_reference_tick_ms = now_ms;
		ball_speed_last_motion_tick_ms = now_ms;
	}
	else if ((now_ms - ball_speed_last_motion_tick_ms) >=
		BALL_CONTROL_SPEED_ZERO_TIMEOUT_MS) {
		ball_control_speed_mm_s = 0.0f;
	}

	return ball_control_speed_mm_s;
}

/**
 * @brief UART1 空闲时发送一条绝对位置命令。
 * @note  底层命令帧使用静态 DMA 缓冲区，因此 UART 忙时绝不能改写下一帧。
 */
static void Ball_Control_Send_Motor_Command(int32_t command_pulse, uint32_t now_ms)
{
	int32_t command_delta;
	uint32_t saved_primask;

	command_delta = command_pulse - ball_motor_last_sent_pulse;
	if (command_delta < 0)
		command_delta = -command_delta;

	if (ball_motor_command_sent &&
		(command_delta < BALL_MOTOR_SEND_HYSTERESIS_PULSE))
		return;
	if (ball_motor_command_sent &&
		((now_ms - ball_motor_last_send_tick_ms) < BALL_MOTOR_MIN_SEND_INTERVAL_MS))
		return;

	/* 检查和启动 DMA 必须连续完成，避免更高优先级中断抢占 UART1。 */
	saved_primask = __get_PRIMASK();
	__disable_irq();
	if (huart1.gState == HAL_UART_STATE_READY) {
		BSP_Emm_V5_Pos_Control(command_pulse);
		ball_motor_last_sent_pulse = command_pulse;
		ball_motor_last_send_tick_ms = now_ms;
		ball_motor_command_sent = 1U;
	}
	__DMB();
	if (saved_primask == 0U)
		__enable_irq();
}

/** 激光无效或控制被关闭时，清除内环状态并让管道回到水平位置。 */
static void Ball_Control_Enter_Safe_State(uint32_t now_ms)
{
	Ball_Control_Reset_PID(&ball_speed_pid);
	ball_speed_estimator_initialized = 0U;
	ball_control_speed_mm_s = 0.0f;
	ball_control_target_speed_mm_s = 0.0f;
	ball_control_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE;
	Ball_Control_Send_Motor_Command(BALL_MOTOR_LEVEL_PULSE, now_ms);
}


volatile float next_motor_pulse;
/**
	* @brief	小球速度内环任务：目标/实际速度 -> 电机绝对位置脉冲。
	* @note		在 BALL_CONTROL_SPEED_LOOP_HZ 对应的固定频率定时中断中调用。
	* @param	无
	* @retval	无
	*/
void User_Task_Speed_Control(void)
{
	const float dt = 1.0f / BALL_CONTROL_SPEED_LOOP_HZ;
	const float maximum_motor_step = BALL_MOTOR_PULSE_SLEW_PER_SECOND * dt;
	const float minimum_absolute_pulse =
		(float)(BALL_MOTOR_LEVEL_PULSE + BALL_MOTOR_MIN_RELATIVE_PULSE);
	const float maximum_absolute_pulse =
		(float)(BALL_MOTOR_LEVEL_PULSE + BALL_MOTOR_MAX_RELATIVE_PULSE);
	float measured_position_mm;
	float measured_speed_mm_s;
	float target_speed_mm_s;
	float relative_motor_pulse;
	float target_motor_pulse;

	uint32_t now_ms;
	uint32_t saved_primask;
	uint8_t laser_valid;

	now_ms = HAL_GetTick();

	/* 同时发布位置和有效标志，保证外环不会读到一新一旧的组合。 */
	saved_primask = __get_PRIMASK();
	__disable_irq();
	User_Task_Laser_UART_Get(&measured_position_mm);
	laser_valid = ball_control_laser_valid;
	if (laser_valid)
		ball_control_position_mm = measured_position_mm;
	__DMB();
	if (saved_primask == 0U)
		__enable_irq();

	if ((!ball_control_enabled) || (!laser_valid)) {
		Ball_Control_Enter_Safe_State(now_ms);
		return;
	}

	measured_speed_mm_s =
		Ball_Control_Update_Speed_Estimate(measured_position_mm, now_ms);
	target_speed_mm_s = ball_control_target_speed_mm_s;

	/* 球已接近静止且外环不再要求移动时，主动清空积分并保持水平。 */
	if ((fabsf(target_speed_mm_s) <= BALL_SPEED_DEADBAND_MM_S) &&
		(fabsf(measured_speed_mm_s) <= BALL_SPEED_DEADBAND_MM_S)) {
		Ball_Control_Reset_PID(&ball_speed_pid);
		relative_motor_pulse = 0.0f;
	}
	else {
		relative_motor_pulse = PID_Compute(&ball_speed_pid,
			target_speed_mm_s,
			measured_speed_mm_s,
			dt);
	}

	relative_motor_pulse *= BALL_CONTROL_POLARITY;
	relative_motor_pulse = Ball_Control_Clamp(relative_motor_pulse,
		(float)BALL_MOTOR_MIN_RELATIVE_PULSE,
		(float)BALL_MOTOR_MAX_RELATIVE_PULSE);
	target_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE + relative_motor_pulse;
	next_motor_pulse = Ball_Control_Approach(ball_control_motor_pulse,
		target_motor_pulse,
		maximum_motor_step);
	next_motor_pulse = Ball_Control_Clamp(next_motor_pulse,
		minimum_absolute_pulse,
		maximum_absolute_pulse);

	ball_control_motor_pulse = next_motor_pulse;
	Ball_Control_Send_Motor_Command(Ball_Control_Round_To_Int32(next_motor_pulse),
		now_ms);
}

/**
	* @brief	小球位置外环任务：目标/实际位置 -> 小球目标速度。
	* @note		在 BALL_CONTROL_POSITION_LOOP_HZ 对应的固定频率定时中断中调用。
	* @param	无
	* @retval	无
	*/
void User_Task_Position_Control(void)
{
	const float dt = 1.0f / BALL_CONTROL_POSITION_LOOP_HZ;
	const float maximum_target_speed_step = BALL_TARGET_SPEED_SLEW_MM_S2 * dt;
	float measured_position_mm;
	float target_position_mm;
	float requested_speed_mm_s;
	uint32_t saved_primask;
	uint8_t control_enabled;
	uint8_t laser_valid;

	/* 位置和值的有效标志由速度任务一起发布，这里用短临界区取得一致快照。 */
	saved_primask = __get_PRIMASK();
	__disable_irq();
	control_enabled = ball_control_enabled;
	laser_valid = ball_control_laser_valid;
	measured_position_mm = ball_control_position_mm;
	target_position_mm = ball_control_target_position_mm;
	__DMB();
	if (saved_primask == 0U)
		__enable_irq();

	if ((!control_enabled) || (!laser_valid)) {
		Ball_Control_Reset_PID(&ball_position_pid);
		ball_control_target_speed_mm_s = 0.0f;
		return;
	}

	if (fabsf(target_position_mm - measured_position_mm) <=
		BALL_POSITION_DEADBAND_MM) {
		Ball_Control_Reset_PID(&ball_position_pid);
		requested_speed_mm_s = 0.0f;
	}
	else {
		requested_speed_mm_s = PID_Compute(&ball_position_pid,
			target_position_mm,
			measured_position_mm,
			dt);
	}

	ball_control_target_speed_mm_s = Ball_Control_Approach(
		ball_control_target_speed_mm_s,
		requested_speed_mm_s,
		maximum_target_speed_step);
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
	//**************屏幕测试的屏幕显示***********
	// static int my_count = 0;       // 自加的数字
	// static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	// my_count++;					   // 自加
	// sprintf(str_buff, "Count: %d", my_count);			//组合打印内容,这里是把数字变成字符
	// ssd1306_Fill(Black);									//清空屏幕缓冲区
	// ssd1306_SetCursor(10, 20);							//锁定打印位置
	// ssd1306_WriteString(str_buff, Font_11x18, White);	//确定打印字符,大小,颜色
	// ssd1306_UpdateScreen_IT();							//启动 I2C 中断刷新，函数立即返回，不阻塞主循环
	
	
	//**************关于方案1的屏幕显示***********
	//现在想要显示参数,分别是CONTROL_MOTOR_SPEED_RPM和CONTROL_MOTOR_ACCEL_PARAM两个宏定义的值
//	static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
//	ssd1306_Fill(Black);			//清空屏幕缓冲区

//	ssd1306_SetCursor(10, 5);									//锁定打印位置
//	sprintf(str_buff, "Count: %d", UPdate_Speed_RPM);	//组合打印内容,这里是把数字变成字符
//	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

//	ssd1306_SetCursor(10, 20);									//锁定打印位置
//	sprintf(str_buff, "Count: %d", UPdate_Accel_Param);	//组合打印内容,这里是把数字变成字符
//	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

//	//打印激光数据
//	static float laser_distance_mm = 0.0f;
//	User_Task_Laser_UART_Get(&laser_distance_mm);	//获取激光数据

//	ssd1306_SetCursor(10, 35);									//锁定打印位置
//	sprintf(str_buff, "Laser: %.2f", laser_distance_mm);	//组合打印内容,这里是把数字变成字符
//	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

	//**************关于方案2的屏幕显示***********
	//现在想要显示参数,分别是内环PID参数,外环PID参数
	static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	ssd1306_Fill(Black);					//清空屏幕缓冲区

	ssd1306_SetCursor(5, 5);									//锁定打印位置
	sprintf(str_buff, "Kp:%.2f H %.2f",ball_speed_pid.Kp,ball_position_pid.Kp);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

	ssd1306_SetCursor(5, 20);									//锁定打印位置
	sprintf(str_buff, "Ki:%.2f H %.2f",ball_speed_pid.Ki,ball_position_pid.Ki);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色
	
	ssd1306_SetCursor(5, 35);									//锁定打印位置
	sprintf(str_buff, "Kd:%.2f H %.2f",ball_speed_pid.Kd,ball_position_pid.Kd);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

	//打印激光数据
	static float laser_distance_mm = 0.0f;
	User_Task_Laser_UART_Get(&laser_distance_mm);	//获取激光数据

	ssd1306_SetCursor(5, 50);									//锁定打印位置
	sprintf(str_buff, "L:%.2f H %.0f", laser_distance_mm,next_motor_pulse);	//组合打印内容,这里是把数字变成字符
	ssd1306_WriteString(str_buff, Font_7x10, White);			//确定打印字符,大小,颜色

	/* 启动 I2C 中断刷新；若上一帧尚未完成，本次调用会直接返回。 */
	(void)ssd1306_UpdateScreen_IT();
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
	//关于陀螺仪数据打印
//	MPU6050_t mpu6050_date = {0};			//声明局部结构体
//	User_Task_MPU6050_Get(&mpu6050_date);	//获取陀螺仪数据

//	printf_dma("x轴加速度%d x轴角速度%d \r\n", mpu6050_date.Accel_X_RAW, mpu6050_date.Gyro_X_RAW);
//	printf_dma("y轴加速度%d y轴角速度%d \r\n", mpu6050_date.Accel_Y_RAW, mpu6050_date.Gyro_Y_RAW);
//	printf_dma("z轴加速度%d z轴角速度%d \r\n", mpu6050_date.Accel_Z_RAW, mpu6050_date.Gyro_Z_RAW);
//	printf_dma(" \r\n");
//	printf_dma("转化后x轴加速度%f g 转化后x轴角速度%f 度/s \r\n", mpu6050_date.Ax, mpu6050_date.Gx);
//	printf_dma("转化后y轴加速度%f g 转化后y轴角速度%f 度/s \r\n", mpu6050_date.Ay, mpu6050_date.Gy);
//	printf_dma("转化后z轴加速度%f g 转化后z轴角速度%f 度/s \r\n", mpu6050_date.Az, mpu6050_date.Gz);
//	printf_dma(" \r\n");
//	printf_dma("当前x轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleX);
//	printf_dma("当前y轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleY);
//	printf_dma(" \r\n");
	
	
	//vofa+波形图测试代码
//	static uint8_t status_print=0;
//	if(status_print==0){
//		printf_dma("d: %f, %f\n", 100.0f,200.0f);
//		status_print=1;
//	}else{
//		printf_dma("d: %f, %f\n", -100.0f,-200.0f);
//		status_print=0;
//	}
	
	//关于vofa+的在线调参
	//打印激光数据(位置数据)
	static float laser_distance_mm = 0.0f;
	User_Task_Laser_UART_Get(&laser_distance_mm);	//获取激光数据
	
	//打印
	float Tem_next_motor_pulse= next_motor_pulse;
	//1:位置数据. 2:当前电机输出
	printf_dma("POS: %f,%f\n", laser_distance_mm,Tem_next_motor_pulse);

}

