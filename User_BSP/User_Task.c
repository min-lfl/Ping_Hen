/**
 * @file    User_Task.c
 * @brief   用户任务模块实现，包含所有业务逻辑的"任务"函数。
 *
 * ============================================================================
 * 文件结构概览
 * ============================================================================
 *
 * 本文件按功能模块分为以下几个区域（用 //#### 分隔线标记）：
 *
 *   1. User_Task_Init          — 总体初始化，按顺序调用各模块的 Init。
 *   2. 参数更新                   — User_Task_Param_Update，运行时修改电机参数。
 *   3. 陀螺仪模块                — MPU6050 的初始化、读取、获取。
 *   4. 激光测距模块              — 激光 DMA 后台接收的初始化和数据读取。
 *   5. 按键模块                   — 三个按键的扫描和处理。
 *   6. 加速度补偿控制（方案 1）   — 已弃用，基于加速度计估算杆倾角的控制方案。
 *   7. 串级控制（方案 2，当前）  — ★ 核心：位置外环 + 速度内环的串级 PID 控制。
 *   8. OLED 显示                 — 屏幕刷新，显示 PID 参数和传感器数据。
 *   9. 串口输出                   — VOFA+ 调试数据发送。
 *
 * ============================================================================
 * 方案 2 串级控制详解（核心控制逻辑）
 * ============================================================================
 *
 * 控制目标：让小球停在管道中间的零点，不左右振荡。
 *
 * 为什么需要串级控制？
 *   单层 PID（直接把位置误差映射为电机脉冲）的问题是：
 *   小球位置变化很慢（因为有惯性），等位置误差变大了电机才反应，
 *   反应又会导致小球加速，等小球速度上来又刹不住，不断过冲。
 *   串级控制在中间加了一层"速度环"，专门负责让小球速度受控，
 *   位置外环只负责告诉速度内环"你应该跑多快"。
 *
 * 数据流：
 *
 *   激光传感器 (约10Hz)
 *     │
 *     ▼
 *   Laser_UART_GetDistance()  ← 读取后台 DMA 缓存，非阻塞
 *     │
 *     ├──→ 位置值 ──→ ball_control_position_mm
 *     │                  │
 *     │                  ▼
 *     │     User_Task_Position_Control()  ← TIM3 中断，10Hz
 *     │       输入：目标位置=0, 实际位置
 *     │       输出：ball_control_target_speed_mm_s
 *     │                  │
 *     │                  ▼
 *     │     User_Task_Speed_Control()  ← TIM2 中断，100Hz
 *     │       输入：目标速度, 实际速度（由位置差分估算）
 *     │       输出：电机脉冲数
 *     │                  │
 *     │                  ▼
 *     │     Ball_Control_Send_Motor_Command()
 *     │       → BSP_Emm_V5_Pos_Control()
 *     │         → 张大头驱动器 UART 命令
 *     │           → 步进电机转动
 *     │             → 杆倾斜
 *     │               → 重力分力 → 小球移动
 *     │
 *     └──→ 位置差分 + 低通滤波 → ball_control_speed_mm_s
 *                                   ↑
 *                              Ball_Control_Update_Speed_Estimate()
 *
 * 关键设计决策：
 *   1. 外环 10Hz、内环 100Hz：激光数据约 10Hz，外环不需要更快；
 *      内环更快是因为速度估算器可以在 100Hz 输出平滑滤波值，电机响应也更快。
 *   2. 速度估算用位置差分 + 低通滤波：没有速度传感器，只能用位置差分。
 *      低通滤波平滑了差分结果的噪声，但也引入了 ~53ms 的延迟。
 *   3. 电机命令通过 UART1 DMA 发送：DMA 不阻塞 CPU，但忙时不能发送新命令。
 *   4. PID 过零清零：在 PID_Compute 中，当误差跨越零点时自动清零积分，
 *      防止积分在过零后继续往旧方向推，导致过冲。
 *   5. 安全状态：激光无效或控制关闭时，电机回到水平位置，清除 PID 状态。
 */

#include "User_Task.H"
#include "Control_Config.h"
#include <math.h>

/**
 * @brief   总体初始化函数。
 *
 * 目标：按正确的顺序初始化所有硬件模块。
 *
 * 实现细节：
 *   - 先初始化陀螺仪和 OLED，因为它们不依赖其他模块。
 *   - 等待 500ms 让电路和传感器稳定。
 *   - 初始化电机（发送 QPos 参数），再等 5ms 确保命令发送完成。
 *   - 初始化方案 1 的控制状态（即使方案 1 已弃用，Control_Init 仍被调用以保持兼容）。
 *   - 最后初始化激光，因为它需要较长的启动时间，放在最后不影响其他模块。
 *
 * 注意：初始化完成后定时器才开启，所以初始化期间不会触发控制中断。
 *
 * @param   无
 * @retval  无
 */
void User_Task_Init(void){

	// 初始化所有任务
	User_Task_MPU6050_Init();	// 初始化陀螺仪：I2C 通信 + 零偏校准
	User_Task_OLED_Init();		// 初始化 OLED：清空屏幕

	/*
	 * 上电延时 500ms：
	 *   1. 等待 MPU6050 完成内部启动和校准。
	 *   2. 等待电机驱动器上电就绪。
	 *   3. 避免上电瞬间的电压波动影响后续初始化。
	 */
	HAL_Delay(500);

	// 发送电机位置模式参数（速度、加速度、绝对/相对模式、多机同步标志）
	BSP_Emm_V5_Pos_Init();
	HAL_Delay(5);			// 确保 UART DMA 发送完成

	// 初始化方案 1 的控制状态（加速度滤波、S 曲线规划器）
	Control_Init();

	// 启动激光 DMA 后台接收（传感器上电后会自动连续发送数据）
	User_Task_Laser_UART_Init();
}

//##################################################################################################
//********关于参数更新模块任务***************************************************************************
//##################################################################################################
/**
 * @brief   参数更新任务。
 *
 * 目标：允许在运行时修改电机的速度和加速度参数，而不需要重新编译。
 *
 * 实现：通过"更新缓存"机制避免重复发送：
 *   - 维护两个静态缓存变量，记录上次成功发送的参数值。
 *   - 每次调用时，将当前参数与缓存对比。
 *   - 只有参数发生变化时才调用 Emm_V5_Set_QPos_Params 发送，并同步更新缓存。
 *   - 第一次调用时缓存未初始化，一定会发送。
 *
 * 好处：主循环中每次 30ms 都调用该函数，但参数很少变。
 *        如果不加缓存对比，每次循环都通过 UART1 DMA 发送一帧参数，
 *        既浪费总线带宽，也可能与电机控制命令产生 DMA 竞争。
 *
 * @param   无
 * @retval  无
 */
volatile uint16_t UPdate_Speed_RPM = CONTROL_MOTOR_SPEED_RPM;
volatile uint8_t UPdate_Accel_Param = CONTROL_MOTOR_ACCEL_PARAM;
void User_Task_Param_Update(void){
	/*
	 * 更新缓存：记录上次成功发送的参数值。
	 * static 保证值在函数调用之间保持，cached 标志区分"从未发送"和"已发送过"。
	 */
	static uint16_t cached_speed_rpm = 0;
	static uint8_t  cached_accel_param = 0;
	static uint8_t  cache_initialized = 0;

	/* 缓存未初始化 或 参数发生变化 → 发送并更新缓存 */
	if ((!cache_initialized) ||
		(UPdate_Speed_RPM != cached_speed_rpm) ||
		(UPdate_Accel_Param != cached_accel_param)) {

		Emm_V5_Set_QPos_Params(1,				// 电机地址固定为 1
		                   UPdate_Speed_RPM,	// 用户可修改的速度
		                   UPdate_Accel_Param,	// 用户可修改的加速度
		                   0x01,				// raF=1：绝对位置模式
		                   EMM_V5_SNF);		// 多机同步标志（本项目单机，填 false）

		/* 同步更新缓存，下次对比时以此为基准 */
		cached_speed_rpm  = UPdate_Speed_RPM;
		cached_accel_param = UPdate_Accel_Param;
		cache_initialized  = 1;
	}
}


//##################################################################################################
//********关于陀螺仪模块任务**************************************************************************
//##################################################################################################
/**
 * @brief   陀螺仪初始化。
 *
 * 目标：初始化 MPU6050 并校准陀螺仪零偏。
 *
 * 实现：
 *   1. 通过 I2C2 初始化 MPU6050。
 *   2. 调用 MPU6050_CalibrateGyro 采集 300 个样本，计算陀螺仪零偏并写入寄存器。
 *   3. 校准成功后将 mpu_ready 标志置 1，后续 Update 才能正常读取。
 *
 * @param   无
 * @retval  无
 */
static uint8_t mpu_ready = 0;
void User_Task_MPU6050_Init(void){
	// 初始化并且上电静止校准陀螺仪
	if (MPU6050_Init(&hi2c2) == 0)
	{
		printf_dma("Keep MPU still: calibrating gyro...\r\n");
		if (MPU6050_CalibrateGyro(&hi2c2, 300) == 0)
			mpu_ready = 1;		// 校准成功，允许后续读取
		else
			printf_dma("MPU gyro calibration failed\r\n");
	}
	else
	{
		printf_dma("MPU WHO_AM_I or I2C failed\r\n");
	}
}


// 定义数据缓冲区（局部静态变量，避免频繁分配和释放内存）
static MPU6050_t mpu6050_date = {0};
/**
 * @brief   陀螺仪数据更新。
 *
 * 目标：读取 MPU6050 的最新数据并更新方案 1 的加速度输入。
 *
 * 实现：
 *   1. 检查 mpu_ready 标志，确保已校准。
 *   2. 调用 MPU6050_Read_All 读取加速度、角速度、姿态角。
 *   3. 将 Ay 和 Az 传给 Control_Input_Update（方案 1 的倾角计算）。
 *
 * 注意：这个函数在主循环中调用，频率由主循环的 HAL_Delay 决定。
 *
 * @param   无
 * @retval  无
 */
void User_Task_MPU6050_Update(void){
	if(mpu_ready) {
		MPU6050_Read_All(&hi2c2, &mpu6050_date, 300); // 读取所有数据，采样频率 300Hz
		Control_Input_Update((float)mpu6050_date.Ay, (float)mpu6050_date.Az);
	}
	else
		printf_dma("MPU not ready, please check initialization and calibration\r\n");
}

/**
 * @brief   获取陀螺仪数据副本。
 *
 * 目标：将内部缓冲区的陀螺仪数据安全地复制给外部调用者。
 *
 * 实现：直接结构体赋值复制整个 mpu6050_date 到调用者提供的指针。
 *        注意：调用者应在 User_Task_MPU6050_Update() 之后调用，
 *        否则获取到的可能是旧数据。
 *
 * @param   data 指向 MPU6050_t 结构体的指针，用于接收数据副本。
 * @retval  无
 */
void User_Task_MPU6050_Get(MPU6050_t* data){
	if (data != NULL) {
		// 把 mpu6050_date 的数据复制一份给外部使用
		*data = mpu6050_date;
	}
}

//##################################################################################################
//**********************关于激光串口模块任务**********************************************************
//##################################################################################################
/**
 * @brief   激光串口初始化。
 *
 * 目标：启动激光传感器的 DMA + IDLE 后台接收。
 *
 * 实现：调用 Laser_UART_Init 配置 UART2 的 DMA 接收。
 *        传感器上电后会主动连续发送数据帧，不需要单片机发送任何初始化命令。
 *        数据帧到达时，UART IDLE 中断自动触发解析，更新后台缓存。
 *
 * @param   无
 * @retval  无
 */
void User_Task_Laser_UART_Init(void){
	Laser_UART_Init(&huart2);
}


/**
 * @brief   获取激光后台缓存的最新位置。
 *
 * 目标：读取激光传感器最近一次有效测量的距离，并转换为相对于零点的位置。
 *
 * 实现：
 *   1. 调用 Laser_UART_GetDistance 读取 DMA 后台缓存的最新值。
 *      （不是主动读串口，只是复制缓存，非常快，不会阻塞。）
 *   2. 减去 BALL_CONTROL_LASER_ORIGIN_MM 得到相对位置。
 *      正数 = 在零点右侧，负数 = 在零点左侧。
 *   3. 更新 ball_control_laser_valid 标志，供控制任务判断数据是否有效。
 *
 * 注意：本函数在速度内环的中断中调用，需要极短执行时间。
 *       它只复制缓存、不访问串口，确保了中断安全。
 *
 * @param   distance_mm 输出相对零点的位置，单位 mm。正数在原点右侧，负数在原点左侧。
 * @retval  无
 */
void User_Task_Laser_UART_Get(float *distance_mm){
	if (distance_mm != NULL) {
		if (Laser_UART_GetDistance(distance_mm)) {
			*distance_mm -= BALL_CONTROL_LASER_ORIGIN_MM;	// 减去原点偏移得到相对位置
			__DMB();										// 内存屏障，确保写入完成后才更新标志
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
 * @brief   按键扫描任务。
 *
 * 目标：处理三个按键的按下事件，提供手动控制电机的能力。
 *
 * 实现：
 *   - PB13：电机正转到 +250 脉冲位置（杆向右倾斜）
 *   - PB14：电机回到零点 + 切换自动/手动控制模式
 *   - PB15：电机反转到 -250 脉冲位置（杆向左倾斜）
 *   每个按键都有软件消抖（20ms 延时 + 二次确认 + 等待释放）。
 *
 * 注意：按键中使用了 while 循环等待释放，会阻塞主循环。
 *        但在手动控制模式下控制任务不运行，所以不影响。
 *
 * @param   无
 * @retval  无
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

					// 自动控制切换开关
					if(control_automatic_enabled==1){
						control_automatic_enabled=0;		// 暂时禁用自动控制
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

/*
 * 方案 1 原理简述：
 *
 *   目标：直接通过加速度计估算杆的倾角，然后映射为电机脉冲，控制杆保持水平。
 *
 *   步骤：
 *     1. 读取 MPU6050 的 Ay 和 Az 加速度
 *     2. 用二阶低通滤波器滤除振动噪声
 *     3. 用 atan2(Ay, Az) 计算杆的倾角（单位弧度，转为度）
 *     4. 通过校准表（linkage_cal_table）将倾角线性插值映射为电机脉冲数
 *     5. 用软件 S 曲线规划器（质量-弹簧-阻尼模型）平滑过渡到目标位置
 *     6. 发送绝对位置命令给电机
 *
 *   弃用原因：
 *     加速度计测量的是"杆的加速度 + 重力分量"，无法区分"杆倾斜"和"小球运动产生的加速度"。
 *     当杆水平但小球在滚动时，加速度计会读到非零的 Ay，被误判为杆倾斜，
 *     导致控制器错误地倾斜杆，反而加速了小球，形成正反馈发散。
 *     改为方案 2（激光测距 + 串级 PID）后问题解决。
 *
 *   以下代码保留仅供学习参考，实际运行时不会执行（control_automatic_enabled 控制）。
 */

#define CONTROL_RAD_TO_DEG (57.2957795f)	// 弧度转角度：180/π

/* 校准表：每个 (杆倾角, 电机脉冲) 对记录了机械联动关系。
 * 杆倾角必须严格递增，脉冲数对应倾角下的电机位置。 */
typedef struct {
	float rod_angle_deg;		// 杆的实际倾角，单位度
	int32_t motor_pulse;		// 对应的电机绝对位置脉冲数
} LinkageCalPoint_t;

/* 二阶低通滤波器状态：stage_1 是第一级输出，stage_2 是第二级输出。
 * 两级串联等效于一个二阶 Butterworth 低通滤波器。
 * 目的是滤除加速度计的高频振动噪声，同时保留杆倾角的低频变化。 */
typedef struct {
	float stage_1;				// 第一级滤波器输出
	float stage_2;				// 第二级滤波器输出（最终输出）
	uint8_t initialized;		// 是否已初始化（首次输入直接赋值，避免滤波启动瞬态）
} ControlLowPass_t;

/* S 曲线规划器的运动状态：位置、速度、加速度。
 * 模拟一个质量-弹簧-阻尼系统的物理运动，实现平滑的轨迹过渡。 */
typedef struct {
	float position;			// 规划器当前位置，单位 pulse
	float velocity;			// 规划器当前速度，单位 pulse/s
	float acceleration;		// 规划器当前加速度，单位 pulse/s^2
} ControlMotion_t;

/*
 * 连杆校准表：记录了 7 个采样点的杆倾角与电机脉冲的对应关系。
 * 杆倾角必须严格递增。如果重新校准，替换每个角度值即可。
 *
 * 使用方式：给定目标倾角，在表中进行线性插值，得到对应的电机脉冲数。
 * 例如：倾角 2.0° 在 (0°, 0) 和 (2.826°, 83) 之间，插值得到约 58.5 脉冲。
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

/* 方案 1 的调试变量，供外部读取观察 */
volatile float control_debug_ay_filtered_g = 0.0f;
volatile float control_debug_az_filtered_g = 1.0f;
volatile float control_debug_target_angle_deg = 0.0f;
volatile float control_debug_target_pulse = 0.0f;
volatile float control_debug_command_pulse = 0.0f;
volatile float control_debug_profile_speed = 0.0f;
volatile uint8_t control_automatic_enabled = CONTROL_AUTOMATIC_ENABLE;

/* 方案 1 的内部状态（static，外部不可见） */
static ControlLowPass_t control_ay_filter = {0};
static ControlLowPass_t control_az_filter = {0};
static ControlMotion_t control_motion = {0};
static volatile float control_target_pulse = (float)CONTROL_LEVEL_PULSE;
static volatile uint8_t control_target_valid = 0;
static int32_t control_last_sent_pulse = CONTROL_LEVEL_PULSE;
static uint8_t control_command_sent = 0;


/*
 * 通用工具函数：数值限幅。
 * 把 value 限制在 [minimum, maximum] 范围内。
 */
static float Control_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}

/*
 * 通用工具函数：渐进逼近。
 * 从 current 向 target 移动，但每步最多移动 maximum_step。
 * 用于实现平滑的斜率限制，防止值突变。
 *
 * 例如：current=0, target=100, maximum_step=10
 *       第一次调用返回 10，第二次 20，... 直到 100。
 */
static float Control_Approach(float current, float target, float maximum_step)
{
	if (target > current + maximum_step)
		return current + maximum_step;
	if (target < current - maximum_step)
		return current - maximum_step;
	return target;
}

/*
 * 二阶低通滤波器更新。
 *
 * 目标：滤除加速度计的高频振动，保留杆倾角的低频变化。
 *
 * 实现：
 *   两个一阶低通滤波器串联 = 二阶 Butterworth 低通滤波器。
 *   每个一阶滤波器的公式：output += alpha * (input - output)
 *   其中 alpha = dt / (RC + dt)，RC = 1/(2π × 截止频率)
 *
 *   首次调用时直接初始化滤波器状态为输入值，避免从 0 慢慢爬升。
 *
 * 参数：
 *   filter：滤波器状态结构体
 *   input：新的原始输入值
 * 返回：滤波后的输出值
 */
static float Control_LowPass_Update(ControlLowPass_t *filter, float input)
{
	const float dt = 1.0f / CONTROL_IMU_SAMPLE_HZ;						// 采样周期
	const float rc = 1.0f / (6.2831853f * CONTROL_ACCEL_LPF_POLE_HZ);	// RC 时间常数
	const float alpha = dt / (rc + dt);									// 滤波系数 (0~1)

	if (!filter->initialized) {
		// 首次调用：直接初始化为输入值，避免滤波瞬态
		filter->stage_1 = input;
		filter->stage_2 = input;
		filter->initialized = 1;
		return input;
	}

	// 两级串联一阶低通
	filter->stage_1 += alpha * (input - filter->stage_1);			// 第一级
	filter->stage_2 += alpha * (filter->stage_1 - filter->stage_2);	// 第二级
	return filter->stage_2;
}

/*
 * 校准表线性插值查找。
 *
 * 目标：给定杆倾角，找到对应的电机脉冲数。
 *
 * 实现：
 *   在校准表中找到包含目标倾角的区间 [left, right]，
 *   然后按比例线性插值：
 *     ratio = (目标倾角 - left倾角) / (right倾角 - left倾角)
 *     脉冲  = left脉冲 + ratio × (right脉冲 - left脉冲)
 *
 *   如果倾角超出表范围，返回最近端点的值（外推不安全，直接限幅）。
 *
 * 参数：
 *   rod_angle_deg：杆的目标倾角，单位度
 * 返回：对应的电机脉冲数
 */
static float Control_Linkage_Angle_To_Pulse(float rod_angle_deg)
{
	const uint32_t point_count = sizeof(linkage_cal_table) / sizeof(linkage_cal_table[0]);
	uint32_t index;

	// 倾角小于表最小值，返回最小脉冲
	if (rod_angle_deg <= linkage_cal_table[0].rod_angle_deg)
		return (float)linkage_cal_table[0].motor_pulse;

	// 查找包含目标倾角的区间
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

	// 倾角大于表最大值，返回最大脉冲
	return (float)linkage_cal_table[point_count - 1].motor_pulse;
}

/*
 * 方案 1 控制状态初始化。
 * 将所有滤波器和 S 曲线规划器重置为初始状态。
 */
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

/*
 * 方案 1 输入处理：加速度计 → 杆倾角 → 目标脉冲。
 *
 * 目标：将加速度计原始数据转换为电机的目标位置脉冲。
 *
 * 实现步骤：
 *   1. 减去零偏校准值，得到修正后的加速度。
 *   2. 对 Ay 进行输入限幅，防止异常大值。
 *   3. 用二阶低通滤波器滤除振动噪声。
 *   4. 用 atan2(Ay, |Az|) 计算杆倾角。
 *      - atan2 返回的是弧度，乘以 CONTROL_RAD_TO_DEG 转为度。
 *      - 使用 |Az| 而不是 Az，因为当杆倒置时 Az 为负，atan2 会出错。
 *      - 如果 |Az| < 0.25g，用 1.0g 代替，防止除零或极端值。
 *   5. 通过校准表线性插值将倾角映射为电机脉冲。
 *   6. 限幅到安全行程范围。
 *
 * 注意：这个函数在 TIM1 中断（300Hz）中调用，由 User_Task_MPU6050_Update 触发。
 */
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

	// 减去零偏校准值
	ay_corrected = ay_g - CONTROL_ACCEL_Y_BIAS_G;
	// 限幅：超出 ±0.3g 的加速度值被截断，防止振动噪声
	ay_corrected = Control_Clamp(ay_corrected,
		-CONTROL_ACCEL_INPUT_LIMIT_G,
		 CONTROL_ACCEL_INPUT_LIMIT_G);
	az_corrected = az_g - CONTROL_ACCEL_Z_BIAS_G;

	// 二阶低通滤波
	ay_filtered = Control_LowPass_Update(&control_ay_filter, ay_corrected);
	az_filtered = Control_LowPass_Update(&control_az_filter, az_corrected);

	// 用 |Az| 作为垂直分量，防止杆倒置时 atan2 出错
	vertical_g = fabsf(az_filtered);
	if (vertical_g < 0.25f)
		vertical_g = 1.0f;		// 太小则用 1g 代替，防止除零

	// atan2(Ay, |Az|) 计算杆倾角，再乘以极性符号
	target_angle_rad = CONTROL_ACCEL_TO_ROD_SIGN * atan2f(ay_filtered, vertical_g);
	target_angle_deg = target_angle_rad * CONTROL_RAD_TO_DEG;	// 弧度转度

	// 校准表查表：倾角 → 脉冲
	relative_pulse = Control_Linkage_Angle_To_Pulse(target_angle_deg);
	absolute_pulse = (float)CONTROL_LEVEL_PULSE + relative_pulse;
	absolute_pulse = Control_Clamp(absolute_pulse, minimum_pulse, maximum_pulse);

	// 更新调试变量和内部状态
	control_debug_ay_filtered_g = ay_filtered;
	control_debug_az_filtered_g = az_filtered;
	control_debug_target_angle_deg = target_angle_deg;
	control_debug_target_pulse = absolute_pulse;
	control_target_pulse = absolute_pulse;
	control_target_valid = 1;		// 标记目标位置有效
}

/*
 * 方案 1 的 S 曲线运动规划器。
 *
 * 目标：让电机从当前位置平滑过渡到目标位置，而不是瞬间跳变。
 *
 * 实现：
 *   用二阶质量-弹簧-阻尼模型模拟物理运动：
 *     F = m × a = -k × (x - target) - c × v
 *     其中 k = ω²，c = 2ζω
 *     desired_accel = ω² × error - 2ζω × velocity
 *
 *   然后对加速度、速度、位置逐级积分，并每级限幅。
 *   加加速度（Jerk）通过 Approach 限制加速度变化率来间接实现。
 *
 *   当位置、速度、加速度都接近目标时，直接锁定到目标位置，避免微小的数值残留。
 *
 * 参数：
 *   target_pulse：目标位置脉冲数
 * 返回：规划后的当前位置脉冲数
 */
static float Control_Motion_Update(float target_pulse)
{
	const float dt = 1.0f / CONTROL_LOOP_HZ;								// 控制周期
	const float natural_omega = 6.2831853f * CONTROL_PROFILE_NATURAL_HZ;	// 自然角频率 ω = 2πf
	const float minimum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MIN_RELATIVE_PULSE);
	const float maximum_pulse = (float)(CONTROL_LEVEL_PULSE + CONTROL_MAX_RELATIVE_PULSE);
	float error = target_pulse - control_motion.position;					// 位置误差
	float desired_acceleration;
	float next_position;

	/*
	 * 稳定检测：如果位置、速度、加速度都足够小，直接锁定到目标。
	 * 避免数值计算残留导致长期微小的振荡。
	 */
	if ((fabsf(error) <= CONTROL_SETTLE_POSITION_PULSE) &&
		(fabsf(control_motion.velocity) <= CONTROL_SETTLE_SPEED_PULSE_S) &&
		(fabsf(control_motion.acceleration) <= CONTROL_SETTLE_ACCEL_PULSE_S2)) {
		control_motion.position = target_pulse;
		control_motion.velocity = 0.0f;
		control_motion.acceleration = 0.0f;
		return control_motion.position;
	}

	/*
	 * 质量-弹簧-阻尼模型计算期望加速度：
	 *   a_desired = ω² × error - 2ζω × velocity
	 * 这是二阶系统的动力学方程：
	 *   - ω² × error：弹簧力，与位置误差成正比，把系统拉向目标
	 *   - 2ζω × velocity：阻尼力，与速度成正比，防止振荡
	 */
	desired_acceleration = natural_omega * natural_omega * error -
		2.0f * CONTROL_PROFILE_DAMPING * natural_omega * control_motion.velocity;
	desired_acceleration = Control_Clamp(desired_acceleration,
		-CONTROL_MAX_PULSE_ACCEL,
		 CONTROL_MAX_PULSE_ACCEL);

	// 加速度变化率限制（实现加加速度 Jerk 限制）
	control_motion.acceleration = Control_Approach(control_motion.acceleration,
		desired_acceleration,
		CONTROL_MAX_PULSE_JERK * dt);

	// 速度积分并限幅
	control_motion.velocity += control_motion.acceleration * dt;
	control_motion.velocity = Control_Clamp(control_motion.velocity,
		-CONTROL_MAX_PULSE_SPEED,
		 CONTROL_MAX_PULSE_SPEED);

	// 位置积分
	next_position = control_motion.position + control_motion.velocity * dt;

	// 位置限幅：碰到限位时速度归零，防止"撞墙反弹"
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
 * @brief   方案 1 控制任务（主循环中调用）。
 *
 * 目标：将 S 曲线规划器输出的位置转换为电机命令并发送。
 *
 * 实现：
 *   1. 检查自动控制是否启用。
 *   2. 获取目标脉冲（来自加速度计倾角估算）。
 *   3. 用 S 曲线规划器平滑过渡。
 *   4. 四舍五入转为整数，限幅到安全范围。
 *   5. 如果位置变化超过发送死区，通过 UART 发送给电机。
 *
 * 注意：当前方案 1 已弃用，这个函数在主循环中调用但不会执行有效控制。
 *       保留供学习和测试使用。
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
	command_pulse = (profiled_pulse >= 0.0f) ?								// 四舍五入取整
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

	// 位置变化超过死区才发送，减少 UART 占用
	if ((!control_command_sent) ||
		(command_delta >= CONTROL_SEND_HYSTERESIS_PULSE)) {
		BSP_Emm_V5_Pos_Control(command_pulse);
		control_last_sent_pulse = command_pulse;
		control_command_sent = 1;
	}
}



//##################################################################################################
//********关于速度环+位置环控制模块任务（方案 2，当前使用的核心控制逻辑）*****************************
//##################################################################################################

/*
 * ============================================================================
 * 方案 2 的全局状态变量
 * ============================================================================
 *
 * 这些变量是控制器的"对外接口"，由定时器中断中的控制任务写入，
 * 由主循环中的 OLED/串口任务读取。标记为 volatile 防止编译器优化掉读写。
 *
 * 32 位 float 在 Cortex-M3 上是单指令原子读写的，所以即使被中断打断
 * 也不会读到"半个"值。但 uint8_t 不是原子的，需要在读取时关中断。
 */

/*
 * 对外可观察状态：
 *   ball_control_enabled          — 1：运行串级控制；0：清除控制状态并回水平位。
 *                                    可通过按键 PB14 或调试器修改。
 *   ball_control_laser_valid      — 1：激光后台缓存中已有有效数据。
 *   ball_control_target_position_mm — 小球目标位置，默认 0（零点）。
 *   ball_control_position_mm       — 小球当前位置（激光测量值 - 原点偏移）。
 *   ball_control_speed_mm_s        — 小球滤波后速度，由速度估算器输出。
 *   ball_control_target_speed_mm_s — 位置外环给出的目标速度，速度内环跟踪此值。
 *   ball_control_motor_pulse       — 速度内环输出的电机绝对位置，发送给驱动器。
 */
volatile uint8_t ball_control_enabled = BALL_CONTROL_ENABLE_DEFAULT;
volatile uint8_t ball_control_laser_valid = 0U;
volatile float ball_control_target_position_mm = BALL_CONTROL_TARGET_POSITION_MM;
volatile float ball_control_position_mm = 0.0f;
volatile float ball_control_speed_mm_s = 0.0f;
volatile float ball_control_target_speed_mm_s = 0.0f;
volatile float ball_control_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE;

/* ---- 两个 PID 控制器的实例 ----
 *
 * 位置外环 PID：输出 = 小球目标速度 mm/s
 *   输入：误差 = 目标位置 - 实际位置（mm）
 *   Kp 决定"小球离目标越远就跑得越快"的程度
 *   Ki 消除稳态误差（如杆不水平导致的漂移）
 *   Kd 设为 0，阻尼由速度内环提供
 *
 * 速度内环 PID：输出 = 电机相对脉冲数
 *   输入：误差 = 目标速度 - 实际速度（mm/s）
 *   Kp 决定"速度误差多大就倾斜多大角度"
 *   Ki 消除速度稳态误差
 *   Kd 对速度误差求微分，起到阻尼（预测）作用
 */
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

/* ---- 速度估算器的内部状态（static，外部不可见） ----
 *
 * 速度估算器通过"位置差分 ÷ 时间"计算速度。
 * 但激光数据只有约 10Hz，而速度环是 100Hz。如果每次都差分，
 * 大部分时候位置没变，差分结果会是 0，偶尔才跳变一次。
 *
 * 解决方法：
 *   1. 记录上次有效差分时的位置和时间。
 *   2. 只有位置变化超过阈值时，才用新位置和旧位置差分。
 *   3. 位置不变时保持上次的速度估算值 + 低通滤波。
 *   4. 如果超过 250ms 位置没变，强制速度归零。
 *
 * 状态变量含义：
 *   ball_speed_estimator_initialized  — 是否已记录初始位置
 *   ball_speed_reference_position_mm  — 上次差分时的参考位置
 *   ball_speed_reference_tick_ms      — 上次差分时的时刻
 *   ball_speed_last_motion_tick_ms    — 上次检测到位置变化的时间（用于超时检测）
 */
static uint8_t ball_speed_estimator_initialized = 0U;
static float ball_speed_reference_position_mm = 0.0f;
static uint32_t ball_speed_reference_tick_ms = 0U;
static uint32_t ball_speed_last_motion_tick_ms = 0U;

/* ---- 电机命令发送状态（static，外部不可见） ----
 *
 * 用于抑制重复的 DMA 发送：
 *   ball_motor_last_sent_pulse  — 上次成功发送的脉冲值
 *   ball_motor_last_send_tick_ms — 上次发送的时刻
 *   ball_motor_command_sent      — 是否已经发送过至少一次命令
 */
static int32_t ball_motor_last_sent_pulse = BALL_MOTOR_LEVEL_PULSE;
static uint32_t ball_motor_last_send_tick_ms = 0U;
static uint8_t ball_motor_command_sent = 0U;

/*
 * ============================================================================
 * 通用工具函数
 * ============================================================================
 */

/*
 * 数值限幅：把 value 限制在 [minimum, maximum] 范围内。
 * 用于位置限幅、速度限幅、PID 输出限幅等。
 */
static float Ball_Control_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}

/*
 * 渐进逼近（斜率限制）：从 current 向 target 移动，但每步最多移动 maximum_step。
 *
 * 目标：防止电机指令突变。比如 PID 突然输出 +250 脉冲，但上一刻电机还在 -250，
 *       如果直接跳变会导致机械冲击。通过 Approach 限制每步变化量，实现平滑过渡。
 *
 * 在方案 2 中有两个用途：
 *   1. 电机目标位置的变化率限制（PULSE_SLEW_PER_SECOND）
 *   2. 位置外环目标速度的变化率限制（TARGET_SPEED_SLEW_MM_S2）
 */
static float Ball_Control_Approach(float current, float target, float maximum_step)
{
	if (target > current + maximum_step)
		return current + maximum_step;
	if (target < current - maximum_step)
		return current - maximum_step;
	return target;
}

/*
 * 浮点数四舍五入转整数。
 * 用于将 PID 计算出的浮点脉冲值转为电机命令需要的整数脉冲。
 */
static int32_t Ball_Control_Round_To_Int32(float value)
{
	return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

/*
 * 清除 PID 历史状态。
 *
 * 目标：当控制关闭、激光无效、或进入死区时，彻底清除 PID 的内部状态。
 *
 * 为什么要清除？
 *   PID 的积分项会累积历史误差。如果不清除，下次启动控制时，
 *   旧的积分值会让电机瞬间大幅倾斜，可能导致小球飞出。
 *   last_error 也会影响微分项，不清除也会导致瞬态输出。
 *
 * 所有字段都清零：error, last_error, integral, differential, output。
 */
static void Ball_Control_Reset_PID(PID_t *pid)
{
	pid->error = 0.0f;
	pid->last_error = 0.0f;
	pid->integral = 0.0f;
	pid->differential = 0.0f;
	pid->output = 0.0f;
}

/*
 * ============================================================================
 * 速度估算器
 * ============================================================================
 */

/**
 * @brief   根据激光位置变化估算小球速度，并对差分结果进行一阶低通滤波。
 *
 * 目标：在没有速度传感器的情况下，通过位置差分来估算小球速度，
 *        并且输出的是平滑的、可用于 PID 的速度值。
 *
 * 为什么不能简单地对位置做差分？
 *   激光传感器约 10Hz 更新一次位置，但速度内环是 100Hz。
 *   如果每次 100Hz 都差分，当位置没变时差分结果是 0，
 *   当新位置到达时差分结果突然跳变，形成锯齿波，
 *   PID 的 D 项会对这种跳变产生巨大的尖峰输出。
 *
 * 实现策略：
 *   1. 首次调用：记录初始位置和时间，返回速度 0。
 *   2. 后续调用：计算当前位置与上次参考位置的差值。
 *      如果差值 < 阈值（0.05mm），说明激光还没有新数据，
 *      保持上次的速度估算值不变（但会检查超时）。
 *      如果差值 ≥ 阈值，说明有新数据，做差分计算原始速度。
 *   3. 对原始速度做一阶低通滤波：speed += alpha * (raw - speed)
 *      其中 alpha = dt / (RC + dt)，RC = 1/(2π × 截止频率)
 *      截止频率 3Hz → 时间常数 ≈ 53ms。
 *   4. 原始速度限幅到 ±1000 mm/s，防止激光跳变产生尖峰。
 *   5. 如果 250ms 内位置没变化，强制速度归零（小球静止了）。
 *
 * 关键设计决策：
 *   - 用位置变化阈值来判断"是否有新数据"，而不是帧序号。
 *     因为帧序号在 Laser_uart 层是内部实现细节，不应该暴露出来。
 *   - 低通滤波引入了延迟（约 53ms），但这是必要的代价。
 *     没有滤波的话，速度差分噪声会完全破坏 PID 的 D 项。
 *   - 超时归零机制防止小球静止时速度估算残留一个小数。
 *
 * @param   position_mm  激光测量的当前位置（已减去原点偏移），单位 mm。
 * @param   now_ms       当前时刻的 HAL_GetTick() 值，单位 ms。
 * @return  滤波后的小球速度，单位 mm/s。正数 = 向右运动。
 */
static float Ball_Control_Update_Speed_Estimate(float position_mm, uint32_t now_ms)
{
	float position_delta_mm;
	uint32_t elapsed_ms;

	/*
	 * 首次调用：初始化参考位置和时间，速度设为 0。
	 * 无法做差分，因为没有历史数据。
	 */
	if (!ball_speed_estimator_initialized) {
		ball_speed_reference_position_mm = position_mm;
		ball_speed_reference_tick_ms = now_ms;
		ball_speed_last_motion_tick_ms = now_ms;
		ball_speed_estimator_initialized = 1U;
		ball_control_speed_mm_s = 0.0f;
		return 0.0f;
	}

	position_delta_mm = position_mm - ball_speed_reference_position_mm;

	/*
	 * 位置变化超过阈值 → 激光有新数据 → 做差分。
	 * 否则保持上次的速度估算值（不重复差分），但检查超时。
	 */
	if (fabsf(position_delta_mm) >= BALL_CONTROL_NEW_SAMPLE_EPSILON_MM) {
		elapsed_ms = now_ms - ball_speed_reference_tick_ms;
		if (elapsed_ms > 0U) {
			const float dt = (float)elapsed_ms * 0.001f;		// 时间间隔，单位秒

			/*
			 * 一阶低通滤波系数 alpha 的计算：
			 *   RC = 1/(2π × fc)  — 时间常数
			 *   alpha = dt / (RC + dt)
			 * 当 dt 很小（高频采样）时 alpha ≈ dt/RC，滤波效果强。
			 * 当 dt 很大（低频采样）时 alpha ≈ 1，几乎不滤波。
			 */
			const float rc = 1.0f / (6.2831853f * BALL_CONTROL_SPEED_FILTER_HZ);
			const float alpha = dt / (rc + dt);

			// 原始速度 = 位置变化量 / 时间间隔
			float raw_speed_mm_s = position_delta_mm / dt;

			// 尖峰限幅：防止激光偶尔跳变导致速度估算值异常大
			raw_speed_mm_s = Ball_Control_Clamp(raw_speed_mm_s,
				-BALL_CONTROL_SPEED_ESTIMATE_LIMIT_MM_S,
				 BALL_CONTROL_SPEED_ESTIMATE_LIMIT_MM_S);

			// 一阶低通滤波：speed += alpha × (raw - speed)
			ball_control_speed_mm_s +=
				alpha * (raw_speed_mm_s - ball_control_speed_mm_s);
		}

		// 更新参考位置和时间，为下次差分做准备
		ball_speed_reference_position_mm = position_mm;
		ball_speed_reference_tick_ms = now_ms;
		ball_speed_last_motion_tick_ms = now_ms;	// 记录最后一次检测到运动的时间
	}
	/*
	 * 位置没有变化，但检查是否超时。
	 * 如果超过 250ms 位置完全没变，说明小球确实静止了，速度应该归零。
	 * 不归零的话，滤波器的速度值会残留一个小数，PID 可能因此产生微小输出。
	 */
	else if ((now_ms - ball_speed_last_motion_tick_ms) >=
		BALL_CONTROL_SPEED_ZERO_TIMEOUT_MS) {
		ball_control_speed_mm_s = 0.0f;
	}

	return ball_control_speed_mm_s;
}

/*
 * ============================================================================
 * 电机命令发送
 * ============================================================================
 */

/**
 * @brief   通过 UART1 DMA 发送绝对位置命令给电机驱动器。
 *
 * 目标：安全地将 PID 输出的电机脉冲数发送给驱动器，同时避免 DMA 冲突。
 *
 * 为什么需要这个函数而不直接调用 BSP_Emm_V5_Pos_Control？
 *   1. DMA 安全：电机驱动器的 UART 命令帧使用静态 DMA 缓冲区。
 *      如果 UART1 正在 DMA 发送上一帧，此时改写缓冲区会导致数据错乱。
 *      所以必须在 UART 空闲时才能发送。
 *   2. 重复抑制：如果新命令和上次发送的命令几乎相同（变化 < 1 脉冲），
 *      就不重复发送，减少 UART 总线占用。
 *   3. 最小间隔：两次发送之间至少间隔 5ms，给 DMA 足够的完成时间。
 *   4. 中断安全：检查和发送必须在关中断临界区内完成，
 *      防止 TIM2 中断（速度内环）在检查状态和发送之间抢占 UART1。
 *
 * 实现细节：
 *   1. 计算命令变化量，如果太小且不是首次发送，跳过。
 *   2. 检查距离上次发送的时间，如果太短，跳过。
 *   3. 关中断，检查 UART1 是否空闲。
 *   4. 如果空闲，发送命令并更新状态；否则静默丢弃（下一帧会重试）。
 *   5. 恢复中断状态（如果原来就是开中断的）。
 *
 * 注意：如果 UART 忙，命令会被静默丢弃。由于控制环是 100Hz 的，
 *       下一帧会重新计算并尝试发送，所以偶尔丢一帧不影响控制。
 *
 * @param   command_pulse 电机的绝对位置脉冲数。
 * @param   now_ms        当前时刻的 HAL_GetTick() 值，单位 ms。
 */
static void Ball_Control_Send_Motor_Command(int32_t command_pulse, uint32_t now_ms)
{
	int32_t command_delta;
	uint32_t saved_primask;

	// 计算与上次发送命令的差值（绝对值）
	command_delta = command_pulse - ball_motor_last_sent_pulse;
	if (command_delta < 0)
		command_delta = -command_delta;

	// 重复抑制：如果已经发送过命令，且变化太小，跳过
	if (ball_motor_command_sent &&
		(command_delta < BALL_MOTOR_SEND_HYSTERESIS_PULSE))
		return;

	// 最小间隔：如果已经发送过命令，且间隔太短，跳过
	if (ball_motor_command_sent &&
		((now_ms - ball_motor_last_send_tick_ms) < BALL_MOTOR_MIN_SEND_INTERVAL_MS))
		return;

	/*
	 * 关中断临界区：检查 UART 状态和发送命令必须原子完成。
	 * 如果 TIM2 中断（100Hz）在"检查 UART 空闲"和"发送命令"之间
	 * 抢占并完成了上一次 DMA 发送，就会导致 UART 状态判断错误。
	 * 关中断确保这个窗口不会被抢占。
	 */
	saved_primask = __get_PRIMASK();	// 保存当前中断状态
	__disable_irq();					// 关闭所有中断
	if (huart1.gState == HAL_UART_STATE_READY) {
		BSP_Emm_V5_Pos_Control(command_pulse);	// 发送命令（设置 DMA 缓冲区并启动）
		ball_motor_last_sent_pulse = command_pulse;
		ball_motor_last_send_tick_ms = now_ms;
		ball_motor_command_sent = 1U;
	}
	__DMB();							// 内存屏障，确保写入完成
	if (saved_primask == 0U)
		__enable_irq();					// 恢复原来的中断状态
}

/*
 * ============================================================================
 * 安全状态处理
 * ============================================================================
 */

/**
 * @brief   进入安全状态：清除控制状态，电机回到水平位置。
 *
 * 目标：当激光数据无效或控制被关闭时，安全地停止控制。
 *
 * 触发条件：
 *   1. ball_control_enabled = 0（控制被手动关闭）
 *   2. ball_control_laser_valid = 0（激光没有有效数据）
 *
 * 执行动作：
 *   1. 清除速度内环 PID 的所有状态（积分、微分、误差）。
 *   2. 重置速度估算器（下次启动时重新初始化）。
 *   3. 将目标速度和实际速度都清零。
 *   4. 将电机位置强制设为水平位置（BALL_MOTOR_LEVEL_PULSE = 0）。
 *   5. 发送水平位置命令给电机。
 *
 * 注意：这里只清除速度内环。位置外环在自己的任务中检测到
 *       laser_valid=0 后也会自行清除状态。
 *
 * @param   now_ms 当前时刻的 HAL_GetTick() 值，单位 ms。
 */
static void Ball_Control_Enter_Safe_State(uint32_t now_ms)
{
	Ball_Control_Reset_PID(&ball_speed_pid);
	ball_speed_estimator_initialized = 0U;
	ball_control_speed_mm_s = 0.0f;
	ball_control_target_speed_mm_s = 0.0f;
	ball_control_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE;
	Ball_Control_Send_Motor_Command(BALL_MOTOR_LEVEL_PULSE, now_ms);
}

/*
 * ============================================================================
 * 速度内环控制任务（核心）
 * ============================================================================
 */

/*
 * 调试变量：next_motor_pulse 记录速度内环输出的电机脉冲数，
 * 供 OLED 和串口读取显示。标记为 volatile 因为它由中断写入。
 */
volatile float next_motor_pulse;

/**
 * @brief   小球速度内环任务：目标速度/实际速度 → 电机绝对位置脉冲。
 *
 * 目标：让小球的实际速度跟踪位置外环给出的目标速度。
 *
 * 数据流：
 *   输入：
 *     - ball_control_target_speed_mm_s（来自位置外环，mm/s）
 *     - 实际速度（由 Ball_Control_Update_Speed_Estimate 估算，mm/s）
 *   输出：
 *     - 电机绝对位置脉冲数（发送给电机驱动器）
 *   中间步骤：
 *     1. 读取激光位置，更新 ball_control_laser_valid 和 ball_control_position_mm。
 *     2. 如果控制未启用或激光无效 → 进入安全状态。
 *     3. 估算小球实际速度。
 *     4. 如果目标速度和实际速度都在死区内 → 认为小球已静止，清空 PID。
 *     5. 否则用 PID 计算电机相对脉冲。
 *     6. 乘以极性、限幅到安全范围。
 *     7. 用 Approach 限制电机位置变化率（防止突变）。
 *     8. 四舍五入后发送给电机。
 *
 * 为什么在中断中调用？
 *   这个函数在 TIM2 中断（100Hz）中调用，dt 严格等于 0.01s。
 *   如果放在主循环中，dt 会随其他任务执行时间变化，PID 计算不准确。
 *
 * 注意：这个函数执行期间会关中断来读取激光数据（临界区），
 *       但临界区极短（只复制几个值），不会影响其他中断。
 *
 * @note   在 BALL_CONTROL_SPEED_LOOP_HZ（100Hz）对应的 TIM2 中断中调用。
 * @param  无
 * @retval 无
 */
void User_Task_Speed_Control(void)
{
	// 预计算常量，避免每次循环重复计算
	const float dt = 1.0f / BALL_CONTROL_SPEED_LOOP_HZ;				// 0.01s
	const float maximum_motor_step = BALL_MOTOR_PULSE_SLEW_PER_SECOND * dt;	// 每周期最大脉冲变化量 = 1200 × 0.01 = 12
	const float minimum_absolute_pulse =
		(float)(BALL_MOTOR_LEVEL_PULSE + BALL_MOTOR_MIN_RELATIVE_PULSE);		// -280
	const float maximum_absolute_pulse =
		(float)(BALL_MOTOR_LEVEL_PULSE + BALL_MOTOR_MAX_RELATIVE_PULSE);		// +280
	float measured_position_mm;
	float measured_speed_mm_s;
	float target_speed_mm_s;
	float relative_motor_pulse;
	float target_motor_pulse;

	uint32_t now_ms;
	uint32_t saved_primask;
	uint8_t laser_valid;

	now_ms = HAL_GetTick();

	/*
	 * 临界区：同时读取激光数据和有效标志，保证一致性。
	 *
	 * 为什么需要关中断？
	 *   User_Task_Laser_UART_Get 会修改 ball_control_laser_valid 和
	 *   ball_control_position_mm。如果不关中断，可能在读取一半时被
	 *   另一个中断打断，导致读到"新有效标志 + 旧位置"的组合。
	 */
	saved_primask = __get_PRIMASK();
	__disable_irq();
	User_Task_Laser_UART_Get(&measured_position_mm);	// 读取激光位置
	laser_valid = ball_control_laser_valid;				// 读取有效标志
	if (laser_valid)
		ball_control_position_mm = measured_position_mm;	// 发布位置供外环使用
	__DMB();
	if (saved_primask == 0U)
		__enable_irq();

	// 控制未启用或激光无效 → 进入安全状态
	if ((!ball_control_enabled) || (!laser_valid)) {
		Ball_Control_Enter_Safe_State(now_ms);
		return;
	}

	/*
	 * 估算小球实际速度。
	 * 速度估算器内部会判断激光是否有新数据，没有新数据时保持上次速度。
	 */
	measured_speed_mm_s =
		Ball_Control_Update_Speed_Estimate(measured_position_mm, now_ms);

	// 获取位置外环传来的目标速度
	target_speed_mm_s = ball_control_target_speed_mm_s;

	/*
	 * 死区检测：如果目标速度和实际速度都小于阈值（3 mm/s），
	 * 认为小球已经静止，不需要控制。此时清空 PID 防止积分累积。
	 *
	 * 为什么要两个条件都满足？
	 *   如果只有目标速度小但实际速度大 → 小球在滑行，需要刹车。
	 *   如果只有实际速度小但目标速度大 → 需要加速。
	 *   只有两者都小，才说明"不需要做任何事"。
	 */
	if ((fabsf(target_speed_mm_s) <= BALL_SPEED_DEADBAND_MM_S) &&
		(fabsf(measured_speed_mm_s) <= BALL_SPEED_DEADBAND_MM_S)) {
		Ball_Control_Reset_PID(&ball_speed_pid);
		relative_motor_pulse = 0.0f;	// 电机保持水平位置
	}
	else {
		/*
		 * PID 计算：
		 *   setpoint = target_speed_mm_s（目标速度）
		 *   measurement = measured_speed_mm_s（实际速度）
		 *   dt = 0.01s
		 *   输出 = Kp×error + Ki×integral + Kd×differential
		 *   输出单位 = 脉冲数（相对水平位置）
		 */
		relative_motor_pulse = PID_Compute(&ball_speed_pid,
			target_speed_mm_s,
			measured_speed_mm_s,
			dt);
	}

	/*
	 * 乘以极性：如果机械安装方向反了，改 BALL_CONTROL_POLARITY 即可。
	 * 极性要么是 +1.0f，要么是 -1.0f。
	 */
	relative_motor_pulse *= BALL_CONTROL_POLARITY;

	// 限幅到安全范围：PID 输出最多使用 ±250 脉冲（比硬件限位 ±280 小 30）
	relative_motor_pulse = Ball_Control_Clamp(relative_motor_pulse,
		(float)BALL_MOTOR_MIN_RELATIVE_PULSE,
		(float)BALL_MOTOR_MAX_RELATIVE_PULSE);

	// 转换为绝对位置：绝对位置 = 水平位置 + 相对偏移
	target_motor_pulse = (float)BALL_MOTOR_LEVEL_PULSE + relative_motor_pulse;

	/*
	 * 斜率限制（Approach）：限制电机目标位置每周期最多变化 12 脉冲。
	 * 这有两个作用：
	 *   1. 防止 PID 输出突变导致机械冲击。
	 *   2. 给驱动器内部梯形规划足够的时间平滑执行。
	 *
	 * 注意：Approach 是针对"目标位置"的，不是针对"电机实际位置"。
	 *       电机实际位置由驱动器内部的梯形规划控制。
	 */
	next_motor_pulse = Ball_Control_Approach(ball_control_motor_pulse,
		target_motor_pulse,
		maximum_motor_step);

	// 再次限幅到绝对安全范围（-280 到 +280）
	next_motor_pulse = Ball_Control_Clamp(next_motor_pulse,
		minimum_absolute_pulse,
		maximum_absolute_pulse);

	// 更新电机位置状态并发送命令
	ball_control_motor_pulse = next_motor_pulse;
	Ball_Control_Send_Motor_Command(Ball_Control_Round_To_Int32(next_motor_pulse),
		now_ms);
}

/*
 * ============================================================================
 * 位置外环控制任务（核心）
 * ============================================================================
 */

/**
 * @brief   小球位置外环任务：目标位置/实际位置 → 小球目标速度。
 *
 * 目标：根据小球当前位置和目标位置的差距，决定小球应该以多快的速度移动。
 *
 * 数据流：
 *   输入：
 *     - ball_control_target_position_mm（目标位置，默认 0 = 零点）
 *     - ball_control_position_mm（实际位置，来自激光测距）
 *   输出：
 *     - ball_control_target_speed_mm_s（目标速度 mm/s）
 *
 * 中间步骤：
 *   1. 在临界区内读取位置、有效标志和使能标志的一致快照。
 *   2. 如果控制未启用或激光无效 → 清空 PID，目标速度归零。
 *   3. 如果位置误差 < 1.5mm（死区）→ 认为已到达目标，清空 PID。
 *   4. 否则用 PID 计算目标速度。
 *   5. 用 Approach 限制目标速度的变化率（防止突变）。
 *
 * 为什么在中断中调用？
 *   这个函数在 TIM3 中断（10Hz）中调用，dt 严格等于 0.1s。
 *   10Hz 与外环频率匹配，因为激光数据也约 10Hz 更新一次。
 *
 * 注意：这个函数只输出目标速度，不直接控制电机。
 *       电机控制由速度内环负责。
 *
 * @note   在 BALL_CONTROL_POSITION_LOOP_HZ（10Hz）对应的 TIM3 中断中调用。
 * @param  无
 * @retval 无
 */
void User_Task_Position_Control(void)
{
	const float dt = 1.0f / BALL_CONTROL_POSITION_LOOP_HZ;				// 0.1s
	const float maximum_target_speed_step = BALL_TARGET_SPEED_SLEW_MM_S2 * dt;	// 每周期最大速度变化 = 800 × 0.1 = 80 mm/s
	float measured_position_mm;
	float target_position_mm;
	float requested_speed_mm_s;
	uint32_t saved_primask;
	uint8_t control_enabled;
	uint8_t laser_valid;

	/*
	 * 临界区：读取位置、有效标志和使能标志的一致快照。
	 *
	 * 这些值由速度内环（TIM2 中断，100Hz）发布。速度内环优先级更高，
	 * 可能在读取过程中打断。关中断确保读取到的是"同一时刻"的值。
	 */
	saved_primask = __get_PRIMASK();
	__disable_irq();
	control_enabled = ball_control_enabled;
	laser_valid = ball_control_laser_valid;
	measured_position_mm = ball_control_position_mm;
	target_position_mm = ball_control_target_position_mm;
	__DMB();
	if (saved_primask == 0U)
		__enable_irq();

	// 控制未启用或激光无效 → 清空 PID，目标速度归零
	if ((!control_enabled) || (!laser_valid)) {
		Ball_Control_Reset_PID(&ball_position_pid);
		ball_control_target_speed_mm_s = 0.0f;
		return;
	}

	/*
	 * 死区检测：如果位置误差 < 1.5mm，认为小球已经到达目标位置。
	 * 清空 PID 防止积分累积导致微小的来回漂移。
	 *
	 * 注意：死区只检查位置误差，不检查速度。
	 *       如果小球正在快速穿过死区，它不会停在这里。
	 *       只有小球确实在死区内且速度也足够低时，速度内环的死区才会触发。
	 */
	if (fabsf(target_position_mm - measured_position_mm) <=
		BALL_POSITION_DEADBAND_MM) {
		Ball_Control_Reset_PID(&ball_position_pid);
		requested_speed_mm_s = 0.0f;
	}
	else {
		/*
		 * PID 计算：
		 *   setpoint = target_position_mm（目标位置，通常是 0）
		 *   measurement = measured_position_mm（实际位置）
		 *   dt = 0.1s
		 *   输出 = Kp×error + Ki×integral + Kd×differential
		 *   输出单位 = mm/s（小球目标速度）
		 *
		 * 注意：误差过零时 PID_Compute 会自动清零积分，防止过冲。
		 */
		requested_speed_mm_s = PID_Compute(&ball_position_pid,
			target_position_mm,
			measured_position_mm,
			dt);
	}

	/*
	 * 斜率限制（Approach）：限制目标速度每周期最多变化 80 mm/s。
	 *
	 * 为什么限制目标速度变化率？
	 *   如果 PID 突然输出从 +200 跳到 -200 mm/s，速度内环会收到一个
	 *   巨大的阶跃输入，导致电机瞬间大幅度倾斜，小球会剧烈振荡。
	 *   限制变化率让目标速度平滑过渡，速度内环也更容易跟踪。
	 */
	ball_control_target_speed_mm_s = Ball_Control_Approach(
		ball_control_target_speed_mm_s,
		requested_speed_mm_s,
		maximum_target_speed_step);
}



//##################################################################################################
//**********************关于OLED模块任务*************************************************************
//##################################################################################################

/**
 * @brief   OLED 初始化。
 *
 * 目标：初始化 SSD1306 OLED 屏幕并清空显存。
 *
 * @param   无
 * @retval  无
 */
void User_Task_OLED_Init(void){
	// 初始化 OLED 屏幕（I2C 通信配置）
	ssd1306_Init();
	// 清空屏幕显存（全部填充满黑色）
	ssd1306_Fill(Black);
}


/**
 * @brief   OLED 显示更新。
 *
 * 目标：在主循环中刷新 OLED 屏幕，显示 PID 参数和传感器数据。
 *
 * 显示内容：
 *   第一行：速度内环 Kp 和位置外环 Kp（格式："Kp:1.20 H 0.60"）
 *   第二行：速度内环 Ki 和位置外环 Ki（格式："Ki:0.10 H 0.70"）
 *   第三行：速度内环 Kd 和位置外环 Kd（格式："Kd:0.30 H 0.00"）
 *   第四行：激光位置（mm）和电机脉冲数（格式："L:12.34 H 56"）
 *            其中 "H" 表示"和"（中文拼音 he 的首字母），不是 PID 的 H 参数。
 *
 * 实现细节：
 *   - 使用 ssd1306_Fill(Black) 清空屏幕缓冲区。
 *   - 使用 ssd1306_SetCursor 定位每行起始位置。
 *   - 使用 sprintf 格式化数字为字符串。
 *   - 使用 ssd1306_WriteString 写入字符串到缓冲区。
 *   - 最后调用 ssd1306_UpdateScreen_IT 通过 I2C 中断刷新屏幕。
 *     如果上一帧 I2C 传输尚未完成，UpdateScreen_IT 会直接返回，不会阻塞。
 *
 * 注意：sprintf 比较耗时，但 OLED 更新在主循环中 30ms 一次，不影响控制。
 *
 * @param   无
 * @retval  无
 */
void User_Task_OLED_Update(void){
	//**************屏幕测试的屏幕显示（已注释，保留供参考）***********
	// static int my_count = 0;       // 自加的数字
	// static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	// my_count++;					   // 自加
	// sprintf(str_buff, "Count: %d", my_count);			//组合打印内容,这里是把数字变成字符
	// ssd1306_Fill(Black);									//清空屏幕缓冲区
	// ssd1306_SetCursor(10, 20);							//锁定打印位置
	// ssd1306_WriteString(str_buff, Font_11x18, White);	//确定打印字符,大小,颜色
	// ssd1306_UpdateScreen_IT();							//启动 I2C 中断刷新，函数立即返回，不阻塞主循环


	//**************关于方案2的屏幕显示***********
	// 显示 PID 参数：内环(速度环) 和 外环(位置环) 的 Kp, Ki, Kd
	static char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	ssd1306_Fill(Black);					// 清空屏幕缓冲区

	// 第一行：Kp（速度内环 Kp, 位置外环 Kp）
	ssd1306_SetCursor(5, 5);
	sprintf(str_buff, "Kp:%.2f H %.2f", ball_speed_pid.Kp, ball_position_pid.Kp);
	ssd1306_WriteString(str_buff, Font_7x10, White);

	// 第二行：Ki（速度内环 Ki, 位置外环 Ki）
	ssd1306_SetCursor(5, 20);
	sprintf(str_buff, "Ki:%.2f H %.2f", ball_speed_pid.Ki, ball_position_pid.Ki);
	ssd1306_WriteString(str_buff, Font_7x10, White);

	// 第三行：Kd（速度内环 Kd, 位置外环 Kd）
	ssd1306_SetCursor(5, 35);
	sprintf(str_buff, "Kd:%.2f H %.2f", ball_speed_pid.Kd, ball_position_pid.Kd);
	ssd1306_WriteString(str_buff, Font_7x10, White);

	// 第四行：激光位置（mm）和电机脉冲数
	static float laser_distance_mm = 0.0f;
	User_Task_Laser_UART_Get(&laser_distance_mm);	// 获取激光数据

	ssd1306_SetCursor(5, 50);
	sprintf(str_buff, "L:%.2f H %.0f", laser_distance_mm, next_motor_pulse);
	ssd1306_WriteString(str_buff, Font_7x10, White);

	/* 启动 I2C 中断刷新；若上一帧尚未完成，本次调用会直接返回。 */
	(void)ssd1306_UpdateScreen_IT();
}




//##################################################################################################
//********关于串口模块任务***************************************************************************
//##################################################################################################
/**
 * @brief   串口输出任务。
 *
 * 目标：向 VOFA+ 上位机发送调试数据，用于实时波形显示。
 *
 * 数据格式："POS: 位置mm, 电机脉冲数\n"
 * VOFA+ 配置为"FireWater"协议即可自动解析并绘制波形。
 *
 * 实现细节：
 *   - 读取激光位置和电机脉冲数。
 *   - 使用 printf_dma 通过 UART1 DMA 发送，不阻塞 CPU。
 *     注意：UART1 同时用于电机命令和调试打印，但 DMA 模式不会冲突。
 *
 * 已注释的代码是之前用于陀螺仪数据打印和 VOFA+ 测试的，保留供参考。
 *
 * @param   无
 * @retval  无
 */
void User_Task_UART_Update(void){
	//关于陀螺仪数据打印（已注释，保留供参考）
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


	//vofa+波形图测试代码（已注释，保留供参考）
//	static uint8_t status_print=0;
//	if(status_print==0){
//		printf_dma("d: %f, %f\n", 100.0f,200.0f);
//		status_print=1;
//	}else{
//		printf_dma("d: %f, %f\n", -100.0f,-200.0f);
//		status_print=0;
//	}

	// VOFA+ 实时调试数据：激光位置 和 电机脉冲
	static float laser_distance_mm = 0.0f;
	User_Task_Laser_UART_Get(&laser_distance_mm);	// 获取激光数据

	// 复制 volatile 变量到局部变量，防止打印过程中被中断修改
	float Tem_next_motor_pulse = next_motor_pulse;

	// 格式："POS: 位置mm, 电机脉冲数\n"
	// VOFA+ 用 FireWater 协议解析：逗号分隔的两列数据，自动绘制波形
	printf_dma("POS: %f,%f\n", laser_distance_mm, Tem_next_motor_pulse);
}