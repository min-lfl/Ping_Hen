
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

/*
 * 浮点数四舍五入转整数。
 * 用于将 PID 计算出的浮点脉冲值转为电机命令需要的整数脉冲。
 */
static int32_t Ball_Control_Round_To_Int32(float value)
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