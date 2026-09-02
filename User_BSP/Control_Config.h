#ifndef __CONTROL_CONFIG_H__
#define __CONTROL_CONFIG_H__

//####头文件引用区#####
#include "main.h"

/* 采样率必须与TIM1和TIM2匹配。*/
#define CONTROL_IMU_SAMPLE_HZ             300.0f		//Time1的频率,陀螺仪读取频率
#define CONTROL_LOOP_HZ                   100.0f		//Time2的频率
#define CONTROL_AUTOMATIC_ENABLE          (1)				//自动控制使能位

/* 电机驱动器限制。ACC=0表示直接启动，此处不应使用 */
#define CONTROL_MOTOR_SPEED_RPM           (400)			//电机速度
#define CONTROL_MOTOR_ACCEL_PARAM         (180)			//电机加速度
#define CONTROL_MANUAL_TEST_PULSE         (250)			//手动控制极点(调参不需要改)

/* 加速度计校准。在重新校准时:修正后的Ay应为0g. */
#define CONTROL_ACCEL_Y_BIAS_G            (-0.001532f)	//y轴恒定偏移加减校准
#define CONTROL_ACCEL_Z_BIAS_G            (0.0f)		//z轴恒定偏移加减校准
#define CONTROL_ACCEL_TO_ROD_SIGN         (-1.0f)		//自动控制方向
#define CONTROL_ACCEL_INPUT_LIMIT_G       (0.30f)		//最大加速度输入限制


//#########加速度补偿(方案1已弃用)专属参数######

/* 低通滤波频率。提高以减少延迟，降低以减少抖动。 */
#define CONTROL_ACCEL_LPF_POLE_HZ         (26.0f)		//低通滤波频率

/* 绝对电机零点和安全联动行程，以电机脉冲为单位。 */
#define CONTROL_LEVEL_PULSE               (0)				//电机零点
#define CONTROL_MIN_RELATIVE_PULSE        (-280)		//下限位
#define CONTROL_MAX_RELATIVE_PULSE        (280)			//上限位

/* 软件S曲线限制，以电机脉冲单位表示。 */
#define CONTROL_PROFILE_NATURAL_HZ        (3.5f)
#define CONTROL_PROFILE_DAMPING           (1.0f)
#define CONTROL_MAX_PULSE_SPEED           (2500.0f)   /* pulse/s */
#define CONTROL_MAX_PULSE_ACCEL           (30000.0f)  /* pulse/s^2 */
#define CONTROL_MAX_PULSE_JERK            (1000000.0f) /* pulse/s^3 */

/* 不要重新发送微小的位置变化。电机保持上次的目标位置。 */
#define CONTROL_SEND_HYSTERESIS_PULSE     (3)
#define CONTROL_SETTLE_POSITION_PULSE     (0.35f)
#define CONTROL_SETTLE_SPEED_PULSE_S      (5.0f)
#define CONTROL_SETTLE_ACCEL_PULSE_S2     (100.0f)


//################################################
//******速度环+位置环控制方案(方案2)专属参数******
//################################################

/*
 * 串级控制结构：
 *   位置外环：小球位置(mm) -> 小球目标速度(mm/s)
 *   速度内环：小球速度(mm/s) -> 电机绝对位置脉冲
 * 两个任务的实际定时器频率必须与下面的频率完全一致。
 */
#define BALL_CONTROL_SPEED_LOOP_HZ              (100.0f)  // 建议速度内环 100 Hz。
#define BALL_CONTROL_POSITION_LOOP_HZ           (10.0f)   // 建议位置外环 10 Hz。

/* 激光测距与速度估算参数。 */
#define BALL_CONTROL_LASER_ORIGIN_MM             (190.0f)  // 传感器原始读数为该值时，小球位于人为规定的 0 点。
#define BALL_CONTROL_TARGET_POSITION_MM          (0.0f)    // 小球目标位置，单位 mm。
#define BALL_CONTROL_NEW_SAMPLE_EPSILON_MM       (0.05f)   // 位置变化超过该值才认为激光产生了一个新样本。
#define BALL_CONTROL_SPEED_FILTER_HZ             (3.0f)    // 差分速度的一阶低通截止频率。
#define BALL_CONTROL_SPEED_ESTIMATE_LIMIT_MM_S   (1000.0f) // 差分速度尖峰限幅，单位 mm/s。
#define BALL_CONTROL_SPEED_ZERO_TIMEOUT_MS       (250U)    // 位置长期不变时将估算速度归零。

/*
 * 控制器极性只能填写 +1.0f 或 -1.0f。
 * 把球放在 0 点右侧；若控制后球继续向右运动，就把该宏正负号反过来。
 */
#define BALL_CONTROL_POLARITY                    (1.0f)

/* 位置外环 PID：输出为小球目标速度(mm/s)，建议先只调 Kp。 */
#define BALL_POSITION_KP                         (0.7f)
#define BALL_POSITION_KI                         (0.8f)
#define BALL_POSITION_KD                         (0.0f)
#define BALL_POSITION_INTEGRAL_LIMIT_MM_SEC      (150.0f)  // 位置误差积分限幅，单位 mm*s。
#define BALL_POSITION_MAX_TARGET_SPEED_MM_S      (280.0f)
#define BALL_POSITION_DEADBAND_MM                 (1.5f)
#define BALL_TARGET_SPEED_SLEW_MM_S2              (800.0f)  // 外环目标速度最大变化率。

/* 速度内环 PID：输出为相对水平位置的电机脉冲数。 */
#define BALL_SPEED_KP                            (1.65f)
#define BALL_SPEED_KI                            (0.1f)
#define BALL_SPEED_KD                            (0.3f)
#define BALL_SPEED_INTEGRAL_LIMIT_MM             (300.0f)
#define BALL_SPEED_DEADBAND_MM_S                  (3.0f)

/* 电机绝对位置、安全行程和命令整形参数。 */
#define BALL_MOTOR_LEVEL_PULSE                    (0)       // 管道水平时的电机绝对位置。
#define BALL_MOTOR_MIN_RELATIVE_PULSE             (-280)    // 相对水平位置的最小安全行程。
#define BALL_MOTOR_MAX_RELATIVE_PULSE             (280)     // 相对水平位置的最大安全行程。
#define BALL_MOTOR_MAX_CONTROL_PULSE              (250.0f)  // PID 可使用的最大相对脉冲。
#define BALL_MOTOR_PULSE_SLEW_PER_SECOND          (1200.0f) // 电机目标位置每秒最大变化脉冲数。
#define BALL_MOTOR_SEND_HYSTERESIS_PULSE          (1)       // 命令变化不足该值时不重复发送。
#define BALL_MOTOR_MIN_SEND_INTERVAL_MS           (5U)      // 避免 UART DMA 长期处于忙状态。
#define BALL_CONTROL_ENABLE_DEFAULT               (1U)


#endif
