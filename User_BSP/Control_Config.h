#ifndef __CONTROL_CONFIG_H__
#define __CONTROL_CONFIG_H__

//####头文件引用区#####
#include "main.h"


/* 采样率必须与TIM1和TIM2匹配。*/
#define CONTROL_IMU_SAMPLE_HZ             300.0f		//Time1的频率,陀螺仪读取频率
#define CONTROL_LOOP_HZ                   100.0f		//Time2的频率
#define CONTROL_AUTOMATIC_ENABLE          (1)				//自动控制使能位

/* 电机驱动器限制。ACC=0表示直接启动，此处不应使用 */
#define CONTROL_MOTOR_SPEED_RPM           (500)			//电机速度
#define CONTROL_MOTOR_ACCEL_PARAM         (255)			//电机加速度
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
#define CONTROL_MIN_RELATIVE_PULSE        (-250)		//下限位
#define CONTROL_MAX_RELATIVE_PULSE        (250)			//上限位

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

//#########速度环位置环控制方案(方案1已弃用)专属参数######

#endif
