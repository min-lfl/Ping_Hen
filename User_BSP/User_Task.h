/**
 * @file    User_Task.h
 * @brief   用户任务模块的头文件，将所有业务逻辑封装为独立的"任务"函数。
 *
 * 设计目标：
 *   把 main.c 中散落的硬件初始化、传感器读取、控制逻辑、显示更新等代码，
 *   按功能拆分为独立的"任务"函数，每个任务有明确的 Init / Update / Get 接口。
 *   这样 main.c 只需按固定顺序调用这些函数，代码结构清晰，便于维护。
 *
 * 任务列表：
 *   - 参数更新：User_Task_Param_Update()           → while 循环中调用
 *   - 陀螺仪：  User_Task_MPU6050_Init/Update/Get   → Init 在启动时，Update 在 while 循环
 *   - 激光测距：User_Task_Laser_UART_Init/Get        → Init 在启动时，Get 在中断中调用
 *   - 按键：    User_Task_key()                     → while 循环中调用
 *   - 控制(方案1)：User_Task_Control()              → while 循环中调用（已弃用）
 *   - 速度内环：User_Task_Speed_Control()           → TIM2 中断中调用
 *   - 位置外环：User_Task_Position_Control()        → TIM3 中断中调用
 *   - OLED：    User_Task_OLED_Init/Update          → Init 在启动时，Update 在 while 循环
 *   - 串口输出：User_Task_UART_Update()             → while 循环中调用
 */

#ifndef __User_Task_H__
#define __User_Task_H__

///#########头文件引用区域########
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"   // OLED 字库
#include <stdio.h>           // sprintf 把数字转字符串打印到 OLED
#include "Printf_DMA.H"      // 串口 DMA 打印函数
#include "mpu6050.h"         // 陀螺仪模块
#include "Emm_V5.h"          // 张大头电机底层驱动（协议帧构造）
#include "BSP_Emm_V5.H"      // 电机中间层封装（简化调用接口）
#include "PID_Cnotrol.H"     // PID 控制算法
#include "Laser_uart.h"      // 激光测距 DMA 接收

//##########外部变量引用区########

/* ---- 方案 1（已弃用）的调试变量 ---- */
extern volatile float control_debug_ay_filtered_g;
extern volatile float control_debug_az_filtered_g;
extern volatile float control_debug_target_angle_deg;
extern volatile float control_debug_target_pulse;
extern volatile float control_debug_command_pulse;
extern volatile float control_debug_profile_speed;
extern volatile uint8_t control_automatic_enabled;

/* ---- 方案 2（当前串级控制）的运行时状态变量 ----
 *
 * 这些变量由定时器中断中的控制任务更新，由主循环中的 OLED/串口任务读取。
 * 它们不是线程安全的——中断可能在任意时刻修改它们——但 32 位 float 在
 * Cortex-M3 上是单指令读写的，所以即使被中断打断也不会读到"半个"值。
 *
 * 使用方式：
 *   - 运行时修改 ball_control_enabled 来开关控制。
 *   - 运行时修改 ball_control_target_position_mm 来改变目标位置。
 *   - 其余变量只读，用于观察控制器实时状态。
 */
extern volatile uint8_t ball_control_enabled;              // 1：运行串级控制；0：清除控制状态并回水平位。
extern volatile uint8_t ball_control_laser_valid;          // 1：激光后台缓存中已有有效数据。
extern volatile float ball_control_target_position_mm;     // 小球目标位置，单位 mm，默认值见 Control_Config.h。
extern volatile float ball_control_position_mm;            // 小球当前位置（激光测距值 - 原点偏移），单位 mm。
extern volatile float ball_control_speed_mm_s;             // 小球滤波后速度，单位 mm/s。由速度估算器输出。
extern volatile float ball_control_target_speed_mm_s;      // 位置外环给出的目标速度，单位 mm/s。速度内环跟踪这个值。
extern volatile float ball_control_motor_pulse;            // 速度内环输出的电机绝对位置，单位 pulse。发送给驱动器。


///#########函数声明区域########

/*
 * 总体初始化
 * 调用顺序：陀螺仪 → OLED → 等待 500ms → 电机 → 等待 5ms → 方案 1 控制 → 激光
 * 500ms 的等待是为了让电路和传感器稳定。
 */
void User_Task_Init(void);

//***关于参数更新函数任务***
/*
 * 在主循环中调用。每次调用都会通过 UART1 将速度和加速度参数发送给电机驱动器。
 * 如果需要在运行时修改参数，修改 UPdate_Speed_RPM 和 UPdate_Accel_Param 即可。
 */
void User_Task_Param_Update(void);

//***关于陀螺仪模块任务***
void User_Task_MPU6050_Init(void);               // 上电时调用一次：初始化 I2C，校准陀螺仪零偏。
void User_Task_MPU6050_Update(void);             // 主循环中调用：读取 MPU6050 数据，更新方案 1 的加速度输入。
void User_Task_MPU6050_Get(MPU6050_t* data);     // 获取陀螺仪最新数据副本。注意：需要在 Update() 之后调用。

//***关于激光串口任务***
/*
 * 激光传感器使用 UART DMA + IDLE 中断接收，数据在后台自动刷新。
 * Init 只需调用一次来启动 DMA 接收。
 * Get 只是读取后台缓存的最新值，不会访问串口。
 */
void User_Task_Laser_UART_Init(void);                // 启动激光 DMA 后台接收，只需调用一次。
void User_Task_Laser_UART_Get(float *distance_mm);   // 读取激光缓存值，输出的是相对于零点的位置（原点右侧为正）。

//***关于按键模块任务***
/*
 * 在主循环中调用，扫描三个按键：
 *   PB13：电机正转到 +250 脉冲位置
 *   PB14：电机回零 + 切换自动/手动控制
 *   PB15：电机反转到 -250 脉冲位置
 */
void User_Task_key(void);

//***关于加速度补偿控制模块任务(方案1已弃用)***
/*
 * 方案 1 的内部函数（static，不对外暴露）。
 * 原理：加速度计 → 杆倾角 → 校准表查表 → 电机脉冲 → S 曲线平滑 → 发送命令。
 * 弃用原因：加速度计无法区分"杆倾斜"和"小球加速"，导致控制发散。
 */
static void Control_Init(void);
static void Control_Input_Update(float ay_g, float az_g);
static float Control_Motion_Update(float target_pulse);
void User_Task_Control(void);         // 方案 1 控制任务，在主循环中调用（当前未使用）。


//***关于速度环+位置环控制模块任务(方案2，当前使用)***
/*
 * 串级控制的两个核心任务，分别在定时器中断中调用：
 *
 *   位置外环 (TIM3, 10Hz)：
 *     User_Task_Position_Control()
 *     输入：目标位置 mm、实际位置 mm
 *     输出：ball_control_target_speed_mm_s（小球目标速度 mm/s）
 *     作用：决定小球应该以多快速度往哪个方向移动。
 *
 *   速度内环 (TIM2, 100Hz)：
 *     User_Task_Speed_Control()
 *     输入：目标速度 mm/s、实际速度 mm/s（由位置差分 + 低通滤波估算）
 *     输出：电机的绝对位置脉冲数
 *     作用：让小球的实际速度跟踪目标速度，通过电机倾斜角度控制重力分力。
 *
 * 为什么在中断中调用？
 *   控制算法需要精确的、固定的时间间隔（dt）来计算 PID。
 *   如果放在主循环中，dt 会因为其他任务（OLED、按键、串口打印）的执行时间而变化，
 *   导致 PID 计算不准。放在定时器中断中，dt 严格等于 1/频率。
 */
void User_Task_Speed_Control(void);     // 速度内环，TIM2 中断中调用，100Hz。
void User_Task_Position_Control(void);  // 位置外环，TIM3 中断中调用，10Hz。


//***关于OLED模块任务***
void User_Task_OLED_Init(void);       // 初始化 OLED 屏幕，清空显存。
/*
 * 在主循环中调用，刷新 OLED 显示：
 *   第一行：速度内环 Kp 和位置外环 Kp
 *   第二行：速度内环 Ki 和位置外环 Ki
 *   第三行：速度内环 Kd 和位置外环 Kd
 *   第四行：激光位置（mm）和电机脉冲数
 */
void User_Task_OLED_Update(void);


//*** 关于串口任务***
/*
 * 在主循环中调用，当前用于向 VOFA+ 上位机发送调试数据：
 *   格式："POS: 位置mm, 电机脉冲数\n"
 *   VOFA+ 可以实时绘制波形，方便观察控制效果。
 */
void User_Task_UART_Update(void);

#endif