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
void User_Task_Param_Update(void);

//***关于陀螺仪模块任务***
void User_Task_MPU6050_Init(void);               // 上电时调用一次：初始化 I2C，校准陀螺仪零偏。
void User_Task_MPU6050_Update(void);             // 主循环中调用：读取 MPU6050 数据，更新方案 1 的加速度输入。
void User_Task_MPU6050_Get(MPU6050_t* data);     // 获取陀螺仪最新数据副本。注意：需要在 Update() 之后调用。

//***关于激光串口任务***
void User_Task_Laser_UART_Init(void);                // 启动激光 DMA 后台接收，只需调用一次。
void User_Task_Laser_UART_Get(float *distance_mm);   // 读取激光缓存值，输出的是相对于零点的位置（原点右侧为正）。

//***关于按键模块任务***
void User_Task_key(void);

//***关于加速度补偿控制模块任务(方案1已弃用)***
static void Control_Init(void);
static void Control_Input_Update(float ay_g, float az_g);
static float Control_Motion_Update(float target_pulse);
void User_Task_Control(void);         // 方案 1 控制任务，在主循环中调用（当前未使用）。


//***关于速度环+位置环控制模块任务(方案2，当前使用)***
void User_Task_Speed_Control(void);     // 速度内环，TIM2 中断中调用，100Hz。
void User_Task_Position_Control(void);  // 位置外环，TIM3 中断中调用，10Hz。


//***关于OLED模块任务***
void User_Task_OLED_Init(void);       // 初始化 OLED 屏幕，清空显存。
void User_Task_OLED_Update(void);


//*** 关于串口任务***
void User_Task_UART_Update(void);

#endif