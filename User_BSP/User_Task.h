#ifndef __User_Task_H__
#define __User_Task_H__

//说明,这是我封装的任务模块,里面包含了很多任务函数,目的是为了解决main函数太长,难以管理的问题

///#########头文件引用区域########
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h" // 包含字库
#include <stdio.h>         // 为了使用 sprintf 函数把数字转成字符串
#include "Printf_DMA.H"		 //串口打印函数
#include "mpu6050.h"			 //陀螺仪模块
#include "Emm_V5.h"				 //电机底层驱动
#include "BSP_Emm_V5.H"		 //电机中间层
#include "PID_Cnotrol.H"	 //PID控制模块
#include "Laser_uart.h"

//##########外部变量引用区########
extern volatile float control_debug_ay_filtered_g;
extern volatile float control_debug_az_filtered_g;
extern volatile float control_debug_target_angle_deg;
extern volatile float control_debug_target_pulse;
extern volatile float control_debug_command_pulse;
extern volatile float control_debug_profile_speed;
extern volatile uint8_t control_automatic_enabled;

/* enable 和 target_position 可在运行时修改，其余变量用于观察控制器实时状态。 */
extern volatile uint8_t ball_control_enabled;              // 1：运行串级控制；0：清除控制状态并回水平位。
extern volatile uint8_t ball_control_laser_valid;          // 1：激光后台缓存中已有有效数据。
extern volatile float ball_control_target_position_mm;     // 小球目标位置，单位 mm，默认值见 Control_Config.h。
extern volatile float ball_control_position_mm;            // 小球当前位置，单位 mm。
extern volatile float ball_control_speed_mm_s;             // 小球滤波后速度，单位 mm/s。
extern volatile float ball_control_target_speed_mm_s;      // 位置外环给出的目标速度，单位 mm/s。
extern volatile float ball_control_motor_pulse;            // 速度内环给出的电机绝对位置，单位 pulse。


///#########函数声明区域########
void User_Task_Init(void);  //任务初始化函数,在main函数中调用,用于初始化所有任务,包括硬件初始化和软件初始化

//***关于参数更新函数任务***
void User_Task_Param_Update(void);    //参数更新任务函数,在main函数的while循环中调用,用于更新系统参数

//***关于陀螺仪模块任务***
void User_Task_MPU6050_Init(void);               //陀螺仪任务初始化函数
void User_Task_MPU6050_Update(void);             //陀螺仪任务函数,在main函数的while循环中调用,用于触发采样,处理陀螺仪数据采集和姿态解算
void User_Task_MPU6050_Get(MPU6050_t* data);     //陀螺仪任务函数,用于获取陀螺仪数据,将mpu6050_date数据传递给外部使用,注意:该函数需要在User_Task_MPU6050_Update()之后调用,否则获取的数据可能不准确

//***关于激光串口任务***
void User_Task_Laser_UART_Init(void);                //激光串口任务初始化函数,由于用的是中断加DMA接收,所以不需要在while循环中调用,只需要在main函数中调用一次即可,用于初始化激光串口相关的硬件和软件资源
void User_Task_Laser_UART_Get(float *distance_mm);   //激光串口任务函数,用于获取激光数据,读数直接就是关于原点距离,单位毫米

//*** 关于串口任务***
void User_Task_UART_Update(void);     //串口任务函数,在main函数的while循环中调用,用于更新串口数据传输

//***关于按键模块任务***
void User_Task_key(void);             //按键任务函数,在main函数的while循环中调用,用于处理按键事件

//***关于加速度补偿控制模块任务(方案1已弃用)***
static void Control_Init(void);
static void Control_Input_Update(float ay_g, float az_g);
static float Control_Motion_Update(float target_pulse);

void User_Task_Control(void);         //加速度补偿控制任务函数,在main函数的while循环中调用,用于处理控制逻辑,包括PID控制和电机控制


//***关于速度环+位置环控制模块任务***
void User_Task_Speed_Control(void);     // 在 BALL_CONTROL_SPEED_LOOP_HZ 对应的固定频率定时中断中调用。
void User_Task_Position_Control(void);  // 在 BALL_CONTROL_POSITION_LOOP_HZ 对应的固定频率定时中断中调用。


//***关于OLED模块任务***
void User_Task_OLED_Init(void);       //OLED任务初始化函数
void User_Task_OLED_Update(void);     //OLED任务函数,在main函数的while循环中调用,用于更新OLED屏幕显示

#endif
