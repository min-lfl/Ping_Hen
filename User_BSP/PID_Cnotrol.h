#ifndef __PID_Cnotrol_H__
#define __PID_Cnotrol_H__

//***头文件引用区***
#include "main.h"


//***结构体区***
typedef struct {
    // 调参参数 (Kp, Ki, Kd)
    float Kp;
    float Ki;
    float Kd;

    // 过程状态
    float error;            // 当前误差
    float last_error;       // 上次误差
    float integral;         // 积分（误差累积）
    float integral_max;     // 积分限幅
		float differential;			// 微分
		
	
    float output;           // 最终计算出的输出值
		float output_max;				// 输出限幅
} PID_t;

//***用户函数区***
float PID_Compute(PID_t *pid, float target, float measure, float dt);

#endif

////******使用案例*****
////默认PID参数(只有带*号的才需要手动修改)
//volatile PID_t pidRoll  = {
//		10.0f, 	//*KP
//		0.0f, 	//*KI
//		0.0f, 	//*KD
//		0, 			//当前误差
//		0, 			//上次误差
//		0, 			//积分(误差累计)
//		500, 		//*积分限幅
//		0,			//微分
//	
//		0,			//最终输出值
//		1000		//*输出限幅
//};

////这个函数要放定时中断里0.01秒执行一次
//out = PID_Compute(&pidRoll,目标量,当前量,0.01);
////记得把out输出给控制函数完成闭环控制

