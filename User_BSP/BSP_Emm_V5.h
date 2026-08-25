#ifndef __BSP_Emm_V5_H__
#define __BSP_Emm_V5_H__

//模块说明:这是个张大头步进电机驱动的中间层,因为他封装的函数太难用了,所以我这里就行一次二次封装

//########头文件引用区#####
#include "main.h"
#include "Emm_V5.h"

//########宏定义区#####
#define EMM_V5_ADDR 1 //电机地址,这个项目只需要一个电
#define EMM_V5_SPEED 1000 //电机速度
#define EMM_V5_ACC 0 //电机加速度
#define EMM_V5_SNF false //电机snF


//########函数声明区#####

//###速度模式###

//###位置模块###
void BSP_Emm_V5_Pos_Control(int16_t pulse);     //绝对位置模式控制,输入int16位数据

#endif
