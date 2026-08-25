#ifndef __User_Task_H__
#define __User_Task_H__

//说明,这是我封装的任务模块,里面包含了很多任务函数,目的是为了解决main函数太长,难以管理的问题

///#########头文件引用区域########
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h" // 包含字库
#include <stdio.h>         // 为了使用 sprintf 函数把数字转成字符串
#include "Printf_DMA.H"
#include "mpu6050.h"
#include "Emm_V5.h"
#include "BSP_Emm_V5.H"

///#########函数声明区域########
void User_Task_Init(void);  //任务初始化函数,在main函数中调用,用于初始化所有任务,包括硬件初始化和软件初始化



void User_Task_key(void);   //按键任务函数,在main函数的while循环中调用,用于处理按键事件


#endif
