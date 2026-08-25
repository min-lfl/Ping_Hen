#include "User_Task.H"




/**
	* @brief	按键任务函数,在main函数的while循环中调用,用于处理按键事件
	* @note		该函数会扫描三个按键,分别对应GPIOB的13,14,15引脚,按下时会翻转对应的LED灯以及电机位置模式正反转的功能
	* @param	无
	* @retval	无
	*/
void User_Task_key(void){
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) 
	{
			/* 软件消抖延时（根据实际情况决定是否保留） */
			HAL_Delay(20); 
			
			/* 再次确认是否依然为低电平 */
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) 
			{
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        			BSP_Emm_V5_Pos_Control(2000);   //实际收到的01 FD 00 03 E8 00 00 00 07 D0 02 00 6B 
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
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
        			BSP_Emm_V5_Pos_Control(-2000);
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
					/* 翻转 PC13 的电平 */
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
					/* 等待引脚释放（变为高电平），防止按住时持续翻转 */
					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
					{
							// 可以加入微小的延时或空指令
					}
			}
	}
}


