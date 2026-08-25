/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h" // 包含字库
#include <stdio.h>         // 为了使用 sprintf 函数把数字转成字符串
#include "Printf_DMA.H"
#include "mpu6050.h"
#include "Emm_V5.h"
#include "BSP_Emm_V5.H"
#include "User_Task.H"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_SAMPLE_DELAY_MS 3U
#define MPU_OUTPUT_SAMPLE_COUNT 150U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */


  //初始化 OLED 屏幕
  ssd1306_Init();
  //清空屏幕显存 (全部填充满黑色)
  ssd1306_Fill(Black);
//	int my_count = 0;       // 自加的数字
//  char str_buff[32];      // 字符缓存区（准备32字节足够装一句话了）
	

	// 定义数据缓冲区
//	MPU6050_t mpu6050_date = {0};
	
	//初始化并且上电静止校准陀螺仪
//	uint8_t mpu_ready = 0;
//	uint16_t output_sample_count = 0;
//	if (MPU6050_Init(&hi2c2) == 0)
//	{
//		printf_dma("Keep MPU still: calibrating gyro...\r\n");
//		if (MPU6050_CalibrateGyro(&hi2c2, 300) == 0)
//			mpu_ready = 1;
//		else
//			printf_dma("MPU gyro calibration failed\r\n");
//	}
//	else
//	{
//		printf_dma("MPU WHO_AM_I or I2C failed\r\n");
//	}
	
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1); 											// 清除IDLE标志
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 							// 使能串UART1 IDLE中断
//  HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); // 开启DMA接收模式
  /* USER CODE BEGIN WHILE */

	/**********************************************************
	***	上电延时500毫秒等待闭环初始化完毕
	**********************************************************/	
		HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
//		// 每次循环读取一帧14字节数据，后续可在此处加入300Hz PID控制代码。
//		if (mpu_ready)
//		{
//			MPU6050_Read_All(&hi2c2, &mpu6050_date);
//			output_sample_count++;
//		}

//		// 每150次采样打印一次，约每0.5秒刷新调试信息，不参与姿态解算。
//		if (mpu_ready && output_sample_count >= 10)
//		{
//			output_sample_count = 0;
//			my_count++;
//			sprintf(str_buff, "X: %d", my_count);							//组合打印内容,这里是把数字变成字符
//			ssd1306_Fill(Black);															//清空屏幕缓冲区
//			ssd1306_SetCursor(10, 20);												//锁定打印位置
//			ssd1306_WriteString(str_buff, Font_11x18, White);	//确定打印字符,大小,颜色
//			ssd1306_UpdateScreen_DMA();												//刷新

//			printf_dma("x轴加速度%d x轴角速度%d \r\n", mpu6050_date.Accel_X_RAW, mpu6050_date.Gyro_X_RAW);
//			printf_dma("y轴加速度%d y轴角速度%d \r\n", mpu6050_date.Accel_Y_RAW, mpu6050_date.Gyro_Y_RAW);
//			printf_dma("z轴加速度%d z轴角速度%d \r\n", mpu6050_date.Accel_Z_RAW, mpu6050_date.Gyro_Z_RAW);
//			printf_dma(" \r\n");
//			printf_dma("转化后x轴加速度%f g 转化后x轴角速度%f 度/s \r\n", mpu6050_date.Ax, mpu6050_date.Gx);
//			printf_dma("转化后y轴加速度%f g 转化后y轴角速度%f 度/s \r\n", mpu6050_date.Ay, mpu6050_date.Gy);
//			printf_dma("转化后z轴加速度%f g 转化后z轴角速度%f 度/s \r\n", mpu6050_date.Az, mpu6050_date.Gz);
//			printf_dma(" \r\n");
//			printf_dma("当前x轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleX);
//			printf_dma("当前y轴姿态角%f 度 \r\n", mpu6050_date.KalmanAngleY);
//			printf_dma(" \r\n");
//		}

		
		// 当前用毫秒延时模拟控制周期；正式控制时可替换为1us定时器中断/任务唤醒。
		HAL_Delay(3);
		
		//执行按键任务,按键任务目前包括,扫描三个按键,发送翻转对应LED灯以及电机位置模式正反转的功能
		User_Task_key();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
