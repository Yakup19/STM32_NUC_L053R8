/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern SPI_HandleTypeDef hspi2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE  1
#define FALSE 0
char buf[1024];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t serialBuf[100];
MPU9250_t MPU9250;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check if timer has triggered and update attitude
  if (htim == &htim6)
  {
    //HAL_ResumeTick();

    MPU_calcAttitude(&hspi2, &MPU9250);
    int16_t roll = roundf(10 * MPU9250.attitude.r);
    uint8_t rollDecimal = abs(roll % 10);
    int16_t pitch = roundf(10 * MPU9250.attitude.p);
    uint8_t pitchDecimal = abs(pitch % 10);
    int16_t yaw = roundf(10 * MPU9250.attitude.y);
    uint8_t yawDecimal = abs(yaw % 10);

    sprintf((char *)serialBuf, "%d.%d,%d.%d,%d.%d\r\n", roll/10, rollDecimal, pitch/10, pitchDecimal, yaw/10, yawDecimal);
    HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);



	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(((MPU9250.rawData.ax) * (150)) / (8000)));
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(((MPU9250.rawData.ay) * (150)) / (8000)));
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, abs(((MPU9250.rawData.az) * (150)) / (8000)));
	  sprintf (buf, "X Ekseni %.2f ", MPU9250.rawData.ax);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
//    		  if(MPU9250.rawData.ax<=2500)
//    			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
//    		  else if (MPU9250.rawData.ax<=5000)
//    			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
//    		  else if(MPU9250.rawData.ax<=10000)
//    			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2500);
//
//    	  else
//    		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);



    	  sprintf (buf, "Y Ekseni %.2f ", MPU9250.rawData.ay);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
//
//    	 		  if(MPU9250.rawData.ay<=2500)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
//    	 		  else if (MPU9250.rawData.ay<=5000)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
//    	 		  else if(MPU9250.rawData.ay<=10000)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2500);
//
//
//
//    	 	  else
//    	 		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

    	  sprintf (buf, "Z Ekseni %.2f\n", MPU9250.rawData.az);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
//    	 		  if(MPU9250.rawData.az<=2500)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
//    	 		  else if (MPU9250.rawData.az<=5000)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
//    	 		  else if(MPU9250.rawData.az<=10000)
//    	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2500);
//
//
//
//    	 	  else
//    	 		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

    	  sprintf (buf, "GX: %.2f ", MPU9250.gyroCal.x);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);

    	  sprintf (buf, "GY Ekseni %.2f ", MPU9250.gyroCal.y);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);

    	  sprintf (buf, "GZ Ekseni %.2f\n", MPU9250.gyroCal.z);
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);





    HAL_SuspendTick();
  }
}
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
	MPU9250.settings.gFullScaleRange = GFSR_500DPS;
	  MPU9250.settings.aFullScaleRange = AFSR_4G;
	  MPU9250.settings.CS_PIN = GPIO_PIN_7;
	  MPU9250.settings.CS_PORT = GPIOB;
	  MPU9250.attitude.tau = 0.98;
	  MPU9250.attitude.dt = 0.004;
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  // Check if IMU configured properly and block if it didn't
  if (MPU_begin(&hspi2, &MPU9250) != TRUE)
    {
      sprintf((char *)serialBuf, "ERROR!\r\n");
      HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
      while (1){}
    }

    // Calibrate the IMU
    sprintf((char *)serialBuf, "CALIBRATING...\r\n");
    HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
    MPU_calibrateGyro(&hspi2, &MPU9250, 1500);

    // Start timer and put processor into an efficient low power mode
    HAL_TIM_Base_Start_IT(&htim6);
    //HAL_PWR_EnableSleepOnExit();
    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // print the Acceleration and Gyro values on the LCD 20x4



	  HAL_Delay (250);  // wait for a while
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
