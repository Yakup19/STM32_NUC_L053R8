/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "string.h"
#include "stdio.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float Ax, Ay, Az, Gx, Gy, Gz;
char buf[1024];
uint32_t mappedpwm;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
int abs(int x) {
    if (x < 0) {
        return -x;
    } else {
        return x;
    }
}
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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  HAL_Delay (1000);  // wait for 1 sec
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // read the Accelerometer and Gyro values

	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();


	  // print the Acceleration and Gyro values on the LCD 20x4

	  sprintf (buf, "X Ekseni %.2f ", Ax);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
	  if(Ax>=1||Ax>=-1){
		  if(abs(Ax)<=0.25)
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
		  else if (abs(Ax)<=0.5)
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
		  else if(abs(Ax)<=1)
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);


	  }
	  else
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);



	  sprintf (buf, "Y Ekseni %.2f ", Ay);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
	  if(Ay>=1||Ay>=-1){
	 		  if(abs(Ay)<=0.25)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
	 		  else if (abs(Ay)<=0.5)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
	 		  else if(abs(Ay)<=1)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);


	 	  }
	 	  else
	 		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	  sprintf (buf, "Z Ekseni %.2f\n", Az);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);
	  if(Az>=1||Az>=-1){
	 		  if(abs(Az)<=0.25)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
	 		  else if (abs(Az)<=0.5)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
	 		  else if(abs(Az)<=1)
	 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);


	 	  }
	 	  else
	 		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

	  sprintf (buf, "GX: %.2f ", Gx);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);

	  sprintf (buf, "GY Ekseni %.2f ", Gy);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);

	  sprintf (buf, "GZ Ekseni %.2f\n", Gz);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 250);


	  HAL_Delay (250);  // wait for a while
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
