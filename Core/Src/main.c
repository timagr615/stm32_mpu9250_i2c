/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "micros.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU9250_t mpu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU9250SetDefault(&mpu);
  HAL_Delay(2000);
  if (!(setupMPU(&mpu, MPU9250_ADDRESS)==1)) {  // change to your own address
	  char badmpu[] = "Check MPU\n\r";
          while (1) {
        	  HAL_UART_Transmit(&huart1, badmpu, strlen((char *)badmpu), 0xFFFF);
        	  HAL_Delay(5000);
          }

  }
  //calibrate(&mpu);
  setMPUSettings(&mpu);
  //accel bias [g]:  11.1511   0.0000   0.0000  ␊
  //gyro bias [deg/s]:  0.0000   0.6698   0.0000  ␊
  //mag bias [mG]:  19.1250   143.4375   0.0000  ␊
  //mag scale []:  1.3571   0.9382   0.8351
  HAL_Delay(100);
  float a[3];
  float g[3];
  float m[3];
  float heading, pitch, roll;
  float q[4];

  uint32_t prev_ms = 0;
  uint8_t str[100];
  	        	  	  	uint8_t str2[100];
  	        	  	  	uint8_t str3[100];
  	        	  	  	uint8_t str4[100];
  	        	  	    uint8_t str5[100];
  	        	  	  	uint8_t str6[100];
  	        	  	  	uint8_t str7[100];
  	        	  	  	uint8_t str8[100];
  	        	  	  	uint8_t str9[100];
  uint8_t head[100];
  uint8_t rl[100];
  uint8_t ptc[100];
  char new_line[] = " \n\r   ";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (updateMPU(&mpu)==1) {

	          //prev_ms = HAL_GetTick();
	          if ((HAL_GetTick() - prev_ms) > 200) {
	        	  char ok[] = "ok";
	        	  		  	  HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);

	        	  	  a[0] = getAccX(&mpu);
	        	  	  a[1] = getAccY(&mpu);
	        	  	  a[2] = getAccZ(&mpu);
	        	  	  g[0] = getGyroX(&mpu);
	        	  	  g[1] = getGyroY(&mpu);
	        	  	  g[2] = getGyroZ(&mpu);
	        	  	  m[0] = getMagX(&mpu);
	        	  	  m[1] = getMagY(&mpu);
	        	  	  m[2] = getMagZ(&mpu);
	        	  	  heading = getYaw(&mpu);
	        	  	  roll = getRoll(&mpu);
	        	  	  pitch = getPitch(&mpu);
	        	  	  q[0] = getQuaternionX(&mpu);
	        	  	  q[1] = getQuaternionY(&mpu);
	        	  	  q[2] = getQuaternionZ(&mpu);
	        	  	  q[3] = getQuaternionW(&mpu);
	        	  	  print_float(head, heading);
	        	  	  print_float(rl, roll);
	        	  	  print_float(ptc, pitch);
	        	  	  HAL_UART_Transmit(&huart1, new_line, strlen((char *)new_line), 0xFFFF);
	        	  	  	/*sprintf(str, "ax = %d.%04d ",(uint32_t)a[0], (uint16_t)((a[0] - (uint32_t)a[0])*10000.));
	        	  	  	sprintf(str2, "ay = %d.%04d ",(uint32_t)a[1], (uint16_t)((a[1] - (uint32_t)a[1])*10000.));
	        	  	  	sprintf(str3, "az = %d.%04d    ",(uint32_t)a[2], (uint16_t)((a[2] - (uint32_t)a[2])*10000.));
	        	  	  	sprintf(str4, "gx = %d.%06d ",(uint32_t)g[0], (uint16_t)((g[0] - (uint32_t)g[0])*1000000.));
	        	  	  	sprintf(str5, "gy = %d.%06d ",(uint32_t)g[1], (uint16_t)((g[1] - (uint32_t)g[1])*1000000.));
	        	  	  	sprintf(str6, "gz = %d.%06d    ",(uint32_t)g[2], (uint16_t)((g[2] - (uint32_t)g[2])*1000000.));
	        	  	  	sprintf(str7, "mx = %d.%06d ",(uint32_t)m[0], (uint16_t)((m[0] - (uint32_t)m[0])*1000000.));
	        	  	    sprintf(str8, "my = %d.%06d ",(uint32_t)m[1], (uint16_t)((m[1] - (uint32_t)m[1])*1000000.));
	        	  	  	sprintf(str9, "mz = %d.%06d\n",(uint32_t)m[2], (uint16_t)((m[2] - (uint32_t)m[2])*1000000.));
	        	  	  HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str2, strlen((char *)str2), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str3, strlen((char *)str3), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str4, strlen((char *)str4), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str5, strlen((char *)str5), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str6, strlen((char *)str6), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str7, strlen((char *)str7), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str8, strlen((char *)str8), 0xFFFF);
	        	  	  	  	HAL_UART_Transmit(&huart1, str9, strlen((char *)str9), 0xFFFF);*/

	              prev_ms = HAL_GetTick();
	          }
	      }



	  //uint8_t whoamimpu = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	  //uint8_t wh[100];
	  uint8_t wha[100];
	  //sprintf(wh, "mpu = 0x%x ", whoamimpu);

	  //HAL_UART_Transmit(&huart1, wh, strlen((char *)wh), 0xFFFF);
	  /*HAL_UART_Transmit(&huart1, wha, strlen((char *)wha), 0xFFFF);

	  /*HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_RESET);
	  	  	 HAL_Delay(200);
	  	  	 HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_SET);
	  	  	 HAL_Delay(200);*/

	  	/*uint8_t str[400];
	  	uint8_t str2[200];
	  	uint8_t str3[100];
	  	uint8_t str4[100];
	    uint8_t str5[100];
	  	uint8_t str6[100];
	  	uint8_t str7[100];
	  	uint8_t str8[100];
	  	uint8_t str9[100];

	  	sprintf(str, "ax = %d.%04d ",(uint32_t)ax, (uint16_t)((ax - (uint32_t)ax)*10000.));
	  	sprintf(str2, "ay = %d.%04d ",(uint32_t)ay, (uint16_t)((ay - (uint32_t)ay)*10000.));
	  	sprintf(str3, "az = %d.%04d\n",(uint32_t)az, (uint16_t)((az - (uint32_t)az)*10000.));
	  	sprintf(str4, "gx = %d.%06d ",(uint32_t)gx, (uint16_t)((gx - (uint32_t)gx)*1000000.));
	  	sprintf(str5, "gy = %d.%06d ",(uint32_t)gy, (uint16_t)((gy - (uint32_t)gy)*1000000.));
	  	sprintf(str6, "gz = %d.%06d\n",(uint32_t)gz, (uint16_t)((gz - (uint32_t)gz)*1000000.));
	  	sprintf(str7, "mx = %d.%06d ",(uint32_t)mx, (uint16_t)((mx - (uint32_t)mx)*1000000.));
	    sprintf(str8, "my = %d.%06d ",(uint32_t)my, (uint16_t)((my - (uint32_t)my)*1000000.));
	  	sprintf(str9, "mz = %d.%06d\n",(uint32_t)mz, (uint16_t)((mz - (uint32_t)mz)*1000000.));

	  	HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str2, strlen((char *)str2), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str3, strlen((char *)str3), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str4, strlen((char *)str4), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str5, strlen((char *)str5), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str6, strlen((char *)str6), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str7, strlen((char *)str7), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str8, strlen((char *)str8), 0xFFFF);
	  	HAL_UART_Transmit(&huart1, str9, strlen((char *)str9), 0xFFFF);*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void setMPUSettings(MPU9250_t * MPU0250){
	//accel bias [g]:  11.1511   0.0000   0.0000  ␊
	  //gyro bias [deg/s]:  0.0000   0.6698   0.0000  ␊
	  //mag bias [mG]:  19.1250   143.4375   0.0000  ␊
	  //mag scale []:  1.3571   0.9382   0.8351
	setAccBias(MPU0250, 11.1511, 0.0, 0.00);
	setGyroBias(MPU0250, 0.0, 0.6698, 0.0);
	setMagBias(MPU0250, 19.125, 143.4375, 0.0000);
	setMagScale(MPU0250, 1.3571, 0.9382, 0.8351);
	/*setAccBias(MPU0250, 280.60, 163.00, 264.40);
	setGyroBias(MPU0250, 12.63, -72.50, -73.97);
	setMagBias(MPU0250, 75.03, 296.37, -636.671);
	setMagScale(MPU0250, 1.11, 1.03, 0.890);*/

	setMagneticDeclination(MPU0250, 10.91);
}

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

