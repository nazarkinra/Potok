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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial_print.h"
#include "lis3mdl.h"
#include "bmp388.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "lora278.h"
#include "motor_control.h"
#include "pca9685.h"
#include <stdio.h>
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
HMC5883L_Data_t mag_data;
struct bmp388_dev bmp388_dev_obj;
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
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  Serial_Printf(&huart1, "\r\n--- Step1_Sensors INIT ---\r\n");

  if (MPU6050_Init(&hi2c1) != HAL_OK) {
      if (MPU6050_Init(&hi2c3) != HAL_OK) {
          Serial_Printf(&huart1, "MPU6050 init failed!\r\n");
      } else {
          Serial_Printf(&huart1, "MPU6050 initialized on I2C3\r\n");
      }
  } else {
      Serial_Printf(&huart1, "MPU6050 initialized on I2C1\r\n");
  }

  if (HMC5883L_Init(&hi2c1) != HAL_OK) {
       if (HMC5883L_Init(&hi2c3) != HAL_OK) {
           Serial_Printf(&huart1, "HMC5883L init failed!\r\n");
       } else {
           Serial_Printf(&huart1, "HMC5883L initialized on I2C3\r\n");
       }
  } else {
      Serial_Printf(&huart1, "HMC5883L initialized on I2C1\r\n");
  }

  int8_t rslt = BMP388_Init(&bmp388_dev_obj, &hi2c1);
  if (rslt != 0) {
      rslt = BMP388_Init(&bmp388_dev_obj, &hi2c3);
      if (rslt != 0) {
          Serial_Printf(&huart1, "BMP388 init failed!\r\n");
      } else {
          Serial_Printf(&huart1, "BMP388 initialized on I2C3\r\n");
      }
  } else {
      Serial_Printf(&huart1, "BMP388 initialized on I2C1\r\n");
  }

  uint32_t last_telemetry_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char uart_buf[200];
  MPU6050_Raw_t raw;
  MPU6050_Physical_t phys;
  struct bmp3_data bmp_data = {0};

  while (1)
  {
      if (HAL_GetTick() - last_telemetry_tick >= 100) {
          last_telemetry_tick = HAL_GetTick();

          float accel_x = 0, accel_y = 0, accel_z = 0;
          float gyro_x = 0, gyro_y = 0, gyro_z = 0;
          float mag_x = 0, mag_y = 0, mag_z = 0;

          if (MPU6050_ReadRaw(&hi2c1, &raw) == HAL_OK || MPU6050_ReadRaw(&hi2c3, &raw) == HAL_OK) {
              MPU6050_ToPhysical(&raw, &phys);
              accel_x = phys.Accel_X;
              accel_y = phys.Accel_Y;
              accel_z = phys.Accel_Z;
              gyro_x = phys.Gyro_X;
              gyro_y = phys.Gyro_Y;
              gyro_z = phys.Gyro_Z;
          }

          if (HMC5883L_ReadData(&hi2c1, &mag_data) == HAL_OK || HMC5883L_ReadData(&hi2c3, &mag_data) == HAL_OK) {
              mag_x = mag_data.x * 0.92f;
              mag_y = mag_data.y * 0.92f;
              mag_z = mag_data.z * 0.92f;
          }

          BMP388_ReadData(&bmp388_dev_obj, &bmp_data);

          // Avoid float printf due to GCC default config on STM32
          int ax = (int)(accel_x * 100), ay = (int)(accel_y * 100), az = (int)(accel_z * 100);
          int gx = (int)(gyro_x * 100), gy = (int)(gyro_y * 100), gz = (int)(gyro_z * 100);
          int mx = (int)(mag_x * 100), my = (int)(mag_y * 100), mz = (int)(mag_z * 100);
          int press = (int)(bmp_data.pressure / 100); // hPa
          int temp = (int)(bmp_data.temperature * 100);

          sprintf(uart_buf, "ACC:%d.%02d,%d.%02d,%d.%02d GYRO:%d.%02d,%d.%02d,%d.%02d MAG:%d.%02d,%d.%02d,%d.%02d BMP:%d hPa, %d.%02d C\r\n",
                   ax/100, abs(ax)%100, ay/100, abs(ay)%100, az/100, abs(az)%100,
                   gx/100, abs(gx)%100, gy/100, abs(gy)%100, gz/100, abs(gz)%100,
                   mx/100, abs(mx)%100, my/100, abs(my)%100, mz/100, abs(mz)%100,
                   press, temp/100, abs(temp)%100);

          Serial_Printf(&huart1, "%s", uart_buf);
      }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
