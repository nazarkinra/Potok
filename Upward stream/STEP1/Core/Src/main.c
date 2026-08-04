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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial_print.h"
#include "mcp23017.h"
#include "bmi088.h"
#include "lis3mdl.h"
#include "bmp388.h"
#include "lora278.h"
#include "dwt_delay.h"
#include "ds18b20.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_PIN_1_GPIO_Port, LED_PIN_1_Pin, 1);

    Serial_Printf(&huart1, "\r\n--- Sensors ---\r\n");

    // Инициализация расширителя
    if (MCP23017_Init() != HAL_OK) {
        Serial_Printf(&huart1, "MCP23017 Expander NOT FOUND!\r\n");
    } else {
        Serial_Printf(&huart1, "MCP23017 OK\r\n");
    }

    MCP23017_WriteAll(0xFFFF);
    /*while(1){
    	// --- 2. LIS3MDL (Магнитометр) ---
		// 0x0F - адрес регистра WHO_AM_I
		uint8_t id_lis = LIS3MDL_ReadReg(0x0F);
		if (id_lis == 0x3D) {
			Serial_Printf(&huart1, "LIS3MDL ALIVE! ID: 0x3D\r\n");
		} else {
			Serial_Printf(&huart1, "LIS3MDL SILENT! ID: 0x%02X\r\n", id_lis);
		}
    }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	// --- 1. BMP388 (Барометр) ---
		uint8_t id_bmp = BMP388_ReadReg(0x00);
		if (id_bmp == 0x50) {
			Serial_Printf(&huart1, "BMP388 ALIVE! ID: 0x50\r\n");
		} else {
			Serial_Printf(&huart1, "BMP388 SILENT! ID: 0x%02X\r\n", id_bmp);
		}

		// --- 2. LIS3MDL (Магнитометр) ---
		// 0x0F - адрес регистра WHO_AM_I
		uint8_t id_lis = LIS3MDL_ReadReg(0x0F);
		if (id_lis == 0x3D) {
			Serial_Printf(&huart1, "LIS3MDL ALIVE! ID: 0x3D\r\n");
		} else {
			Serial_Printf(&huart1, "LIS3MDL SILENT! ID: 0x%02X\r\n", id_lis);
		}

		// --- 3. BMI088 Акселерометр ---
		// Акселерометр требует 1 dummy байт
		uint8_t tx_acc[3] = {0x00 | 0x80, 0xFF, 0xFF};
		uint8_t rx_acc[3] = {0};
		MCP23017_WritePin(2, 0); // Опускаем CS (программный пин 2)
		HAL_SPI_TransmitReceive(&hspi1, tx_acc, rx_acc, 3, 100);
		MCP23017_WritePin(2, 1); // Поднимаем CS
		if (rx_acc[2] == 0x1E) {
			Serial_Printf(&huart1, "BMI088 ACC ALIVE! ID: 0x1E\r\n");
		} else {
			Serial_Printf(&huart1, "BMI088 ACC SILENT! ID: 0x%02X\r\n", rx_acc[2]);
		}

		// --- 4. BMI088 Гироскоп ---
		// Гироскоп читается стандартно, без dummy байта
		uint8_t tx_gyr[2] = {0x00 | 0x80, 0xFF};
		uint8_t rx_gyr[2] = {0};
		MCP23017_WritePin(4, 0); // Опускаем CS (программный пин 4)
		HAL_SPI_TransmitReceive(&hspi1, tx_gyr, rx_gyr, 2, 100);
		MCP23017_WritePin(4, 1); // Поднимаем CS
		if (rx_gyr[1] == 0x0F) {
			Serial_Printf(&huart1, "BMI088 GYR ALIVE! ID: 0x0F\r\n");
		} else {
			Serial_Printf(&huart1, "BMI088 GYR SILENT! ID: 0x%02X\r\n", rx_gyr[1]);
		}

		Serial_Printf(&huart1, "--------------------------\r\n");
		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
