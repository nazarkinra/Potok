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
#include "lora278.h"
#include "hmc5883l.h"
#include "bmp280.h"
#include "mpu6050.h"
#include <stdio.h>
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
#define LORA_FREQUENCY    433000000UL
#define LORA_SF           7
#define LORA_BW           125000UL
#define LORA_SYNC_WORD    0x12
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
static uint32_t last_tx_tick = 0;
static char tx_buf[200];
HMC5883L_Data_t mag_data;
BMP280_t sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile bool _packetReady = false;
volatile int _pendingLength = 0;
void OnLoRaPacketReceived(int packetSize) {
    _pendingLength = packetSize;
    _packetReady = true;  // 👈 Только флаг, без чтения!
}
void I2C_Scan(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    char buf[64];
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n--- I2C BUS SCAN ---\r\n", 21, 100);

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        // HAL_I2C_IsDeviceReady принимает 8-битный адрес (addr << 1)
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 50) == HAL_OK) {
            snprintf(buf, sizeof(buf), "[OK] 7-bit: 0x%02X | 8-bit: 0x%02X\r\n", addr, addr << 1);
            HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), 100);
        }
    }
    HAL_UART_Transmit(huart, (uint8_t*)"--- SCAN END ---\r\n", 17, 100);
}
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  // 1. Проверяем состояние SPI
  Serial_Printf(&huart1, "SPI State: 0x%02X\r\n", HAL_SPI_GetState(&hspi2));
  // Должно быть 0x20 (HAL_SPI_STATE_READY). Если 0x00 → MX_SPI2_Init() не вызван.

  // 2. Прямой тест SPI без обёрток
  uint8_t tx = 0x42 & 0x7F; // REG_VERSION
  uint8_t rx = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_StatusTypeDef s1 = HAL_SPI_Transmit(&hspi2, &tx, 1, 10);
  tx = 0x00;
  HAL_StatusTypeDef s2 = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  Serial_Printf(&huart1, "Tx Status: %d | Rx Status: %d | Got: 0x%02X\r\n", s1, s2, rx);

  uint8_t ver = LoRa_ReadReg(REG_VERSION);
  while(ver!=0x12){
	  ver = LoRa_ReadReg(REG_VERSION);
	  Serial_Printf(&huart1, "LoRa Version: 0x%02X\r\n", ver);
	  HAL_Delay(100);
	  break;
  }

  // 🚀 Инициализация радио
  if (!LoRa_Begin(433000000)) {
	  Serial_Printf(&huart1, "❌ LoRa init failed! Check wiring & SPI speed.\r\n");
	  Error_Handler();
  }

  // ⚙️ Параметры (точно как в твоём Arduino скетче)
  LoRa_SetSpreadingFactor(7);
  LoRa_SetSignalBandwidth(125000);
  LoRa_SetCodingRate4(5);
  LoRa_EnableCrc();
  LoRa_SetSyncWord(0x12);
  LoRa_SetTxPower(17, 1); // 17 dBm, PA_BOOST

  Serial_Printf(&huart1, "📡 Ready. Sending every 2s...\r\n");
  LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
  if (HMC5883L_Init(&hi2c1) != HAL_OK) {
	  // Ошибка: датчик не отвечает или неверный ID
	  Error_Handler();
  }
  I2C_Scan(&hi2c1, &huart1);
  sensor.hi2c = &hi2c1;
  Serial_Printf(&huart1, "Init sensor... ");
  if (BMP280_Init(&sensor) == HAL_OK) {
      Serial_Printf(&huart1, "OK (chip: %s)\r\n", sensor.chip_id == 0x60 ? "BME280" : "BMP280");
  } else {
      Serial_Printf(&huart1, "FAIL\r\n");
      Error_Handler();
  }

  if (MPU6050_Init(&hi2c1) != HAL_OK) {
	  // Ошибка инициализации: проверьте подключение, подтяжки, адрес
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char uart_buf[70];
  MPU6050_Raw_t raw;
  MPU6050_Physical_t phys;
  while (1) {
	  /*if (_packetReady) {
		  _packetReady = false;
		  printf("📦 Пакет %d байт: ", _pendingLength);
		  while (LoRa_Available()) {
			  printf("%c", (char)LoRa_Read());
		  }
		  printf("\r\n");
	  }
	  HAL_Delay(10);*/
	  char buf[128];
	  if (MPU6050_ReadRaw(&hi2c1, &raw) == HAL_OK) {
		  MPU6050_ToPhysical(&raw, &phys);

		  // Вывод через UART (или используйте свою функцию логирования)

		  snprintf(buf, sizeof(buf),
				   "Accel: X=%.3f Y=%.3f Z=%.3f | Gyro: X=%.2f Y=%.2f Z=%.2f | T=%.1f°C\r\n",
				   phys.Accel_X, phys.Accel_Y, phys.Accel_Z,
				   phys.Gyro_X, phys.Gyro_Y, phys.Gyro_Z,
				   phys.Temp_C);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	  }
	  float t, p, h;
	  if (BMP280_GetData(&sensor, &t, &p, &h) == HAL_OK) {
		  Serial_Printf(&huart1, "T: %.2f C | P: %.2f Pa", t, p);
		  if (sensor.chip_id == 0x60 && h >= 0) {
			  Serial_Printf(&huart1, " | H: %.2f %%", h);
		  }
		  Serial_Printf(&huart1, "\r\n");
	  }

	  HAL_StatusTypeDef status = HMC5883L_ReadData(&hi2c1, &mag_data);

	  if (status == HAL_OK) {
		  // Преобразование в микроТеслы (для gain ±1.3G: 1 LSB = 0.92 µT)
		  float x_uT = mag_data.x * 0.92f;
		  float y_uT = mag_data.y * 0.92f;
		  float z_uT = mag_data.z * 0.92f;

		  snprintf(uart_buf, sizeof(uart_buf), "X: %+5d (%.2f uT) | Y: %+5d (%.2f uT) | Z: %+5d (%.2f uT)\r\n",
				   mag_data.x, x_uT, mag_data.y, y_uT, mag_data.z, z_uT);
		  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
	  } else if (status == HAL_BUSY) {
		  // Данные ещё не готовы (нормально при частоте опроса > 15 Гц)
	  } else {
		  // I2C ошибка
	  }
	  if (HAL_GetTick() - last_tx_tick >= 2000) {
		  last_tx_tick = HAL_GetTick();
		  snprintf(tx_buf, sizeof(tx_buf), "T: %.2f C | P: %.2f Pa | H: %.2f %%\r\n%s%s\r\n", t, p, h, uart_buf, buf);

		  Serial_Printf(&huart1, "TX: %s\r\n", tx_buf);

		  LoRa_BeginPacket(false);                     // Explicit header
		  LoRa_Write((uint8_t*)tx_buf, strlen(tx_buf)); // Загрузка в FIFO
		  int status = LoRa_EndPacket(false);          // Переключение в Tx и ожидание

		  if (status > 0)
			  Serial_Printf(&huart1, "✅ Sent OK\r\n");
		  else
			  Serial_Printf(&huart1, "❌ TX Error / Timeout\r\n");
	  }
	  HAL_Delay(2000);
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == LORA_DIO0_PIN) {
        LoRa_HandleDio0Rise();  // 👈 Обработка приёма
    }
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
