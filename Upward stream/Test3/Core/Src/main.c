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
// Жестко упаковываем структуру пакета для бинарной передачи по радио
#pragma pack(push, 1)
typedef struct {
    uint16_t  node_id;       // Мой ID (2 байт)
    int16_t  accel_x;       // Ускорение X (2 байта)
    int16_t  accel_y;       // Ускорение Y (2 байта)
    int16_t  accel_z;       // Ускорение Z (2 байта)
    int16_t  gyro_x;        // Гироскоп X (2 байта)
    int16_t  gyro_y;        // Гироскоп Y (2 байта)
    int16_t  gyro_z;        // Гироскоп Z (2 байта)
    int16_t  mag_x;         // Магнитометр X (2 байта)
    int16_t  mag_y;         // Магнитометр Y (2 байта)
    int16_t  mag_z;         // Магнитометр Z (2 байта)
    uint8_t  pressure[3];   // Давление (3 байта)
    int16_t  temperature;   // Температура (2 байта)
    uint8_t  state_flags;   // Флаги состояния (1 байт)
    uint8_t  checksum;
} TelemetryPacket_t;
#pragma pack(pop)
// Жестко упаковываем структуру пакета для входящих команд
#pragma pack(push, 1)
typedef struct {
    uint16_t target_id;   // ID узла, которому адресована команда (0xFF - широковещательная для всех)
    uint8_t cmd_id;      // Код команды (например: 0x01 - светодиод, 0x02 - перезагрузка)
    int16_t param;       // Универсальный параметр (например, задержка или значение ШИМ)
    uint8_t  checksum;
} CommandPacket_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile bool _packetReady = false;
volatile int _pendingLength = 0;
extern ADC_HandleTypeDef hadc1;

// ID вашего устройства и статусы датчиков
uint16_t MY_NODE_ID = 0xBBB9;
bool bmi_ok = false;
bool lis_ok = false;
bool bmp_ok = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void OnLoRaPacketReceived(int packetSize) {
    _pendingLength = packetSize;
    _packetReady = true;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    DWT_Init();
    HAL_GPIO_WritePin(LED_PIN_1_GPIO_Port, LED_PIN_1_Pin, 1);

    Serial_Printf(&huart1, "\r\n--- Sensors & LoRa INIT ---\r\n");

    // Инициализация расширителя MCP23017
    if (MCP23017_Init() != HAL_OK) {
        Serial_Printf(&huart1, "MCP23017 Expander NOT FOUND!\r\n");
    } else {
        Serial_Printf(&huart1, "MCP23017 OK\r\n");
    }

    // Подтягиваем все выходы CS в HIGH
    MCP23017_WriteAll(0xFFFF);

    // Инициализация BMI088
    if (BMI088_Init(&hspi1) != HAL_OK) {
        Serial_Printf(&huart1, "BMI088 init failed!\r\n");
        bmi_ok = false;
    } else {
        Serial_Printf(&huart1, "BMI088 OK. Calibrating...\r\n");
        BMI088_CalibrateGyro(&hspi1, 200);
        bmi_ok = true;
    }

    // Инициализация магнитометра LIS3MDL[cite: 3, 5]
    if (LIS3MDL_Init() != 1) {
        Serial_Printf(&huart1, "LIS3MDL init failed!\r\n");
        lis_ok = false;
    } else {
        Serial_Printf(&huart1, "LIS3MDL OK\r\n");
        lis_ok = true;
        LIS3MDL_Calibrate();
    }

    // Инициализация барометра BMP388[cite: 2, 4]
    if (BMP388_Init() != BMP388_OK) {
        Serial_Printf(&huart1, "BMP388 init failed!\r\n");
        bmp_ok = false;
    } else {
        Serial_Printf(&huart1, "BMP388 OK\r\n");
        bmp_ok = true;
    }

    // Инициализация LoRa
    LoRa_Init();
    if (!LoRa_Begin(441000000)) {
        Serial_Printf(&huart1, "LoRa init failed!\r\n");
    } else {
        LoRa_SetSpreadingFactor(7);
        LoRa_SetSignalBandwidth(125000);
        LoRa_SetCodingRate4(5);
        LoRa_EnableCrc();
        LoRa_SetSyncWord(0x12);
        LoRa_SetTxPower(20, 1);

        LoRa_OnReceive(OnLoRaPacketReceived);
        LoRa_Receive(0);
        Serial_Printf(&huart1, "LoRa Ready!\r\n");
    }

    HAL_ADC_Start(&hadc1);

    uint32_t last_telemetry_tick = HAL_GetTick();
    uint32_t counter = 0;

    uint32_t ds18b20_timer = 0;
    uint8_t ds18b20_state = 0;
    float current_temp_ds = 0.0f;
    uint16_t photo_val = 0;
    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char uart_buf[200];
      char command[50];
      BMI088_Raw_t raw;
      BMI088_Physical_t phys;

      while (1)
      {
    	    // --- 1. Обработка входящих команд LoRa ---
			if (_packetReady) {
				_packetReady = false;
				// Проверяем, соответствует ли размер пакета нашей структуре команды
				if (_pendingLength == sizeof(CommandPacket_t)) {
					CommandPacket_t rx_cmd;
					uint8_t *ptr = (uint8_t*)&rx_cmd;
					int i = 0;
					// Считываем данные из FIFO буфера LoRa в структуру байт за байтом
					while (LoRa_Available() && i < sizeof(CommandPacket_t)) {
						ptr[i++] = (uint8_t)LoRa_Read();
					}
					// Проверяем, нам ли адресована эта команда (или всем сразу через 0xFF)
					if (rx_cmd.target_id == MY_NODE_ID || rx_cmd.target_id == 0xFF) {
						Serial_Printf(&huart1, "RX CMD | ID: 0x%02X, Param: %d\r\n", rx_cmd.cmd_id, rx_cmd.param);
						// Здесь в будущем можно будет добавить обработчик
						switch (rx_cmd.cmd_id) {
							case 0x01: // Пример: Управление отладочным светодиодом
								HAL_GPIO_WritePin(LED_PIN_1_GPIO_Port, LED_PIN_1_Pin, rx_cmd.param ? GPIO_PIN_SET : GPIO_PIN_RESET);
								Serial_Printf(&huart1, "Action: LED state changed to %d\r\n", rx_cmd.param);
								break;
							case 0x02: // Пример: Программная перезагрузка МК
								Serial_Printf(&huart1, "Action: System Rebooting...\r\n");
								HAL_Delay(100);
								NVIC_SystemReset();
								break;
							default:
								Serial_Printf(&huart1, "Unknown command ID!\r\n");
								break;
						}
					} else {
						Serial_Printf(&huart1, "CMD ignored (Target ID mismatch: 0x%02X)\r\n", rx_cmd.target_id);
					}
				} else {
					// Если пришел пакет другого размера (например, чужая телеметрия или мусор),
					// обязательно вычитываем буфер до конца, чтобы не застрять в прерываниях
					while (LoRa_Available()) {
						LoRa_Read();
					}
					Serial_Printf(&huart1, "RX Unknown packet size: %d bytes\r\n", _pendingLength);
				}
			}

          // --- 2. Неблокирующий опрос DS18B20 ---
          if (ds18b20_state == 0) {
              if (DS18B20_Start()) {
                  DS18B20_WriteByte(0xCC);
                  DS18B20_WriteByte(0x44);
                  ds18b20_timer = HAL_GetTick();
                  ds18b20_state = 1;
              }
          }
          else if (ds18b20_state == 1) {
              if (HAL_GetTick() - ds18b20_timer >= 750) {
                  if (DS18B20_Start()) {
                      DS18B20_WriteByte(0xCC);
                      DS18B20_WriteByte(0xBE);

                      uint8_t temp_lsb = DS18B20_ReadByte();
                      uint8_t temp_msb = DS18B20_ReadByte();

                      int16_t temp_raw = (temp_msb << 8) | temp_lsb;
                      current_temp_ds = temp_raw / 16.0f;
                  }
                  ds18b20_state = 0;
              }
          }

          // --- 3. Сбор и отправка телеметрии ---
          if (HAL_GetTick() - last_telemetry_tick >= 1000) {
			last_telemetry_tick = HAL_GetTick();
			counter++;

			TelemetryPacket_t pkt;
			memset(&pkt, 0, sizeof(pkt)); // Очистка памяти перед заполнением
			pkt.node_id = MY_NODE_ID;
			pkt.state_flags = 0; // Изначально все флаги сброшены (0)

			// Опрос фоторезистора (опционально, если нужно будет добавить в пакет позже)
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
				photo_val = HAL_ADC_GetValue(&hadc1);
			}

			// 1. Ускорение и Гироскоп (BMI088)
			if (bmi_ok && (BMI088_ReadRaw(&hspi1, &raw) == HAL_OK)) {
				BMI088_ToPhysical(&raw, &phys);
				pkt.accel_x = (int16_t)(phys.Accel_X * 100);
				pkt.accel_y = (int16_t)(phys.Accel_Y * 100);
				pkt.accel_z = (int16_t)(phys.Accel_Z * 100);
				pkt.gyro_x  = (int16_t)(phys.Gyro_X * 100);
				pkt.gyro_y  = (int16_t)(phys.Gyro_Y * 100);
				pkt.gyro_z  = (int16_t)(phys.Gyro_Z * 100);
				pkt.state_flags |= (1 << 0); // Устанавливаем нулевой бит флага (BMI088 OK)
			} else {
				pkt.accel_x = INT16_MAX; pkt.accel_y = INT16_MAX; pkt.accel_z = INT16_MAX;
				pkt.gyro_x  = INT16_MAX; pkt.gyro_y  = INT16_MAX; pkt.gyro_z  = INT16_MAX;
			}

			// 2. Магнитометр (LIS3MDL)
			if (lis_ok) {
				// Функция LIS3MDL_Read_Mag напрямую записывает данные в формате int16_t[cite: 3, 5]
				LIS3MDL_Read_Mag(&pkt.mag_x, &pkt.mag_y, &pkt.mag_z);
				pkt.state_flags |= (1 << 1); // Устанавливаем первый бит флага (LIS3MDL OK)
			} else {
				pkt.mag_x = INT16_MAX; pkt.mag_y = INT16_MAX; pkt.mag_z = INT16_MAX;
			}

			// 3. Давление и Температура (BMP388)
			if (bmp_ok) {
				float bmp_temp, bmp_press;
				// Функция BMP388_Read принимает указатели на float для температуры и давления[cite: 2, 4]
				if (BMP388_Read(&bmp_temp, &bmp_press) == BMP388_OK) {

					// Умножаем давление на 100 (чтобы не терять точность) и разбираем на 3 байта
					uint32_t press_int = (uint32_t)(bmp_press * 100);
					pkt.pressure[0] = (press_int >> 16) & 0xFF; // Старший байт
					pkt.pressure[1] = (press_int >> 8) & 0xFF;  // Средний байт
					pkt.pressure[2] = press_int & 0xFF;         // Младший байт

					// Температура с барометра
					pkt.temperature = (int16_t)(bmp_temp * 100);
					pkt.state_flags |= (1 << 2); // Устанавливаем второй бит флага (BMP388 OK)
				}
			} else {
				// Если не работает, ставим максимальные значения
				pkt.pressure[0] = 0xFF; pkt.pressure[1] = 0xFF; pkt.pressure[2] = 0xFF;
				pkt.temperature = INT16_MAX;
			}

			// 4. Приоритетная Температура (DS18B20)
			// Если внешний датчик выдал адекватную температуру, перезаписываем показания с барометра
			if (current_temp_ds != 0.0f) {
				pkt.temperature = (int16_t)(current_temp_ds * 100);
				pkt.state_flags |= (1 << 3); // Устанавливаем третий бит (DS18B20 OK)
			}

			// --- Расчет контрольной суммы (XOR всех байт, кроме самого поля checksum) ---
			uint8_t calc_crc = 0;
			uint8_t *ptr = (uint8_t*)&pkt;
			for (int i = 0; i < sizeof(TelemetryPacket_t) - 1; i++) {
			  calc_crc ^= ptr[i];
			}
			pkt.checksum = calc_crc; // Записываем в последний байт пакета

			// Вывод в UART для отладки
			uint32_t press_val = ((uint32_t)pkt.pressure[0] << 16) |
							   ((uint32_t)pkt.pressure[1] << 8) |
							   pkt.pressure[2];

			Serial_Printf(&huart1, "TX Node:%04X | Acc:%d.%02d,%d.%02d,%d.%02d | Gyr:%d.%02d,%d.%02d,%d.%02d | Mag:%d,%d,%d | Press:%lu.%02lu | Temp:%d.%02d | Flags:0x%02X | CRC:0x%02X\r\n",
				   pkt.node_id,
				   pkt.accel_x/100, abs(pkt.accel_x)%100,
				   pkt.accel_y/100, abs(pkt.accel_y)%100,
				   pkt.accel_z/100, abs(pkt.accel_z)%100,
				   pkt.gyro_x/100, abs(pkt.gyro_x)%100,
				   pkt.gyro_y/100, abs(pkt.gyro_y)%100,
				   pkt.gyro_z/100, abs(pkt.gyro_z)%100,
				   pkt.mag_x, pkt.mag_y, pkt.mag_z,
				   press_val/100, press_val%100,
				   pkt.temperature/100, abs(pkt.temperature)%100,
				   pkt.state_flags,
				   pkt.checksum); // Выводим CRC в консоль

			// Отправка бинарного пакета по LoRa
			LoRa_BeginPacket(0);
			LoRa_Write((uint8_t*)&pkt, sizeof(TelemetryPacket_t));
			LoRa_EndPacket(0);

			// Возврат в режим приёма
			LoRa_Receive(0);
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
