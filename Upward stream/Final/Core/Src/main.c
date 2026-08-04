/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "sd_mcp23017.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)
typedef struct {
    uint16_t  node_id;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    uint8_t  pressure[3];
    int16_t  temperature;
    uint8_t  state_flags;
    uint8_t  checksum;
} TelemetryPacket_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t target_id;
    uint8_t cmd_id;
    int16_t param;
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

// ID вашего устройства
uint16_t MY_NODE_ID = 0xBBB9;
bool bmi_ok = false;
bool lis_ok = false;
bool bmp_ok = false;
bool lora_ready = false; // Флаг готовности LoRa для логов

// Глобальный буфер для хранения самых свежих данных
TelemetryPacket_t current_pkt;
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

// Универсальная функция для отправки логов в UART и по LoRa
void Send_Log(const char* msg) {
    // Вывод в UART (для локальной отладки)
    Serial_Printf(&huart1, "%s\r\n", msg);

    // Если LoRa уже работает, отправляем в радиоэфир
    if (lora_ready) {
        uint8_t lora_buf[128];

        // 1. Первые 2 байта всегда жестко задаем как наш бинарный ID (0xBBB9)
        uint16_t* id_ptr = (uint16_t*)lora_buf;
        *id_ptr = MY_NODE_ID;

        // 2. Начиная с 3-го байта (индекс 2) пишем сам текст
        int len = snprintf((char*)(lora_buf + 2), sizeof(lora_buf) - 2, "LOG: %s", msg);

        LoRa_BeginPacket(0);
        LoRa_Write(lora_buf, len + 2); // Длина текста + 2 байта ID
        LoRa_EndPacket(0);

        LoRa_Receive(0);
        HAL_Delay(50);
    }
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

    if (MCP23017_Init() != HAL_OK) {
        Serial_Printf(&huart1, "MCP23017 Expander NOT FOUND!\r\n");
    } else {
        Serial_Printf(&huart1, "MCP23017 OK\r\n");
    }

    // Подтягиваем CS всех датчиков
    MCP23017_WriteAll(0xFFFF);

    // Инициализируем LoRa ПЕРВЫМ из датчиков, чтобы отправлять статусы остальных
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
        lora_ready = true; // Разрешаем отправку логов по радио
        Send_Log("LoRa Ready!");
    }

    if (BMI088_Init(&hspi1) != HAL_OK) {
        Send_Log("BMI088 init failed!");
        bmi_ok = false;
    } else {
        Send_Log("BMI088 OK. Calibrating...");
        BMI088_CalibrateGyro(&hspi1, 200);
        bmi_ok = true;
    }

    if (LIS3MDL_Init() != 1) {
        Send_Log("LIS3MDL init failed!");
        lis_ok = false;
    } else {
        Send_Log("LIS3MDL OK");
        lis_ok = true;
        //LIS3MDL_Calibrate();
    }

    if (BMP388_Init() != BMP388_OK) {
        Send_Log("BMP388 init failed!");
        bmp_ok = false;
    } else {
        Send_Log("BMP388 OK");
        bmp_ok = true;
    }

    if (SD_MCP_Init(&hspi2) != 0) {
        Send_Log("SD Init Failed!");
    } else {
        Send_Log("SD OK!");
    }

    HAL_ADC_Start(&hadc1);

    uint32_t last_poll_tick = HAL_GetTick();
    uint32_t last_sd_tick   = HAL_GetTick();
    uint32_t last_tx_tick   = HAL_GetTick();

    uint32_t ds18b20_timer = 0;
    uint8_t ds18b20_state = 0;
    float current_temp_ds = 0.0f;
    uint16_t photo_val = 0;

    memset(&current_pkt, 0, sizeof(current_pkt));
    current_pkt.node_id = MY_NODE_ID;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      BMI088_Raw_t raw;
      BMI088_Physical_t phys;

      Send_Log("INIT DONE. Entering main loop...");

      while (1)
      {
          // --- 1. Обработка входящих команд LoRa ---
          if (_packetReady) {
              _packetReady = false;
              if (_pendingLength == sizeof(CommandPacket_t)) {
                  CommandPacket_t rx_cmd;
                  uint8_t *ptr = (uint8_t*)&rx_cmd;
                  int i = 0;
                  while (LoRa_Available() && i < sizeof(CommandPacket_t)) {
                      ptr[i++] = (uint8_t)LoRa_Read();
                  }
                  if (rx_cmd.target_id == MY_NODE_ID || rx_cmd.target_id == 0xFF) {
                      char cmd_log[64];
                      snprintf(cmd_log, sizeof(cmd_log), "RX CMD | ID: 0x%02X, Param: %d", rx_cmd.cmd_id, rx_cmd.param);
                      Send_Log(cmd_log);

                      switch (rx_cmd.cmd_id) {
                          case 0x01:
                              HAL_GPIO_WritePin(LED_PIN_1_GPIO_Port, LED_PIN_1_Pin, rx_cmd.param ? GPIO_PIN_SET : GPIO_PIN_RESET);
                              break;
                          case 0x02:
                              Send_Log("Action: System Rebooting...");
                              HAL_Delay(100);
                              NVIC_SystemReset();
                              break;
                      }
                  }
              } else {
                  while (LoRa_Available()) LoRa_Read();
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

          // --- 3. Опрос датчиков (Например, 50 Гц / каждые 20 мс) ---
          if (HAL_GetTick() - last_poll_tick >= 20) {
              last_poll_tick = HAL_GetTick();
              current_pkt.state_flags = 0;

              HAL_ADC_Start(&hadc1);
              if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                  photo_val = HAL_ADC_GetValue(&hadc1);
              }

              if (bmi_ok && (BMI088_ReadRaw(&hspi1, &raw) == HAL_OK)) {
                  BMI088_ToPhysical(&raw, &phys);
                  current_pkt.accel_x = (int16_t)(phys.Accel_X * 100);
                  current_pkt.accel_y = (int16_t)(phys.Accel_Y * 100);
                  current_pkt.accel_z = (int16_t)(phys.Accel_Z * 100);
                  current_pkt.gyro_x  = (int16_t)(phys.Gyro_X * 100);
                  current_pkt.gyro_y  = (int16_t)(phys.Gyro_Y * 100);
                  current_pkt.gyro_z  = (int16_t)(phys.Gyro_Z * 100);
                  current_pkt.state_flags |= (1 << 0);
              }

              if (lis_ok) {
                  LIS3MDL_Read_Mag(&current_pkt.mag_x, &current_pkt.mag_y, &current_pkt.mag_z);
                  current_pkt.state_flags |= (1 << 1);
              }

              if (bmp_ok) {
                  float bmp_temp, bmp_press;
                  if (BMP388_Read(&bmp_temp, &bmp_press) == BMP388_OK) {
                      uint32_t press_int = (uint32_t)(bmp_press * 100);
                      current_pkt.pressure[0] = (press_int >> 16) & 0xFF;
                      current_pkt.pressure[1] = (press_int >> 8) & 0xFF;
                      current_pkt.pressure[2] = press_int & 0xFF;
                      current_pkt.temperature = (int16_t)(bmp_temp * 100);
                      current_pkt.state_flags |= (1 << 2);
                  }
              }

              if (current_temp_ds != 0.0f) {
                  current_pkt.temperature = (int16_t)(current_temp_ds * 100);
                  current_pkt.state_flags |= (1 << 3);
              }
          }

          // --- 4. Запись на SD карту (Например, 10 Гц / каждые 100 мс) ---
          if (HAL_GetTick() - last_sd_tick >= 100) {
              last_sd_tick = HAL_GetTick();

              char sd_buffer[200];
              uint32_t press_val = ((uint32_t)current_pkt.pressure[0] << 16) |
                                   ((uint32_t)current_pkt.pressure[1] << 8) |
                                   current_pkt.pressure[2];

              snprintf(sd_buffer, sizeof(sd_buffer), "%X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%u\n",
                       current_pkt.node_id,
                       current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
                       current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
                       current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
                       press_val, current_pkt.temperature, current_pkt.state_flags);

              SD_MCP_WriteFile("data.csv", sd_buffer);
          }

          // --- 5. Отправка телеметрии по LoRa (Например, 1 Гц / каждую 1000 мс) ---
          if (HAL_GetTick() - last_tx_tick >= 1000) {
              last_tx_tick = HAL_GetTick();

              uint8_t calc_crc = 0;
              uint8_t *ptr = (uint8_t*)&current_pkt;
              for (int i = 0; i < sizeof(TelemetryPacket_t) - 1; i++) {
                  calc_crc ^= ptr[i];
              }
              current_pkt.checksum = calc_crc;

              uint32_t press_val = ((uint32_t)current_pkt.pressure[0] << 16) |
                                 ((uint32_t)current_pkt.pressure[1] << 8) |
                                 current_pkt.pressure[2];

              Serial_Printf(&huart1, "TX Node:%X | Acc:%d,%d,%d | Gyr:%d,%d,%d | Mag:%d,%d,%d | Press:%lu | Temp:%d | Flags:0x%02X | CRC:0x%02X\r\n",
                     current_pkt.node_id,
                     current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
                     current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
                     current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
                     press_val, current_pkt.temperature, current_pkt.state_flags, current_pkt.checksum);

              LoRa_BeginPacket(0);
              LoRa_Write((uint8_t*)&current_pkt, sizeof(TelemetryPacket_t));
              LoRa_EndPacket(0);

              LoRa_Receive(0);
          }
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
