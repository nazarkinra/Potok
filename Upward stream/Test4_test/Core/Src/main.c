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
#include "fatfs.h"
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
#include "pca9685.h"
#include "servo_control.h"
#include "motor_control.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern uint8_t LoRa_ReadReg(uint8_t addr);
extern void LoRa_WriteReg(uint8_t addr, uint8_t val);
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
    int16_t  altitude;
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
Servo_HandleTypeDef myServo1;
Servo_HandleTypeDef myServo2;

volatile bool lora_irq_triggered = false;
Motor_HandleTypeDef motorE;
// Базовое давление на земле для планера (по умолчанию стандартное 1013.25 гПа)
float ground_pressure = 1013.25f;

typedef enum { STATE_WAIT, STATE_FLIGHT, STATE_EMERGENCY } State_t;
State_t current_state = STATE_WAIT;

float target_roll = 0, target_pitch = -5.0f, target_yaw = 0;
float prev_err_roll = 0, prev_err_pitch = 0, prev_err_yaw = 0;
float integral_roll = 0, integral_pitch = 0, integral_yaw = 0;

// PID constants (dummy values, need tuning)
float kp_r = 1.0f, ki_r = 0.01f, kd_r = 0.5f;
float kp_p = 1.0f, ki_p = 0.01f, kd_p = 0.5f;
float kp_y = 1.0f, ki_y = 0.01f, kd_y = 0.5f;

float roll = 0, pitch = 0, yaw = 0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
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

      // Инициализация PCA9685 на 50 Гц для сервомоторов
      PCA9685_Init(50);
      Motor_Init(&motorE, 13, 14);

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
          LoRa_SetTxPower(10, 1);

          //LoRa_OnReceive(OnLoRaPacketReceived);
          LoRa_Receive(0);
          lora_ready = true; // Разрешаем отправку логов по радио
          Send_Log("LoRa Ready!");
      }

      if (HAL_I2C_IsDeviceReady(&hi2c1, PCA9685_I2C_ADDR << 1, 3, 100) == HAL_OK) {
              Send_Log("PCA9685 is online!");
          } else {
              Send_Log("ERROR: PCA9685 NOT found!");
          }

          // Инициализация сервоприводов (каналы 7 и 4)
          Servo_Init(&myServo1, 7);
          Servo_Init(&myServo2, 4);

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

      // Инициализация карты
	  if (SD_MCP_Init(&hspi2) == 0) {
		  Send_Log("\r\n[MAIN] SD карта готова.\r\n");

		  // СРАЗУ ОТКРЫВАЕМ ФАЙЛ ДЛЯ ЛОГОВ!
		  if (SD_MCP_OpenFile("0:/data.csv") == 0) {
			  Send_Log("[MAIN] Файл data.csv открыт для записи.");
		  } else {
			  Send_Log("[MAIN] Ошибка открытия data.csv!");
		  }
	  } else {
		  Send_Log("\r\n[MAIN] Критическая ошибка инициализации SD.\r\n");
	  }

    HAL_ADC_Start(&hadc1);

    uint32_t last_poll_tick = HAL_GetTick();
    uint32_t last_sd_tick   = HAL_GetTick();
    uint32_t last_tx_tick   = HAL_GetTick();
    uint32_t last_pid_tick  = HAL_GetTick();

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
              if (HAL_GetTick() - last_pid_tick >= 10) {
                  last_pid_tick = HAL_GetTick();

                  // Separation simulation
                  int adc_sim = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET ? 1 : 0;
                  if (current_state == STATE_WAIT && adc_sim == 1) {
                      current_state = STATE_FLIGHT;
                      Send_Log(">>> SEPARATION DETECTED! SWITCHING TO FLIGHT MODE <<<");
                  }

                  if (current_state == STATE_FLIGHT) {
                      float dt = 0.01f;

                      float accel_x = current_pkt.accel_x / 100.0f;
                      float accel_y = current_pkt.accel_y / 100.0f;
                      float accel_z = current_pkt.accel_z / 100.0f;
                      float gyro_x = current_pkt.gyro_x / 100.0f;
                      float gyro_y = current_pkt.gyro_y / 100.0f;
                      float gyro_z = current_pkt.gyro_z / 100.0f;

                      MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

                      // Calculate angles from quaternions
                      roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
                      pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
                      yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f;

                      // Roll PID
                      float err_r = target_roll - roll;
                      integral_roll += err_r * dt;
                      float deriv_r = (err_r - prev_err_roll) / dt;
                      float out_r = (kp_r * err_r) + (ki_r * integral_roll) + (kd_r * deriv_r);
                      prev_err_roll = err_r;

                      // Pitch PID
                      float err_p = target_pitch - pitch;
                      integral_pitch += err_p * dt;
                      float deriv_p = (err_p - prev_err_pitch) / dt;
                      float out_p = (kp_p * err_p) + (ki_p * integral_pitch) + (kd_p * deriv_p);
                      prev_err_pitch = err_p;

                      // Yaw PID
                      float err_y = target_yaw - yaw;
                      integral_yaw += err_y * dt;
                      float deriv_y = (err_y - prev_err_yaw) / dt;
                      float out_y = (kp_y * err_y) + (ki_y * integral_yaw) + (kd_y * deriv_y);
                      prev_err_yaw = err_y;

                      // Map to motors (Assuming elevons on myServo1/2 and rudder on motorE)
                      int left_elevon = (int)(out_p + out_r);
                      int right_elevon = (int)(out_p - out_r);
                      int rudder = (int)out_y;

                      // Servo max angle handling or limiting if needed. The angles are in degrees for servo, usually 0-180.
                      // We should map them so that 0 output corresponds to 90 degrees.
                      Servo_SetAngle(&myServo1, 90 + left_elevon);
                      Servo_SetAngle(&myServo2, 90 + right_elevon);
                      Motor_SetSpeed(&motorE, rudder);
                  } else if (current_state == STATE_EMERGENCY) {
                      Motor_SetSpeed(&motorE, 0);
                      Servo_SetAngle(&myServo1, 90); // or neutral position
                      Servo_SetAngle(&myServo2, 90); // or neutral position
                  }
              }
		  // --- ПРЯМОЙ ОПРОС РЕГИСТРОВ SX1278 ---
		  uint8_t irqFlags = LoRa_ReadReg(0x12);

		  if (irqFlags & 0x40) { // RX_DONE
		      LoRa_WriteReg(0x12, irqFlags); // Сбрасываем флаги прерываний

		      if ((irqFlags & 0x20) == 0) { // Нет аппаратной ошибки CRC
		          int pSize = LoRa_ReadReg(0x13);

		          // 1. СТРОГАЯ ПРОВЕРКА РАЗМЕРА: пакет команды должен весить РОВНО 6 байт
		          if (pSize == sizeof(CommandPacket_t)) {
		              LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10)); // Ставим указатель на начало пакета

		              CommandPacket_t rx_cmd;
		              uint8_t *ptr = (uint8_t*)&rx_cmd;

		              // Вычитываем байты из буфера
		              for (int i = 0; i < sizeof(CommandPacket_t); i++) {
		                  ptr[i] = LoRa_ReadReg(0x00);
		              }

		              if (rx_cmd.target_id == MY_NODE_ID) {
		                  // 2. ПРОВЕРКА КОНТРОЛЬНОЙ СУММЫ (отсекает случайный мусор и обрывки телеметрии)
		                  uint8_t calc_crc = 0;
		                  for (int i = 0; i < sizeof(CommandPacket_t) - 1; i++) {
		                      calc_crc ^= ptr[i];
		                  }

		                  if (calc_crc == rx_cmd.checksum) {
		                      char cmd_log[64];
		                      snprintf(cmd_log, sizeof(cmd_log), "RX CMD | ID: 0x%02X, Param: %d", rx_cmd.cmd_id, rx_cmd.param);
		                      Send_Log(cmd_log);

		                      // Управление сервоприводами
		                      // Управление сервоприводами и системные команды
		                      switch (rx_cmd.cmd_id) {
		                          case 0x10:
		                              Servo_SetAngle(&myServo1, rx_cmd.param);
		                              Send_Log("Action: Servo 1 OK");
		                              break;
		                          case 0x11:
		                              Servo_SetAngle(&myServo2, rx_cmd.param);
		                              Send_Log("Action: Servo 2 OK");
		                              break;

		                          // --- ПЕРЕЗАПУСК ДАТЧИКОВ ---
		                          case 0x20: // BMI088
		                              Send_Log("Re-init: BMI088...");
		                              if (BMI088_Init(&hspi1) == HAL_OK) {
		                                  BMI088_CalibrateGyro(&hspi1, 200);
		                                  bmi_ok = true;
		                                  Send_Log("Re-init: BMI088 OK");
		                              } else {
		                                  bmi_ok = false;
		                                  Send_Log("Re-init: BMI088 FAILED!");
		                              }
		                              break;

		                          case 0x21: // LIS3MDL
		                              Send_Log("Re-init: LIS3MDL...");
		                              if (LIS3MDL_Init() == 1) {
		                                  lis_ok = true;
		                                  Send_Log("Re-init: LIS3MDL OK");
		                              } else {
		                                  lis_ok = false;
		                                  Send_Log("Re-init: LIS3MDL FAILED!");
		                              }
		                              break;

		                          case 0x22: // BMP388
		                              Send_Log("Re-init: BMP388...");
		                              if (BMP388_Init() == BMP388_OK) {
		                                  bmp_ok = true;
		                                  Send_Log("Re-init: BMP388 OK");
		                              } else {
		                                  bmp_ok = false;
		                                  Send_Log("Re-init: BMP388 FAILED!");
		                              }
		                              break;

		                          case 0x23: // SD Card
		                              Send_Log("Re-init: SD Card...");
		                              if (SD_MCP_Init(&hspi2) == 0 && SD_MCP_OpenFile("0:/data.csv") == 0) {
		                                  Send_Log("Re-init: SD Card OK");
		                              } else {
		                                  Send_Log("Re-init: SD Card FAILED!");
		                              }
		                              break;

									  // --- КАЛИБРОВКА ДАТЧИКОВ ---
									  case 0x30: // BMI088 (Гироскоп)
										  Send_Log("Calibrating BMI088 Gyro...");
										  // ВАЖНО: Во время калибровки планер должен быть абсолютно неподвижен!
										  BMI088_CalibrateGyro(&hspi1, 200);
										  Send_Log("BMI088 Gyro Calibrated!");
										  break;

									  case 0x33: // BMI088 (Акселерометр)
									          Send_Log("Calibrating BMI088 Accel...");
									          // ВНИМАНИЕ: Планер должен лежать СТРОГО горизонтально на ровном столе!
									          BMI088_CalibrateAccel(&hspi1, 200);
									          Send_Log("BMI088 Accel Calibrated!");
									          break;

									  case 0x31: // LIS3MDL (Магнитометр)
										  Send_Log("Calibrating LIS3MDL Mag...");
										  // Вызываем функцию, которая у вас была закомментирована в инициализации
										  LIS3MDL_Calibrate();
										  Send_Log("LIS3MDL Mag Calibrated!");
										  break;

									  case 0x32: // BMP388 (Установка нуля высоты)
										  Send_Log("Calibrating BMP388 Altitude...");
										  if (bmp_ok) {
											  float temp, press;
											  float press_sum = 0.0f;
											  int samples = 10;

											  for (int i = 0; i < samples; i++) {
												  BMP388_Read(&temp, &press); // Читаем компенсированное давление[cite: 13]
												  press_sum += press;
												  HAL_Delay(50); // Ждем обновления данных датчика
											  }

											  ground_pressure = press_sum / samples; // Сохраняем давление земли

											  char alt_log[64];
											  // Выводим полученное значение в лог (%.2f выводит 2 знака после запятой)
											  snprintf(alt_log, sizeof(alt_log), "BMP388 Zeroed! Base: %d hPa", (int)ground_pressure);
											  Send_Log(alt_log);
										  } else {
											  Send_Log("Error: BMP388 is not ready!");
										  }
										  break;

									  default:
										  {
											  char unk_log[32];
											  snprintf(unk_log, sizeof(unk_log), "Unknown CMD: 0x%02X", rx_cmd.cmd_id);
											  Send_Log(unk_log);
										  }
										  break;
		                      }
		                  }
		              }
		          }
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
			  // Считаем высоту относительно стартовой точки (ground_pressure мы добавили ранее)
						  float rel_alt = BMP388_Altitude(ground_pressure, bmp_press);

						  // Умножаем на 10, чтобы передать десятые доли метра в целом числе (например, 15.2 м -> 152)
						  current_pkt.altitude = (int16_t)(rel_alt * -10.0f);
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

                  snprintf(sd_buffer, sizeof(sd_buffer), "%X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
						 current_pkt.node_id,
						 current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
						 current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
						 current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
						 current_pkt.altitude, current_pkt.temperature, current_pkt.state_flags);

				  SD_MCP_WriteFile("0:/data.csv", sd_buffer);
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

				  Serial_Printf(&huart1, "TX Node:%X | Acc:%d,%d,%d | Gyr:%d,%d,%d | Mag:%d,%d,%d | Alt:%d | Temp:%d | Flags:0x%02X | CRC:0x%02X\r\n",
					   current_pkt.node_id,
					   current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
					   current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
					   current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
					   current_pkt.altitude, current_pkt.temperature, current_pkt.state_flags, current_pkt.checksum);

                  LoRa_BeginPacket(0);
                  LoRa_Write((uint8_t*)&current_pkt, sizeof(TelemetryPacket_t));
                  LoRa_EndPacket(0);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Если прерывание пришло от 15-го пина (PA15, к которому подключен DIO0)
    if (GPIO_Pin == GPIO_PIN_15) {
        lora_irq_triggered = true;
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
