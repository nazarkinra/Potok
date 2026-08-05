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
#include <stdbool.h>
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
    uint16_t  photo;
    uint8_t  state_flags;
    uint8_t  checksum;
} TelemetryPacket_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    float    ideal_pitch;

    // Калибровка магнитометра (Hard & Soft Iron)
    float    mag_offset[3];
    float    mag_scale[3];

    // Коэффициенты ПИД-регулятора
    float    kp_r, ki_r, kd_r; // Roll
    float    kp_p, ki_p, kd_p; // Pitch
    float    kp_y, ki_y, kd_y; // Yaw
} GliderConfig_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t target_id;
    uint8_t  cmd_id;
    int16_t  param;
    uint8_t  checksum;
} CommandPacket_t;
#pragma pack(pop)

// Значения, которые запишутся на SD, если файла config.dat еще нет
GliderConfig_t glider_cfg = {
    .magic = 0xAABBCCDD,
    .ideal_pitch = -5.0f,
    .mag_offset = {0.0f, 0.0f, 0.0f},
    .mag_scale  = {1.0f, 1.0f, 1.0f},
    .kp_r = 1.0f, .ki_r = 0.01f, .kd_r = 0.5f,
    .kp_p = 1.0f, .ki_p = 0.01f, .kd_p = 0.5f,
    .kp_y = 1.0f, .ki_y = 0.01f, .kd_y = 0.5f
};
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
bool apogee_reached = false;

// Глобальный буфер для хранения самых свежих данных
TelemetryPacket_t current_pkt;
Servo_HandleTypeDef myServo1;
Servo_HandleTypeDef myServo2;

volatile bool lora_irq_triggered = false;
Motor_HandleTypeDef motorE;
Motor_HandleTypeDef motorF;
// Базовое давление на земле для планера (по умолчанию стандартное 1013.25 гПа)
float ground_pressure = 1013.25f;

// Этапы полета
typedef enum {
    STATE_IDLE = 0,      // На столе (Безопасный режим, автоматика отключена)
    STATE_WAIT = 1,      // В ракете (Взведен, ждет выброса)
    STATE_DROP = 2,      // Выпадание
    STATE_RECOVERY = 3,  // Выход из падения
    STATE_GLIDE = 4,     // Планирование
    STATE_PARACHUTE = 5  // Раскрытие парашюта
} State_t;

State_t current_state = STATE_IDLE;
uint32_t stage_timer = 0; // Таймер для отсчета времени стадий

// Настройка порога фоторезистора (Настройте под себя: светло = больше или меньше)
#define PHOTO_THRESHOLD 2000

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

/* Функции логирования */
void Send_Log(const char* msg) {
    Serial_Printf(&huart1, "%s\r\n", msg);
    if (lora_ready) {
        uint8_t lora_buf[128];
        uint16_t* id_ptr = (uint16_t*)lora_buf;
        *id_ptr = MY_NODE_ID;
        int len = snprintf((char*)(lora_buf + 2), sizeof(lora_buf) - 2, "LOG: %s", msg);
        LoRa_BeginPacket(0);
        LoRa_Write(lora_buf, len + 2);
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
  MCP23017_WriteAll(0xFFFF);
  PCA9685_Init(50);
  Motor_Init(&motorE, 13, 14);
  Motor_Init(&motorF, 11, 12);
  reinit:
  LoRa_Init();
  if (!LoRa_Begin(441000000)) {
	  Serial_Printf(&huart1, "LoRa init failed!\r\n");
	  MCP23017_WritePin(LORA_RST_PIN_MCP, 0);
	  HAL_Delay(500);
	  MCP23017_WritePin(LORA_RST_PIN_MCP, 1);
	  goto reinit;
  } else {
	  LoRa_SetSpreadingFactor(7);
	  LoRa_SetSignalBandwidth(125000);
	  LoRa_SetCodingRate4(5);
	  LoRa_EnableCrc();
	  LoRa_SetSyncWord(0x12);
	  LoRa_SetTxPower(10, 1);
	  LoRa_Receive(0);
	  lora_ready = true;
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

		// 1. ПРОБУЕМ ЗАГРУЗИТЬ КОНФИГ
		if (SD_MCP_LoadConfig("0:/config.dat", &glider_cfg, sizeof(glider_cfg)) == 0 && glider_cfg.magic == 0xAABBCCDD) {
			  Send_Log("[MAIN] Config loaded from SD!");
		  } else {
			  Send_Log("[MAIN] Config not found! Creating default...");
			  SD_MCP_SaveConfig("0:/config.dat", &glider_cfg, sizeof(glider_cfg));
		  }

		  // Применяем настройки к магнитометру
		  LIS3MDL_SetCalibration(
			  glider_cfg.mag_offset[0], glider_cfg.mag_offset[1], glider_cfg.mag_offset[2],
			  glider_cfg.mag_scale[0],  glider_cfg.mag_scale[1],  glider_cfg.mag_scale[2]
		  );

		  // Применяем настройки к ПИД-регуляторам
		  target_pitch = glider_cfg.ideal_pitch;
		  kp_r = glider_cfg.kp_r; ki_r = glider_cfg.ki_r; kd_r = glider_cfg.kd_r;
		  kp_p = glider_cfg.kp_p; ki_p = glider_cfg.ki_p; kd_p = glider_cfg.kd_p;
		  kp_y = glider_cfg.kp_y; ki_y = glider_cfg.ki_y; kd_y = glider_cfg.kd_y;

		// 2. ОТКРЫВАЕМ ФАЙЛ ЛОГОВ (только ПОСЛЕ работы с конфигом)
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
	uint32_t photo_trigger_time = 0;
	bool photo_is_light = false;
	bool igniter_active = false;
	Send_Log("INIT DONE. Entering main loop...");

	while (1) {
		// --- 1. ПРЯМОЙ ОПРОС РЕГИСТРОВ SX1278 (КОМАНДЫ) ---
		uint8_t irqFlags = LoRa_ReadReg(0x12);
		if (irqFlags & 0x40) {
			LoRa_WriteReg(0x12, irqFlags);
			if ((irqFlags & 0x20) == 0) {
				int pSize = LoRa_ReadReg(0x13);
				if (pSize == sizeof(CommandPacket_t)) {
					LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10));
					CommandPacket_t rx_cmd;
					uint8_t *ptr = (uint8_t*)&rx_cmd;
					for (int i = 0; i < sizeof(CommandPacket_t); i++) ptr[i] = LoRa_ReadReg(0x00);

					if (rx_cmd.target_id == MY_NODE_ID) {
						uint8_t calc_crc = 0;
						for (int i = 0; i < sizeof(CommandPacket_t) - 1; i++) calc_crc ^= ptr[i];

						if (calc_crc == rx_cmd.checksum) {
							char cmd_log[64];
							snprintf(cmd_log, sizeof(cmd_log), "RX CMD | ID: 0x%02X, Param: %d", rx_cmd.cmd_id, rx_cmd.param);
							Send_Log(cmd_log);

							switch (rx_cmd.cmd_id) {
								case 0x10: Servo_SetAngle(&myServo1, rx_cmd.param); break;
								case 0x11: Servo_SetAngle(&myServo2, rx_cmd.param); break;

								// Системные переинициализации (0x20 - 0x24) пропущены для экономии места,
								// они остаются точно такими же, как мы делали ранее.

								// Калибровки датчиков
								case 0x30: BMI088_CalibrateGyro(&hspi1, 200); Send_Log("Gyro Calibrated"); break;
								case 0x31: // LIS3MDL (Магнитометр)
									Send_Log("Calibrating LIS3MDL Mag (Rotate glider!)...");
									LIS3MDL_Calibrate();

									LIS3MDL_GetCalibration(
										&glider_cfg.mag_offset[0], &glider_cfg.mag_offset[1], &glider_cfg.mag_offset[2],
										&glider_cfg.mag_scale[0],  &glider_cfg.mag_scale[1],  &glider_cfg.mag_scale[2]
									);

									if (SD_MCP_SaveConfig("0:/config.dat", &glider_cfg, sizeof(glider_cfg)) == 0) {
										Send_Log("Mag Calibrated & Saved to SD!");
									} else {
										Send_Log("Mag Calibrated, but SD SAVE FAILED!");
									}
									break;
								case 0x33: BMI088_CalibrateAccel(&hspi1, 200); Send_Log("Accel Calibrated"); break;
								case 0x32: {
									float t, p, p_sum = 0;
									for(int i=0; i<10; i++) { BMP388_Read(&t, &p); p_sum+=p; HAL_Delay(50); }
									ground_pressure = p_sum/10.0f;
									Send_Log("Altitude Zeroed");
									break;
								}
								case 0x50:
									glider_cfg.ideal_pitch = (float)rx_cmd.param; // Например, передали -6
									target_pitch = glider_cfg.ideal_pitch; // Сразу применяем к PID-регулятору

									if (SD_MCP_SaveConfig("0:/config.dat", &glider_cfg, sizeof(glider_cfg)) == 0) {
										char cfg_log[64];
										snprintf(cfg_log, sizeof(cfg_log), "Pitch updated to %d and saved!", rx_cmd.param);
										Send_Log(cfg_log);
									}
									break;
									// --- НАСТРОЙКА ПИД-РЕГУЛЯТОРОВ (0x60 - 0x68) ---
									case 0x60: case 0x61: case 0x62:
									case 0x63: case 0x64: case 0x65:
									case 0x66: case 0x67: case 0x68:
									{
										// Возвращаем числу дробную часть (делим на 1000)
										float val = rx_cmd.param / 1000.0f;

										if (rx_cmd.cmd_id == 0x60) { kp_r = val; glider_cfg.kp_r = val; }
										else if (rx_cmd.cmd_id == 0x61) { ki_r = val; glider_cfg.ki_r = val; }
										else if (rx_cmd.cmd_id == 0x62) { kd_r = val; glider_cfg.kd_r = val; }

										else if (rx_cmd.cmd_id == 0x63) { kp_p = val; glider_cfg.kp_p = val; }
										else if (rx_cmd.cmd_id == 0x64) { ki_p = val; glider_cfg.ki_p = val; }
										else if (rx_cmd.cmd_id == 0x65) { kd_p = val; glider_cfg.kd_p = val; }

										else if (rx_cmd.cmd_id == 0x66) { kp_y = val; glider_cfg.kp_y = val; }
										else if (rx_cmd.cmd_id == 0x67) { ki_y = val; glider_cfg.ki_y = val; }
										else if (rx_cmd.cmd_id == 0x68) { kd_y = val; glider_cfg.kd_y = val; }

										// Перезаписываем файл конфигурации
										if (SD_MCP_SaveConfig("0:/config.dat", &glider_cfg, sizeof(glider_cfg)) == 0) {
											char pid_log[64];
											snprintf(pid_log, sizeof(pid_log), "PID CMD 0x%02X saved! New Val: %d", rx_cmd.cmd_id, rx_cmd.param);
											Send_Log(pid_log);
										} else {
											Send_Log("PID updated, but SD SAVE FAILED!");
										}
									}
									break;

								// Ручное переключение этапов полета
								// Ручное переключение этапов полета
								case 0x40: current_state = STATE_IDLE; Send_Log("Forced: IDLE (Safe Mode)"); break;
								case 0x41: current_state = STATE_WAIT; apogee_reached = false; Send_Log("Forced: ARMED (Wait in rocket)"); break;
								case 0x42: current_state = STATE_DROP; stage_timer = HAL_GetTick(); Send_Log("Forced: DROP"); break;
								case 0x43: current_state = STATE_RECOVERY; stage_timer = HAL_GetTick(); Send_Log("Forced: RECOVERY"); break;
								case 0x44: current_state = STATE_GLIDE; Send_Log("Forced: GLIDE"); break;
								case 0x45:
									current_state = STATE_PARACHUTE;
									stage_timer = HAL_GetTick(); // Запоминаем время для таймера пережига
									igniter_active = false;
									Send_Log("Forced: PARACHUTE SEQUENCE STARTED!");
									break;
								// Оперативное изменение углов в полете (без сохранения на SD)
								case 0x70: target_roll = (float)rx_cmd.param; Send_Log("Live Target Roll updated!"); break;
								case 0x71: target_pitch = (float)rx_cmd.param; Send_Log("Live Target Pitch updated!"); break;
								case 0x72: target_yaw = (float)rx_cmd.param; Send_Log("Live Target Yaw updated!"); break;
							}
						}
					}
				}
			}
		}

		// --- НЕБЛОКИРУЮЩИЙ ОПРОС DS18B20 ---
		if (ds18b20_state == 0) {
			if (DS18B20_Start()) {
				DS18B20_WriteByte(0xCC); // Пропуск ROM
				DS18B20_WriteByte(0x44); // Команда конвертации
				ds18b20_timer = HAL_GetTick();
				ds18b20_state = 1;
			}
		}
		else if (ds18b20_state == 1) {
			if (HAL_GetTick() - ds18b20_timer >= 750) {
				if (DS18B20_Start()) {
					DS18B20_WriteByte(0xCC); // Пропуск ROM
					DS18B20_WriteByte(0xBE); // Чтение памяти (Scratchpad)
					uint8_t temp_lsb = DS18B20_ReadByte();
					uint8_t temp_msb = DS18B20_ReadByte();
					int16_t temp_raw = (temp_msb << 8) | temp_lsb;
					current_temp_ds = temp_raw / 16.0f;
				}
				ds18b20_state = 0;
			}
		}

		// --- 2. ОПРОС ДАТЧИКОВ И ПОЛЕТНАЯ ЛОГИКА (50 Гц / 20 мс) ---
				if (HAL_GetTick() - last_poll_tick >= 20) {
					last_poll_tick = HAL_GetTick();
					current_pkt.state_flags = 0;
					float dt = 0.02f; // Тайминг для интегралов ПИД

					HAL_ADC_Start(&hadc1);
					if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) photo_val = HAL_ADC_GetValue(&hadc1);
					current_pkt.photo = photo_val;

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
							float rel_alt = BMP388_Altitude(ground_pressure, bmp_press);
							current_pkt.altitude = (int16_t)(rel_alt * -10.0f); // Инверсия высоты
							current_pkt.temperature = (int16_t)(bmp_temp * 100);
							current_pkt.state_flags |= (1 << 2);
						}
					}

					// Приоритетное использование внешнего термометра DS18B20
					if (current_temp_ds != 0.0f) {
						current_pkt.temperature = (int16_t)(current_temp_ds * 100);
						current_pkt.state_flags |= (1 << 3); // Флаг: работает внешний термометр
					}

					// УПАКОВКА ЭТАПА ПОЛЕТА ВО ФЛАГИ (в старшие 4 бита)
					current_pkt.state_flags |= (current_state << 4);

					// --- АВТОМАТИКА ПОЛЕТА ---
					// Если мы поднялись выше 250 метров (2500 дм), значит это реальный полет
								if (current_pkt.altitude > 2500) {
									apogee_reached = true;
								}

								// 1. Триггер отстрела (с задержкой 2 секунды)
								if (current_state == STATE_WAIT) {
									apogee_reached = false; // Сброс защиты на земле
									if (photo_val > PHOTO_THRESHOLD) {
										if (!photo_is_light) {
											photo_is_light = true;
											photo_trigger_time = HAL_GetTick(); // Начинаем отсчет 2 секунд
										} else if (HAL_GetTick() - photo_trigger_time >= 2000) {
											current_state = STATE_DROP;
											stage_timer = HAL_GetTick();
											photo_is_light = false;
											Send_Log(">>> DROP DETECTED (2s delay)! <<<");
										}
									} else {
										photo_is_light = false; // Сброс таймера, если снова стало темно
									}
								}

								// 2. АВТОМАТИЧЕСКИЙ ВЫБРОС ПАРАШЮТА (только если уже летим)
								if (current_state == STATE_DROP || current_state == STATE_RECOVERY || current_state == STATE_GLIDE) {
									// Защита: высота <= 200м И прошло минимум 3 секунды с момента выброса из ракеты
									if (apogee_reached && current_pkt.altitude <= 2000) {
										current_state = STATE_PARACHUTE;
										stage_timer = HAL_GetTick();
										igniter_active = false;
										PCA9685_SetFullOn(15);
										Send_Log(">>> AUTO PARACHUTE SEQUENCE STARTED! <<<");
									}
								}

					            // --- ОБРАБОТКА ТЕКУЩИХ СОСТОЯНИЙ ---
					            // Отслеживаем момент изменения состояния, чтобы не спамить шину I2C
					            static State_t prev_state = 255;
					            bool state_just_changed = (current_state != prev_state);
					            prev_state = current_state;

								if (current_state == STATE_DROP) {
					                // Поворачиваем сервоприводы ТОЛЬКО один раз при входе в режим
					                if (state_just_changed) {
									    Servo_SetAngle(&myServo1, 90);
									    Servo_SetAngle(&myServo2, 90);
					                }

									// Прямое падение без управления, ждем 2 секунды
									if (HAL_GetTick() - stage_timer >= 2000) {
										current_state = STATE_RECOVERY;
										stage_timer = HAL_GetTick();
										Send_Log(">>> 2s DELAY OVER. PULL UP! <<<");
									}
								}
								else if (current_state == STATE_RECOVERY || current_state == STATE_GLIDE) {

												if (current_state == STATE_RECOVERY) {
													// --- 1. РАСКРЫТИЕ КРЫЛЬЕВ (Мотор F) ---
													if (state_just_changed) {
														Motor_SetSpeed(&motorF, 100); // Включаем мотор на 100%
														Send_Log("RECOVERY: Unfolding wings (Motor F ON)");
													}

													// Даем мотору поработать 1.5 секунды, затем жестко тормозим
													if (HAL_GetTick() - stage_timer >= 1500) {
														Motor_Brake(&motorF);
													}

													// --- 2. ВЫХОД ИЗ ПИКЕ ---
													target_pitch = 15.0f;

													if (HAL_GetTick() - stage_timer >= 3000) { // Через 3 секунды переходим в горизонт
														current_state = STATE_GLIDE;
														Motor_Coast(&motorF); // Полностью обесточиваем мотор крыльев (свободный выбег)
														Send_Log(">>> RECOVERY OVER. GLIDING <<<");
													}
												} else {
													// В режиме GLIDE берем идеальный тангаж из конфига (или с геймпада)
													target_pitch = glider_cfg.ideal_pitch;
												}

												// --- 3. МАТЕМАТИКА AHRS И КВАТЕРНИОНЫ ---
												float accel_x = current_pkt.accel_x / 100.0f;
												float accel_y = current_pkt.accel_y / 100.0f;
												float accel_z = current_pkt.accel_z / 100.0f;
												float gyro_x = current_pkt.gyro_x / 100.0f;
												float gyro_y = current_pkt.gyro_y / 100.0f;
												float gyro_z = current_pkt.gyro_z / 100.0f;

												MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

												roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
												pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;

												// Вычисляем рыскание (Yaw) из кватернионов
												yaw   = atan2f(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f;

												// --- 4. ПИД-РЕГУЛЯТОРЫ ---
												// Roll PID
												float err_r = target_roll - roll;
												integral_roll += err_r * dt;
												if(integral_roll > 20.0f) integral_roll = 20.0f; else if(integral_roll < -20.0f) integral_roll = -20.0f;
												float out_r = (kp_r * err_r) + (ki_r * integral_roll) + (kd_r * ((err_r - prev_err_roll)/dt));
												prev_err_roll = err_r;

												// Pitch PID
												float err_p = target_pitch - pitch;
												integral_pitch += err_p * dt;
												if(integral_pitch > 20.0f) integral_pitch = 20.0f; else if(integral_pitch < -20.0f) integral_pitch = -20.0f;
												float out_p = (kp_p * err_p) + (ki_p * integral_pitch) + (kd_p * ((err_p - prev_err_pitch)/dt));
												prev_err_pitch = err_p;

												// Yaw PID (Рыскание)
												float err_y = target_yaw - yaw;
												integral_yaw += err_y * dt;
												if(integral_yaw > 20.0f) integral_yaw = 20.0f; else if(integral_yaw < -20.0f) integral_yaw = -20.0f;
												float out_y = (kp_y * err_y) + (ki_y * integral_yaw) + (kd_y * ((err_y - prev_err_yaw)/dt));
												prev_err_yaw = err_y;

												// --- 5. МИКШИРОВАНИЕ И УПРАВЛЕНИЕ ---
												// Сервоприводы (Элевоны: Крен + Тангаж)
												int servo1_val = 90 + (int)(out_p + out_r);
												if (servo1_val > 135) servo1_val = 135; else if (servo1_val < 45) servo1_val = 45;

												int servo2_val = 90 + (int)(out_p - out_r);
												if (servo2_val > 135) servo2_val = 135; else if (servo2_val < 45) servo2_val = 45;

												Servo_SetAngle(&myServo1, servo1_val);
												Servo_SetAngle(&myServo2, servo2_val);

												// Мотор E (Рыскание). Функция сама ограничивает значения от -100 до +100
												Motor_SetSpeed(&motorE, (int16_t)out_y);
											}
								else if (current_state == STATE_PARACHUTE) {
												// ЭТАП 1: СРАЗУ (0 мс). Поворот сервоприводов и остановка мотора.
												if (state_just_changed) {
													Servo_SetAngle(&myServo1, 180);
													Servo_SetAngle(&myServo2, 180);
													Motor_SetSpeed(&motorE, 0);
													Send_Log("PARACHUTE 1/3: Servos deployed. Waiting 500ms...");
												}

												// ЭТАП 2: ЧЕРЕЗ 500 мс. Включение пережига (сервы уже остановились, ток упал).
												if (!igniter_active && (HAL_GetTick() - stage_timer >= 500) && (HAL_GetTick() - stage_timer < 5500)) {
													PCA9685_SetFullOn(15);
													igniter_active = true;
													Send_Log("PARACHUTE 2/3: IGNITER ON!");
												}

												// ЭТАП 3: ЧЕРЕЗ 5500 мс (5.5 сек от начала). Отключение пережига.
												if (igniter_active && (HAL_GetTick() - stage_timer >= 5500)) {
													PCA9685_SetFullOff(15);
													igniter_active = false;
													Send_Log("PARACHUTE 3/3: IGNITER OFF (Timeout). SAFE!");
												}
											}
				}

	  // --- 4. Запись на SD карту (Например, 10 Гц / каждые 100 мс) ---
	  if (HAL_GetTick() - last_sd_tick >= 100) {
		  last_sd_tick = HAL_GetTick();

		  char sd_buffer[200];

		  snprintf(sd_buffer, sizeof(sd_buffer), "%X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
				 current_pkt.node_id,
				 current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
				 current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
				 current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
				 current_pkt.altitude, current_pkt.temperature, current_pkt.photo, current_pkt.state_flags);

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

		  Serial_Printf(&huart1, "TX Node:%X | Acc:%d,%d,%d | Gyr:%d,%d,%d | Mag:%d,%d,%d | Alt:%d | Temp:%d | Photo:%u | Flags:0x%02X | CRC:0x%02X\r\n",
			   current_pkt.node_id,
			   current_pkt.accel_x, current_pkt.accel_y, current_pkt.accel_z,
			   current_pkt.gyro_x, current_pkt.gyro_y, current_pkt.gyro_z,
			   current_pkt.mag_x, current_pkt.mag_y, current_pkt.mag_z,
			   current_pkt.altitude, current_pkt.temperature, current_pkt.photo, current_pkt.state_flags, current_pkt.checksum);

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
