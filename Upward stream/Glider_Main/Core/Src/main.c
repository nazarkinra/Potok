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
#include "MadgwickAHRS.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
volatile bool _packetReady = false;
volatile int _pendingLength = 0;
Motor_HandleTypeDef motorA, motorB, motorC, motorD, motorE, motorF;
HMC5883L_Data_t mag_data;

// Madgwick outputs
float roll = 0, pitch = 0, yaw = 0;

typedef enum { STATE_WAIT, STATE_FLIGHT, STATE_EMERGENCY } State_t;
State_t current_state = STATE_WAIT;

float target_roll = 0, target_pitch = -5.0f, target_yaw = 0;
float prev_err_roll = 0, prev_err_pitch = 0, prev_err_yaw = 0;
float integral_roll = 0, integral_pitch = 0, integral_yaw = 0;

// PID constants (dummy values, need tuning)
float kp_r = 1.0f, ki_r = 0.01f, kd_r = 0.5f;
float kp_p = 1.0f, ki_p = 0.01f, kd_p = 0.5f;
float kp_y = 1.0f, ki_y = 0.01f, kd_y = 0.5f;
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

// Non-blocking motor stop logic
uint32_t motor_stop_time = 0;
bool motor_active = false;

void stop_motors() {
    Motor_SetSpeed(&motorB, 0);
    Motor_SetSpeed(&motorD, 0);
    Motor_SetSpeed(&motorA, 0);
    Motor_SetSpeed(&motorC, 0);
    motor_active = false;
}

void move(int m_left, int m_right, int m_mid, int duration) {
    // motorB - правая сторона, motorD - левая сторона, как в LoRa278
    Motor_SetSpeed(&motorB, m_right);
    Motor_SetSpeed(&motorD, m_left);
    Motor_SetSpeed(&motorA, m_mid);
    Motor_SetSpeed(&motorC, m_mid);

    if (duration > 0) {
        motor_stop_time = HAL_GetTick() + duration;
        motor_active = true;
    } else {
        motor_active = false;
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
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  Serial_Printf(&huart1, "\r\n--- STM32 Test1 INIT ---\r\n");

  // Init Motors via PCA9685
  PCA9685_Init(1000);
  Motor_Init(&motorD, 0, 1);
  Motor_Init(&motorC, 2, 3);
  Motor_Init(&motorB, 4, 5);
  Motor_Init(&motorA, 6, 7);
  Motor_Init(&motorF, 11, 12);
  Motor_Init(&motorE, 13, 14);

  // Init LoRa
  if (!LoRa_Begin(433000000)) {
	  Serial_Printf(&huart1, "LoRa init failed!\r\n");
  } else {
      LoRa_SetSpreadingFactor(7);
      LoRa_SetSignalBandwidth(125000);
      LoRa_SetCodingRate4(5);
      LoRa_EnableCrc();
      LoRa_SetSyncWord(0x12);
      LoRa_SetTxPower(17, 1);

      LoRa_OnReceive(OnLoRaPacketReceived);
      LoRa_Receive(0);
      Serial_Printf(&huart1, "LoRa Ready!\r\n");
  }

  if (MPU6050_Init(&hi2c1) != HAL_OK) {
      if (MPU6050_Init(&hi2c3) != HAL_OK) {
          Serial_Printf(&huart1, "MPU6050 init failed!\r\n");
      }
  }

  if (HMC5883L_Init(&hi2c1) != HAL_OK) {
       if (HMC5883L_Init(&hi2c3) != HAL_OK) {
           Serial_Printf(&huart1, "HMC5883L init failed!\r\n");
       }
  }

  uint32_t last_telemetry_tick = HAL_GetTick();
  uint32_t last_pid_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char uart_buf[128];
  char command[50];
  MPU6050_Raw_t raw;
  MPU6050_Physical_t phys;

  while (1)
  {
      if (HAL_GetTick() - last_pid_tick >= 10) {
          last_pid_tick = HAL_GetTick();

          // Separation simulation
          int adc_sim = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET ? 1 : 0;
          if (current_state == STATE_WAIT && adc_sim == 1) {
              current_state = STATE_FLIGHT;
          }

          if (current_state == STATE_FLIGHT) {
              float dt = 0.01f;

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

              // Map to motors (assuming elevons on motorA, motorB and rudder on motorC)
              int left_elevon = (int)(out_p + out_r);
              int right_elevon = (int)(out_p - out_r);
              int rudder = (int)out_y;

              Motor_SetSpeed(&motorA, left_elevon);
              Motor_SetSpeed(&motorB, right_elevon);
              Motor_SetSpeed(&motorC, rudder);
          } else if (current_state == STATE_EMERGENCY) {
              stop_motors();
          }
      }

      if (_packetReady) {
          _packetReady = false;
          int i = 0;

          while (LoRa_Available() && i < sizeof(command) - 1) {
              char c = (char)LoRa_Read();
              command[i] = c;
              i++;
          }
          command[i] = '\0';

          Serial_Printf(&huart1, "RX CMD: %s\r\n", command);

          char direction[20];
          int speed = 0;
          int duration = 0;

          if (sscanf(command, "%19s %d %d", direction, &speed, &duration) == 3) {
              if (speed > 100) speed = 100;
              if (speed < -100) speed = -100;
              if (duration > 5000) duration = 5000;
              if (duration < 0) duration = 0;

              if (strcmp(direction, "right") == 0) {
                  move(0, speed, 0, duration);
              }
              else if (strcmp(direction, "left") == 0) {
                  move(speed, 0, 0, duration);
              }
              else if (strcmp(direction, "forward") == 0) {
                  move(speed, speed, speed, duration);
              }
              else if (strcmp(direction, "backward") == 0) {
                  move(-speed, -speed, -speed, duration);
              }
              else if (strcmp(direction, "stop") == 0) {
                  move(0, 0, 0, 0);
              }
          }
      }

      // Non-blocking motor stop check
      if (motor_active && HAL_GetTick() >= motor_stop_time) {
          stop_motors();
      }

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

              // Note: axes may need adjustment depending on sensor mounting
              MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

              // Calculate angles from quaternions
              roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
              pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
              yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f;
          }

          if (HMC5883L_ReadData(&hi2c1, &mag_data) == HAL_OK || HMC5883L_ReadData(&hi2c3, &mag_data) == HAL_OK) {
              mag_x = mag_data.x * 0.92f;
              mag_y = mag_data.y * 0.92f;
              mag_z = mag_data.z * 0.92f;
          }

          // Avoid float printf due to GCC default config on STM32
          int r = (int)(roll * 100), p = (int)(pitch * 100), y = (int)(yaw * 100);

          sprintf(uart_buf, "ST:%d ROLL:%d.%02d PITCH:%d.%02d YAW:%d.%02d",
                   current_state, r/100, abs(r)%100, p/100, abs(p)%100, y/100, abs(y)%100);

          // Send via LoRa
          LoRa_BeginPacket(0); // 0 = explicit header
          LoRa_Write((uint8_t*)uart_buf, strlen(uart_buf));
          LoRa_EndPacket(0);   // wait for tx

          // switch back to receive mode
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
    if (GPIO_Pin == DIO0_RADIO_Pin) {
        LoRa_HandleDio0Rise();
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
