#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"
#include "mcp23017.h"

/* Пины CS на расширителе портов MCP23017 */
#define BMI088_CS_ACC_PIN_MCP     2  // 19 ножка
#define BMI088_CS_GYR_PIN_MCP     4  // 21 ножка

// --- Регистры Акселерометра ---
#define BMI088_ACC_CHIP_ID        0x00
#define BMI088_ACC_DATA           0x12
#define BMI088_ACC_TEMP_DATA      0x22
#define BMI088_ACC_CONF           0x40
#define BMI088_ACC_RANGE          0x41
#define BMI088_ACC_PWR_CONF       0x7C
#define BMI088_ACC_PWR_CTRL       0x7D

// --- Регистры Гироскопа ---
#define BMI088_GYR_CHIP_ID        0x00
#define BMI088_GYR_DATA           0x02
#define BMI088_GYR_RANGE          0x0F
#define BMI088_GYR_BANDWIDTH      0x10

typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp_Raw;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} BMI088_Raw_t;

typedef struct {
    float Accel_X; // g
    float Accel_Y; // g
    float Accel_Z; // g
    float Temp_C;  // °C
    float Gyro_X;  // °/s
    float Gyro_Y;  // °/s
    float Gyro_Z;  // °/s
} BMI088_Physical_t;

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef BMI088_CalibrateGyro(SPI_HandleTypeDef *hspi, uint16_t num_samples);
HAL_StatusTypeDef BMI088_CalibrateAccel(SPI_HandleTypeDef *hspi, uint16_t num_samples);
HAL_StatusTypeDef BMI088_ReadRaw(SPI_HandleTypeDef *hspi, BMI088_Raw_t *raw);
void BMI088_ToPhysical(const BMI088_Raw_t *raw, BMI088_Physical_t *phys);

#endif
