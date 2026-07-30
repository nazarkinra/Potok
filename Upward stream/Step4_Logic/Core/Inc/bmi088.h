#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"

/*
 * I2C Адреса (сдвинуты на 1 бит влево для HAL).
 * ВАЖНО: Адреса зависят от подключения пинов SDO1 и SDO2 на вашей плате!
 * По умолчанию: Accel SDO1 к GND (0x18), Gyro SDO2 к GND (0x68)
 */
#define BMI088_ACC_I2C_ADDR       (0x18 << 1) // Если SDO1 к 3.3V, то (0x19 << 1)
#define BMI088_GYR_I2C_ADDR       (0x68 << 1) // Если SDO2 к 3.3V, то (0x69 << 1)

// --- Регистры Акселерометра ---
#define BMI088_ACC_CHIP_ID        0x00
#define BMI088_ACC_DATA           0x12 // Начало блока данных (0x12..0x17)
#define BMI088_ACC_TEMP_DATA      0x22 // Данные температуры (0x22..0x23)
#define BMI088_ACC_CONF           0x40 // Конфигурация (частота, фильтры)
#define BMI088_ACC_RANGE          0x41 // Диапазон (±3g, ±6g, ±12g, ±24g)
#define BMI088_ACC_PWR_CONF       0x7C // Управление питанием (Active/Suspend)
#define BMI088_ACC_PWR_CTRL       0x7D // Включение акселерометра

// --- Регистры Гироскопа ---
#define BMI088_GYR_CHIP_ID        0x00
#define BMI088_GYR_DATA           0x02 // Начало блока данных (0x02..0x07)
#define BMI088_GYR_RANGE          0x0F // Диапазон гироскопа
#define BMI088_GYR_BANDWIDTH      0x10 // Настройки фильтра (ODR/Filter)

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

HAL_StatusTypeDef BMI088_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMI088_ReadRaw(I2C_HandleTypeDef *hi2c, BMI088_Raw_t *raw);
void BMI088_ToPhysical(const BMI088_Raw_t *raw, BMI088_Physical_t *phys);

#endif
