#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

#define MPU6050_I2C_ADDR        (0x68 << 1) // 0xD0 (HAL использует 8-битный адрес)
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_OUT_H     0x3B

typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp_Raw;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU6050_Raw_t;

typedef struct {
    float Accel_X; // g
    float Accel_Y; // g
    float Accel_Z; // g
    float Temp_C;  // °C
    float Gyro_X;  // °/s
    float Gyro_Y;  // °/s
    float Gyro_Z;  // °/s
} MPU6050_Physical_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw);
void MPU6050_ToPhysical(const MPU6050_Raw_t *raw, MPU6050_Physical_t *phys);

#endif
