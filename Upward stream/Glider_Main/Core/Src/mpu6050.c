#include "mpu6050.h"

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;

    // 1. Пробуждение чипа
    status = MPU6050_WriteReg(hi2c, MPU6050_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // 2. Делитель частоты (внутренний 1 кГц / 8 = 125 Гц)
    status = MPU6050_WriteReg(hi2c, MPU6050_SMPLRT_DIV, 7);
    if (status != HAL_OK) return status;

    // 3. DLPF 44 Гц (снижает шум)
    status = MPU6050_WriteReg(hi2c, MPU6050_CONFIG, 0x03);
    if (status != HAL_OK) return status;

    // 4. Гироскоп ±250 °/s, Акселерометр ±2 g
    status = MPU6050_WriteReg(hi2c, MPU6050_GYRO_CONFIG, 0x00);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteReg(hi2c, MPU6050_ACCEL_CONFIG, 0x00);

    return status;
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw) {
    uint8_t data[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_ACCEL_OUT_H,
                                                I2C_MEMADD_SIZE_8BIT, data, 14, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    raw->Accel_X = (int16_t)((data[0] << 8) | data[1]);
    raw->Accel_Y = (int16_t)((data[2] << 8) | data[3]);
    raw->Accel_Z = (int16_t)((data[4] << 8) | data[5]);
    raw->Temp_Raw = (int16_t)((data[6] << 8) | data[7]);
    raw->Gyro_X = (int16_t)((data[8] << 8) | data[9]);
    raw->Gyro_Y = (int16_t)((data[10] << 8) | data[11]);
    raw->Gyro_Z = (int16_t)((data[12] << 8) | data[13]);

    return HAL_OK;
}

void MPU6050_ToPhysical(const MPU6050_Raw_t *raw, MPU6050_Physical_t *phys) {
    // Масштабирование для ±2g / ±250°/s
    phys->Accel_X = raw->Accel_X / 16384.0f;
    phys->Accel_Y = raw->Accel_Y / 16384.0f;
    phys->Accel_Z = raw->Accel_Z / 16384.0f;

    phys->Gyro_X = raw->Gyro_X / 131.0f;
    phys->Gyro_Y = raw->Gyro_Y / 131.0f;
    phys->Gyro_Z = raw->Gyro_Z / 131.0f;

    // Температура: T(°C) = raw/340 + 36.53
    phys->Temp_C = (raw->Temp_Raw / 340.0f) + 36.53f;
}
