#include "bmi088.h"

// Вспомогательная функция для записи, теперь принимает адрес устройства, так как их два
static HAL_StatusTypeDef BMI088_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMI088_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;

    // ==========================================
    // 1. Инициализация Акселерометра
    // ==========================================

    // Перевод питания акселерометра в Active mode
    status = BMI088_WriteReg(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CONF, 0x00);
    if (status != HAL_OK) return status;
    HAL_Delay(10); // Ожидание включения логики

    // Включение самого акселерометра
    status = BMI088_WriteReg(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CTRL, 0x04);
    if (status != HAL_OK) return status;
    HAL_Delay(50); // Ждем стабилизации MEMS структуры

    // Настройка: Normal mode, OSR4, 100Hz (встроенный ФНЧ)
    status = BMI088_WriteReg(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_CONF, 0xA8);
    if (status != HAL_OK) return status;

    // Диапазон: ±6g (значение 0x01)
    status = BMI088_WriteReg(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_RANGE, 0x01);
    if (status != HAL_OK) return status;

    // ==========================================
    // 2. Инициализация Гироскопа
    // ==========================================

    // Диапазон: ±250 °/s (значение 0x03) для совместимости с логикой прошлого кода
    status = BMI088_WriteReg(hi2c, BMI088_GYR_I2C_ADDR, BMI088_GYR_RANGE, 0x03);
    if (status != HAL_OK) return status;

    // Пропускная способность: OSR4, 116Hz
    status = BMI088_WriteReg(hi2c, BMI088_GYR_I2C_ADDR, BMI088_GYR_BANDWIDTH, 0x02);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadRaw(I2C_HandleTypeDef *hi2c, BMI088_Raw_t *raw) {
    uint8_t acc_data[6];
    uint8_t gyr_data[6];
    uint8_t temp_data[2];
    HAL_StatusTypeDef status;

    // Чтение данных акселерометра
    status = HAL_I2C_Mem_Read(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_DATA,
                              I2C_MEMADD_SIZE_8BIT, acc_data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Чтение данных гироскопа
    status = HAL_I2C_Mem_Read(hi2c, BMI088_GYR_I2C_ADDR, BMI088_GYR_DATA,
                              I2C_MEMADD_SIZE_8BIT, gyr_data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Чтение температуры
    status = HAL_I2C_Mem_Read(hi2c, BMI088_ACC_I2C_ADDR, BMI088_ACC_TEMP_DATA,
                              I2C_MEMADD_SIZE_8BIT, temp_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // ВНИМАНИЕ: BMI088 передает сначала LSB (младший байт), затем MSB (старший байт)!
    raw->Accel_X = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    raw->Accel_Y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    raw->Accel_Z = (int16_t)((acc_data[5] << 8) | acc_data[4]);

    raw->Gyro_X  = (int16_t)((gyr_data[1] << 8) | gyr_data[0]);
    raw->Gyro_Y  = (int16_t)((gyr_data[3] << 8) | gyr_data[2]);
    raw->Gyro_Z  = (int16_t)((gyr_data[5] << 8) | gyr_data[4]);

    // Температура у BMI088 занимает 11 бит:
    // temp_data[0] (0x22) содержит старшие 8 бит.
    // temp_data[1] (0x23) содержит младшие 3 бита со смещением << 5.
    uint16_t temp_uint = (temp_data[0] << 3) | (temp_data[1] >> 5);
    if (temp_uint > 1023) { // Обработка отрицательных температур (знаковое 11-битное число)
        raw->Temp_Raw = (int16_t)temp_uint - 2048;
    } else {
        raw->Temp_Raw = (int16_t)temp_uint;
    }

    return HAL_OK;
}

void BMI088_ToPhysical(const BMI088_Raw_t *raw, BMI088_Physical_t *phys) {
    // Масштабирование акселерометра для ±6g
    // 16 бит (от -32768 до +32767). 32768 / 6 = 5461.33 LSB/g
    phys->Accel_X = raw->Accel_X / 5461.33f;
    phys->Accel_Y = raw->Accel_Y / 5461.33f;
    phys->Accel_Z = raw->Accel_Z / 5461.33f;

    // Масштабирование гироскопа для ±250 °/s
    // 32768 / 250 = 131.072 LSB/°/s (почти как у MPU6050)
    phys->Gyro_X = raw->Gyro_X / 131.072f;
    phys->Gyro_Y = raw->Gyro_Y / 131.072f;
    phys->Gyro_Z = raw->Gyro_Z / 131.072f;

    // Конвертация температуры по даташиту: T(°C) = (Raw * 0.125) + 23
    phys->Temp_C = (raw->Temp_Raw * 0.125f) + 23.0f;
}
