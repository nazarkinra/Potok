#include "sht40.h"
#include "i2c.h"  // Подключаем глобальный хэндл I2C из сгенерированного кода CubeMX
#include <math.h>

extern I2C_HandleTypeDef hi2c3; // Хэндл для I2C3 (настроен на PA8/PB4)

// Полином для CRC-8 (x^8 + x^5 + x^4 + 1)
#define CRC8_POLYNOMIAL 0x31

/**
 * @brief Расчет CRC-8 для проверки целостности данных.
 * @param data Указатель на массив данных для расчета.
 * @param len Длина массива данных.
 * @return Вычисленное значение CRC.
 */
static uint8_t SHT40_CalcCrc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF; // Начальное значение

    for (uint8_t byte_idx = 0; byte_idx < len; byte_idx++) {
        crc ^= data[byte_idx];
        for (uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Инициализация датчика SHT40.
 */
void SHT40_Init(void) {
    // Небольшая задержка после подачи питания
    HAL_Delay(10);
}

/**
 * @brief Выполняет мягкий сброс датчика SHT40.
 */
void SHT40_SoftReset(void) {
    uint8_t cmd = SHT40_CMD_SOFT_RESET;
    HAL_I2C_Master_Transmit(&hi2c3, SHT40_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
}

/**
 * @brief Чтение данных температуры и влажности с датчика SHT40.
 * @param temperature Указатель на переменную для хранения температуры (градусы Цельсия).
 * @param humidity Указатель на переменную для хранения влажности (%).
 * @return 0 в случае успеха, иначе код ошибки.
 */
uint8_t SHT40_Read_Data(float *temperature, float *humidity) {
    uint8_t cmd = SHT40_CMD_HIGH_PREC;
    uint8_t raw_data[6];
    uint16_t raw_temp, raw_humidity;

    // Отправка команды
    if (HAL_I2C_Master_Transmit(&hi2c3, SHT40_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }

    HAL_Delay(10);

    // Прием данных
    if (HAL_I2C_Master_Receive(&hi2c3, SHT40_I2C_ADDR, raw_data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return 2;
    }

    // Проверка CRC
    if (SHT40_CalcCrc8(&raw_data[0], 2) != raw_data[2] ||
        SHT40_CalcCrc8(&raw_data[3], 2) != raw_data[5]) {
        return 3;
    }

    raw_temp = ((uint16_t)raw_data[0] << 8) | raw_data[1];
    raw_humidity = ((uint16_t)raw_data[3] << 8) | raw_data[4];

    *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *humidity = -6.0f + 125.0f * ((float)raw_humidity / 65535.0f);

    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;

    return 0;
}
