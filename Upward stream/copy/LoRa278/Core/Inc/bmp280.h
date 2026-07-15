#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Адрес (у тебя 0xEC, так как сканер показал 7-bit: 0x76)
#define BMP280_I2C_ADDR 0xEC

// Допустимые ID
#define BMP280_ID 0x58
#define BME280_ID 0x60

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t chip_id;  // 0x58 или 0x60

    // Калибровка для температуры/давления (одинакова для BMP280/BME280)
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    // Калибровка влажности (только для BME280)
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4, dig_H5;
    int8_t   dig_H6;
} BMP280_t;  // Можно переименовать в ENV_SENSOR_t для универсальности

HAL_StatusTypeDef BMP280_Init(BMP280_t *sensor);
HAL_StatusTypeDef BMP280_GetData(BMP280_t *sensor, float *temperature, float *pressure, float *humidity);

#endif
