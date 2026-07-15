#ifndef BMP388_H
#define BMP388_H

#include "main.h"

// Пины CS (настройте под свой проект)
#define BMP388_CS_PORT    GPIOB
#define BMP388_CS_PIN     GPIO_PIN_12

// Результаты функций
#define BMP388_OK         0
#define BMP388_ERROR      1
#define BMP388_TIMEOUT    2

// Структура калибровочных коэффициентов (согласно даташиту)
typedef struct {
    uint16_t T1;
    uint16_t T2;
    int8_t   T3;
    uint16_t P1;
    uint16_t P2;
    int8_t   P3;
    int8_t   P4;
    uint16_t P5;
    uint16_t P6;
    int8_t   P7;
    int8_t   P8;
    int16_t  P9;
    uint8_t  P10;
    uint8_t  P11;
} BMP388_CalibData;

// Инициализация датчика (передаём указатель на SPI хэндл)
uint8_t BMP388_Init(SPI_HandleTypeDef *hspi);
uint8_t BMP388_ReadReg(uint8_t reg);

// Чтение скомпенсированных значений (температура в °C, давление в Па)
uint8_t BMP388_Read(float *temperature, float *pressure);

// Вычисление высоты по текущему давлению и опорному (Па)
float BMP388_Altitude(float sea_level_pressure, float current_pressure);

#endif
