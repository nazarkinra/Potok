#ifndef BMP388_H
#define BMP388_H

#include "stm32f4xx_hal.h"

#define BMP388_CS_PIN_MCP  3

#define BMP388_OK       0
#define BMP388_ERROR    1
#define BMP388_TIMEOUT  2

typedef struct {
    uint16_t T1;
    uint16_t T2;
    int8_t   T3;
    int16_t  P1;   // Исправлено на signed
    int16_t  P2;   // Исправлено на signed
    int8_t   P3;
    int8_t   P4;
    uint16_t P5;
    uint16_t P6;
    int8_t   P7;
    int8_t   P8;
    int16_t  P9;
    int8_t   P10;
    int8_t   P11;
} BMP388_CalibData;

uint8_t BMP388_Init(void);
uint8_t BMP388_ReadReg(uint8_t reg);
uint8_t BMP388_Read(float *temperature, float *pressure);
float   BMP388_Altitude(float sea_level_pressure, float current_pressure);

#endif // BMP388_H
