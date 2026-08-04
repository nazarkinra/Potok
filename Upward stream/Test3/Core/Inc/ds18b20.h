#ifndef DS18B20_H
#define DS18B20_H

#include "stm32f4xx_hal.h"

uint8_t DS18B20_Start(void);
void DS18B20_WriteByte(uint8_t data);
uint8_t DS18B20_ReadByte(void);

#endif
