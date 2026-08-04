#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include "stm32f4xx_hal.h"

void DWT_Init(void);
void DWT_Delay_us(uint32_t us);

#endif
