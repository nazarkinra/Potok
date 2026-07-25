#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32f4xx_hal.h"

#define HMC5883L_I2C_ADDR (0x1E << 1) // 8-битный адрес для HAL

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L_Data_t;

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HMC5883L_ReadData(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data);

#endif /* HMC5883L_H */
