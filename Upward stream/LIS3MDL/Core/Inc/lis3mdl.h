#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "main.h"
#include "stm32f4xx_hal_spi.h"

#define LIS3MDL_CS_LOW()   HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, GPIO_PIN_RESET)
#define LIS3MDL_CS_HIGH()  HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, GPIO_PIN_SET)

// Адреса регистров
#define LIS3MDL_WHO_AM_I       0x0F
#define LIS3MDL_CTRL_REG1      0x20
#define LIS3MDL_CTRL_REG2      0x21
#define LIS3MDL_CTRL_REG3      0x22
#define LIS3MDL_CTRL_REG4      0x23
#define LIS3MDL_STATUS_REG     0x27
#define LIS3MDL_OUT_X_L        0x28
#define LIS3MDL_ZYXDA_MASK     0x08

// Функции
uint8_t LIS3MDL_Init(void);
void LIS3MDL_Read_Mag(int16_t* mx, int16_t* my, int16_t* mz);
uint8_t LIS3MDL_ReadReg(uint8_t reg);
uint8_t LIS3MDL_IsDataReady(void);

#endif // LIS3MDL_H
