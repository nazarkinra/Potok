#ifndef SHT40_H
#define SHT40_H

#include "main.h"

// I2C адрес датчика (7 бит)
#define SHT40_I2C_ADDR          (0x44 << 1)

// Команды для SHT40
#define SHT40_CMD_HIGH_PREC    0xFD   // Высокая точность
#define SHT40_CMD_MED_PREC     0xF6   // Средняя точность
#define SHT40_CMD_LOW_PREC     0xE0   // Низкая точность
#define SHT40_CMD_SOFT_RESET   0x94   // Мягкий сброс

// Функции драйвера
void SHT40_Init(void);
uint8_t SHT40_Read_Data(float *temperature, float *humidity);

#endif
