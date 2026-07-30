#ifndef FXL6408_H
#define FXL6408_H

#include "main.h"

// Стандартный 7-битный адрес FXL6408
#define FXL6408_I2C_ADDR 0x43

// Регистры FXL6408
#define FXL6408_REG_DEVICE_ID_CTRL 0x01
#define FXL6408_REG_IO_DIR         0x03
#define FXL6408_REG_OUT_STATE      0x05
#define FXL6408_REG_OUT_HIGH_Z     0x07

// Инициализация (настраивает все 8 пинов как выходы и включает их)
void FXL6408_Init(void);

// Установить состояние сразу всех 8 пинов (data: маска 0x00..0xFF)
void FXL6408_WriteAll(uint8_t data);

// Установить состояние конкретного пина (pin: 0..7, state: 1 или 0)
void FXL6408_WritePin(uint8_t pin, uint8_t state);

#endif // FXL6408_H
