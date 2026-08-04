#ifndef MCP23017_H
#define MCP23017_H

#include "main.h"

/*
 * I2C адрес MCP23017.
 * Если пины A0, A1, A2 подключены к GND, базовый адрес 0x20.
 * Сдвиг влево для HAL (0x20 << 1) дает 0x40.
 */
#define MCP23017_I2C_ADDR (0x20 << 1)

// Регистры MCP23017 (при IOCON.BANK = 0, по умолчанию)
#define MCP23017_IODIRA   0x00 // Направление порта A (1 = Вход, 0 = Выход)
#define MCP23017_IODIRB   0x01 // Направление порта B
#define MCP23017_GPIOA    0x12 // Чтение порта A
#define MCP23017_GPIOB    0x13 // Чтение порта B
#define MCP23017_OLATA    0x14 // Запись порта A (защелка)
#define MCP23017_OLATB    0x15 // Запись порта B (защелка)

// Инициализация
HAL_StatusTypeDef MCP23017_Init(void);

// Записать состояние сразу всех 16 пинов
void MCP23017_WriteAll(uint16_t data);

// Установить конкретный пин (pin: 0..15, state: 0 или 1)
// Пины 0-7 = Порт A, Пины 8-15 = Порт B
void MCP23017_WritePin(uint8_t pin, uint8_t state);

#endif // MCP23017_H
