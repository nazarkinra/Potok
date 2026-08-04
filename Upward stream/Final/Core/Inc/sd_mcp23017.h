#ifndef SD_MCP23017_H
#define SD_MCP23017_H

#include "stm32f4xx_hal.h"
#include "ff.h"            // Библиотека FatFs
#include "mcp23017.h"      // Ваша библиотека расширителя портов[cite: 1]
#include "serial_print.h"  // Ваша библиотека вывода логов[cite: 2]

// Укажите UART, который используется для вывода логов
extern UART_HandleTypeDef huart1;
#define DEBUG_UART &huart1

// Пин CS подключен к GPA0 (индекс 0)
#define SD_CS_PIN 0

// Макросы управления пином CS через MCP23017
#define SD_CS_LOW()  MCP23017_WritePin(SD_CS_PIN, 0) //[cite: 1, 3]
#define SD_CS_HIGH() MCP23017_WritePin(SD_CS_PIN, 1) //[cite: 1, 3]

/**
 * @brief Инициализация SD карты и файловой системы
 * @param hspi: Указатель на структуру SPI (например, &hspi2)
 * @retval 0 - Успех, иначе - код ошибки
 */
uint8_t SD_MCP_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief Запись строки в файл на SD карте
 * @param filename: Имя файла (например, "data.txt")
 * @param data: Строка для записи
 * @retval 0 - Успех, иначе - код ошибки
 */
uint8_t SD_MCP_WriteFile(const char *filename, const char *data);

#endif /* SD_MCP23017_H */
