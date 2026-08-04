#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H

#include "stm32f4xx_hal.h"

/**
 * @brief  Выводит форматированную строку в UART (аналог printf).
 * @param  huart: указатель на структуру UART_HandleTypeDef
 * @param  format: форматная строка (поддерживаются все спецификаторы printf)
 * @param  ...: переменное количество аргументов
 * @retval None
 */
void Serial_Printf(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief  Выводит простую строку в UART.
 * @param  huart: указатель на структуру UART_HandleTypeDef
 * @param  str: указатель на нуль-терминированную строку
 * @retval None
 */
void Serial_PrintString(UART_HandleTypeDef *huart, const char *str);

#endif /* SERIAL_PRINT_H */
