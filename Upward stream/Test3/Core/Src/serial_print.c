#include "serial_print.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// Размер внутреннего буфера. При необходимости измените под свои задачи.
#define SERIAL_PRINTF_BUFFER_SIZE 256

void Serial_Printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    if (huart == NULL || format == NULL) return;

    char buffer[SERIAL_PRINTF_BUFFER_SIZE];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0)
    {
        // Если строка не поместилась в буфер, выводим только то, что поместилось.
        size_t transmit_len = (len < sizeof(buffer)) ? len : sizeof(buffer) - 1;
        HAL_UART_Transmit(huart, (uint8_t*)buffer, transmit_len, HAL_MAX_DELAY);
    }
}

void Serial_PrintString(UART_HandleTypeDef *huart, const char *str)
{
    if (huart == NULL || str == NULL) return;
    HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
