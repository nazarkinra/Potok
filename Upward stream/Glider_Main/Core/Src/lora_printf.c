#include "lora_printf.h"
#include <string.h>
#include <stdio.h>

static LORA_SendFunc_t g_send_func = NULL;
static LORA_RecvFunc_t g_recv_func = NULL;

// Инициализация: передаём функции отправки и приёма
void LoRa_Printf_Init(LORA_SendFunc_t send_func, LORA_RecvFunc_t recv_func) {
    g_send_func = send_func;
    g_recv_func = recv_func;
}

// 🔹 Отправка (без изменений)
int LoRa_Printf(const char *format, ...) {
    if (g_send_func == NULL || format == NULL) return -1;

    char buffer[LORA_PRINTF_MAX_LEN];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0) return -1;
    if (len >= LORA_PRINTF_MAX_LEN) len = LORA_PRINTF_MAX_LEN - 1;

    return g_send_func((uint8_t*)buffer, (uint8_t)len, LORA_PRINTF_TIMEOUT);
}

// 🔹 Приём: вызывается из вашей библиотеки после успешного чтения из радио
void LoRa_Printf_HandleRx(uint8_t *data, uint8_t len) {
    if (g_recv_func == NULL || data == NULL || len == 0) return;

    // Копируем в локальный буфер и добавляем '\0' для работы со строкой
    char str[LORA_PRINTF_MAX_LEN + 1];
    uint8_t copy_len = (len > LORA_PRINTF_MAX_LEN) ? LORA_PRINTF_MAX_LEN : len;

    memcpy(str, data, copy_len);
    str[copy_len] = '\0';  // Null-терминатор

    // Вызываем пользовательский обработчик
    g_recv_func(str, copy_len);
}
