#ifndef LORA_PRINTF_H
#define LORA_PRINTF_H

#include <stdarg.h>
#include <stdint.h>
#include "main.h"

// 🔧 Настройки
#define LORA_PRINTF_MAX_LEN  200
#define LORA_PRINTF_TIMEOUT  5000

// 🔄 Типы функций-коллбэков
typedef int  (*LORA_SendFunc_t)(uint8_t *data, uint8_t len, uint32_t timeout_ms);
typedef void (*LORA_RecvFunc_t)(char *str, uint8_t len);  // 🔹 Новый: приём строки

// === Публичные функции ===
void LoRa_Printf_Init(LORA_SendFunc_t send_func, LORA_RecvFunc_t recv_func);
int  LoRa_Printf(const char *format, ...);

// 🔹 Функция для вызова из вашей библиотеки приёма
void LoRa_Printf_HandleRx(uint8_t *data, uint8_t len);

#endif
