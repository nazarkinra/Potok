#include "ds18b20.h"
#include "dwt_delay.h"

#define DS_PORT GPIOA
#define DS_PIN  GPIO_PIN_2

static void DS_Set(uint8_t state) {
    HAL_GPIO_WritePin(DS_PORT, DS_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t DS_Read(void) {
    return HAL_GPIO_ReadPin(DS_PORT, DS_PIN);
}

uint8_t DS18B20_Start(void) {
    uint8_t response = 0;
    DS_Set(0);
    DWT_Delay_us(480); // Импульс сброса
    DS_Set(1);
    DWT_Delay_us(80);  // Ждем ответ от датчика
    if (!DS_Read()) response = 1; // Датчик притянул линию к нулю (Presence)
    DWT_Delay_us(400);
    return response;
}

static void DS18B20_WriteBit(uint8_t bit) {
    DS_Set(0);
    DWT_Delay_us(bit ? 10 : 60);
    DS_Set(1);
    DWT_Delay_us(bit ? 50 : 10);
}

static uint8_t DS18B20_ReadBit(void) {
    uint8_t bit = 0;
    DS_Set(0);
    DWT_Delay_us(2);
    DS_Set(1);
    DWT_Delay_us(10);
    if (DS_Read()) bit = 1;
    DWT_Delay_us(50);
    return bit;
}

void DS18B20_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        DS18B20_WriteBit((data >> i) & 1);
    }
}

uint8_t DS18B20_ReadByte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data |= (DS18B20_ReadBit() << i);
    }
    return data;
}
