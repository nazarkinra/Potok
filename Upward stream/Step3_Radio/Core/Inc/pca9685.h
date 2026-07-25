#ifndef PCA9685_H
#define PCA9685_H

#include "main.h"  // для uint8_t, uint16_t и т.д.

// Адрес PCA9685 по умолчанию (7-битный 0x40)
#define PCA9685_I2C_ADDR 0x40

// Регистры
#define PCA9685_MODE1     0x00
#define PCA9685_MODE2     0x01
#define PCA9685_PRE_SCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// Инициализация: частота ШИМ (например, 50 Гц для сервоприводов)
void PCA9685_Init(uint16_t pwm_freq_hz);

// Установка значений ON и OFF для канала (12-битные значения)
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

// Установка скважности в процентах (0..100%) для канала
void PCA9685_SetDutyPercent(uint8_t channel, uint8_t percent);

// Полностью включить канал (100% ШИМ)
void PCA9685_SetFullOn(uint8_t channel);

// Полностью выключить канал (0% ШИМ)
void PCA9685_SetFullOff(uint8_t channel);

#endif
