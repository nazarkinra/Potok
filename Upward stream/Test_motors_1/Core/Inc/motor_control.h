#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

// Структура, описывающая один двигатель, подключённый к двум каналам PCA9685
typedef struct {
    uint8_t ch_ain1;   // канал для AIN1 (или BIN1)
    uint8_t ch_ain2;   // канал для AIN2 (или BIN2)
} Motor_HandleTypeDef;

// Инициализация структуры мотора (просто сохраняет номера каналов)
void Motor_Init(Motor_HandleTypeDef *hmotor, uint8_t ch_ain1, uint8_t ch_ain2);

// Установка скорости мотора: -100..100 (отрицательная = обратное направление)
void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t speed);

// Активное торможение (оба входа HIGH)
void Motor_Brake(Motor_HandleTypeDef *hmotor);

// Свободный выбег (оба входа LOW)
void Motor_Coast(Motor_HandleTypeDef *hmotor);

#endif
