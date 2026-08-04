#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"

// Структура, описывающая один сервопривод, подключенный к каналу PCA9685
typedef struct {
    uint8_t channel;      // Канал на PCA9685 (0-15)
    uint16_t min_pwm;     // Значение ШИМ (тиков) для 0 градусов
    uint16_t max_pwm;     // Значение ШИМ (тиков) для максимального угла
    uint8_t max_angle;    // Максимальный угол поворота сервы (обычно 180)
} Servo_HandleTypeDef;

// Инициализация сервопривода со стандартными параметрами (0-180 град)
// Подходит для большинства SG90 / MG996R
void Servo_Init(Servo_HandleTypeDef *hservo, uint8_t channel);

// Инициализация сервопривода с ручной калибровкой крайних точек ШИМ
// Полезно, если сервопривод немного "не дотягивает" до краев
void Servo_Init_Custom(Servo_HandleTypeDef *hservo, uint8_t channel, uint16_t min_pwm, uint16_t max_pwm, uint8_t max_angle);

// Установка угла сервопривода (0 .. max_angle)
void Servo_SetAngle(Servo_HandleTypeDef *hservo, uint8_t angle);

#endif /* SERVO_CONTROL_H */
