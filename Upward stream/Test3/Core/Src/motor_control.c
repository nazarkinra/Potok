#include "motor_control.h"
#include "pca9685.h"

void Motor_Init(Motor_HandleTypeDef *hmotor, uint8_t ch_ain1, uint8_t ch_ain2) {
    hmotor->ch_ain1 = ch_ain1;
    hmotor->ch_ain2 = ch_ain2;
    // По умолчанию останавливаем мотор (свободный выбег)
    Motor_Coast(hmotor);
}

void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t speed) {
    // Ограничение диапазона
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    uint8_t percent = (speed > 0) ? speed : -speed;  // абсолютное значение

    if (speed > 0) {
        // Вращение вперёд: AIN1 = ШИМ, AIN2 = 0
        PCA9685_SetDutyPercent(hmotor->ch_ain1, percent);
        PCA9685_SetDutyPercent(hmotor->ch_ain2, 0);
    }
    else if (speed < 0) {
        // Вращение назад: AIN1 = 0, AIN2 = ШИМ
        PCA9685_SetDutyPercent(hmotor->ch_ain1, 0);
        PCA9685_SetDutyPercent(hmotor->ch_ain2, percent);
    }
    else { // speed == 0
        // Остановка – свободный выбег (оба входа LOW)
        Motor_Coast(hmotor);
    }
}

void Motor_Brake(Motor_HandleTypeDef *hmotor) {
    // Активное торможение: оба входа HIGH
    PCA9685_SetFullOn(hmotor->ch_ain1);
    PCA9685_SetFullOn(hmotor->ch_ain2);
}

void Motor_Coast(Motor_HandleTypeDef *hmotor) {
    // Свободный выбег: оба входа LOW
    PCA9685_SetFullOff(hmotor->ch_ain1);
    PCA9685_SetFullOff(hmotor->ch_ain2);
}
