#include "servo_control.h"
#include "pca9685.h"

/*
 * Стандартные значения для сервоприводов при частоте 50 Гц (период 20 мс = 4096 тиков).
 * Импульс 0.5 мс = ~102 тика (0 градусов)
 * Импульс 2.5 мс = ~512 тиков (180 градусов)
 * Эти значения могут немного отличаться в зависимости от модели серво.
 */
#define SERVO_DEFAULT_MIN_PWM 102
#define SERVO_DEFAULT_MAX_PWM 512
#define SERVO_DEFAULT_MAX_ANGLE 180

void Servo_Init(Servo_HandleTypeDef *hservo, uint8_t channel) {
    // Вызываем кастомную инициализацию с дефолтными значениями
    Servo_Init_Custom(hservo, channel, SERVO_DEFAULT_MIN_PWM, SERVO_DEFAULT_MAX_PWM, SERVO_DEFAULT_MAX_ANGLE);
}

void Servo_Init_Custom(Servo_HandleTypeDef *hservo, uint8_t channel, uint16_t min_pwm, uint16_t max_pwm, uint8_t max_angle) {
    hservo->channel = channel;
    hservo->min_pwm = min_pwm;
    hservo->max_pwm = max_pwm;
    hservo->max_angle = max_angle;

    // По умолчанию ставим в центр (90 градусов)
    Servo_SetAngle(hservo, hservo->max_angle / 2);
}

void Servo_SetAngle(Servo_HandleTypeDef *hservo, uint8_t angle) {
    // Ограничиваем угол, чтобы не сломать редуктор
    if (angle > hservo->max_angle) {
        angle = hservo->max_angle;
    }

    // Линейная интерполяция (аналог функции map() в Arduino)
    // Приведение к uint32_t обязательно, чтобы при умножении не было переполнения 16-битного числа
    uint32_t pwm_range = hservo->max_pwm - hservo->min_pwm;
    uint16_t pwm_val = (uint16_t)(((uint32_t)angle * pwm_range) / hservo->max_angle + hservo->min_pwm);

    // Устанавливаем расчитанное значение для нужного канала
    PCA9685_SetPWM(hservo->channel, 0, pwm_val);
}
