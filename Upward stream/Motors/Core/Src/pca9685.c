#include "pca9685.h"
#include "i2c.h"   // внешний хэндл hi2c1, который вы сгенерировали в CubeMX

extern I2C_HandleTypeDef hi2c1;

static void PCA9685_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void PCA9685_WriteRegMulti(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void PCA9685_Init(uint16_t pwm_freq_hz) {
    // Перевести в режим сна
    PCA9685_WriteReg(PCA9685_MODE1, 0x10);

    // Рассчитать значение предделителя
    // Formula: prescale = round(25e6 / (4096 * pwm_freq_hz)) - 1
    uint32_t freq_raw = 25000000UL;  // внутренний осциллятор 25 МГц
    uint32_t prescale = (freq_raw / (4096UL * pwm_freq_hz)) - 1;
    PCA9685_WriteReg(PCA9685_PRE_SCALE, (uint8_t)prescale);

    // Выход из сна + включить автоинкремент адреса (бит AI)
    PCA9685_WriteReg(PCA9685_MODE1, 0x21);  // 0x21 = 0b00100001 (AI=1, SLEEP=0)

    // Режим2: выходы с totem pole (OUTDRV=1), инвертировать не нужно
    PCA9685_WriteReg(PCA9685_MODE2, 0x04);

    HAL_Delay(1);
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) return;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];
    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;
    PCA9685_WriteRegMulti(reg, data, 4);
}

void PCA9685_SetDutyPercent(uint8_t channel, uint8_t percent) {
    if (percent >= 100) {
        PCA9685_SetFullOn(channel);
    } else if (percent == 0) {
        PCA9685_SetFullOff(channel);
    } else {
        uint16_t off = (percent * 4095) / 100;
        PCA9685_SetPWM(channel, 0, off);
    }
}

void PCA9685_SetFullOn(uint8_t channel) {
    if (channel > 15) return;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4] = {0, 0x10, 0, 0};  // ON=4096 (full on)
    PCA9685_WriteRegMulti(reg, data, 4);
}

void PCA9685_SetFullOff(uint8_t channel) {
    if (channel > 15) return;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4] = {0, 0, 0, 0x10};  // OFF=4096 (full off)
    PCA9685_WriteRegMulti(reg, data, 4);
}
