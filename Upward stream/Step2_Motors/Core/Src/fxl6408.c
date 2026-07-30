#include "fxl6408.h"
#include "i2c.h" // Внешний хэндл hi2c1 из CubeMX

extern I2C_HandleTypeDef hi2c1;

// Вспомогательная функция для записи в регистр
static void FXL6408_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, FXL6408_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// Вспомогательная функция для чтения из регистра
static uint8_t FXL6408_ReadReg(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, FXL6408_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

void FXL6408_Init(void) {
    // Программный сброс (записываем 1 в 0-й бит)
    FXL6408_WriteReg(FXL6408_REG_DEVICE_ID_CTRL, 0x01);
    HAL_Delay(1);

    // Возврат в нормальный режим работы
    FXL6408_WriteReg(FXL6408_REG_DEVICE_ID_CTRL, 0x00);
    HAL_Delay(1);

    // Устанавливаем все 8 пинов на ВЫХОД (1 = выход, 0xFF = все выходы)
    FXL6408_WriteReg(FXL6408_REG_IO_DIR, 0xFF);

    // Снимаем состояние High-Z со всех пинов (1 = выход активен)
    FXL6408_WriteReg(FXL6408_REG_OUT_HIGH_Z, 0xFF);

    // По умолчанию сбрасываем все выходы в 0 (выключены)
    FXL6408_WriteReg(FXL6408_REG_OUT_STATE, 0x00);
}

void FXL6408_WriteAll(uint8_t data) {
    FXL6408_WriteReg(FXL6408_REG_OUT_STATE, data);
}

void FXL6408_WritePin(uint8_t pin, uint8_t state) {
    if (pin > 7) return;

    // Читаем текущее состояние выходов, чтобы изменить только нужный бит
    uint8_t current_state = FXL6408_ReadReg(FXL6408_REG_OUT_STATE);

    if (state) {
        current_state |= (1 << pin);  // Устанавливаем бит в 1
    } else {
        current_state &= ~(1 << pin); // Сбрасываем бит в 0
    }

    // Записываем обновленное состояние
    FXL6408_WriteReg(FXL6408_REG_OUT_STATE, current_state);
}
