#include "mcp23017.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef MCP23017_WriteReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, MCP23017_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static uint8_t MCP23017_ReadReg(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MCP23017_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

HAL_StatusTypeDef MCP23017_Init(void) {
    HAL_Delay(50);

    // Проверяем физическое присутствие MCP23017 на шине I2C
    if (HAL_I2C_IsDeviceReady(&hi2c1, MCP23017_I2C_ADDR, 3, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // 1. Устанавливаем 1 на всех пинах, чтобы CS датчиков не упали в 0 при включении
    MCP23017_WriteReg(MCP23017_OLATA, 0xFF);
    MCP23017_WriteReg(MCP23017_OLATB, 0xFF);

    // 2. Настраиваем все 16 пинов как ВЫХОДЫ (у MCP23017 0 = Выход)
    MCP23017_WriteReg(MCP23017_IODIRA, 0x00);
    MCP23017_WriteReg(MCP23017_IODIRB, 0x00);

    // 3. ПРОВЕРКА: действительно ли регистры приняли настройки?
    uint8_t check_dira = MCP23017_ReadReg(MCP23017_IODIRA);
    uint8_t check_dirb = MCP23017_ReadReg(MCP23017_IODIRB);

    if (check_dira != 0x00 || check_dirb != 0x00) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void MCP23017_WriteAll(uint16_t data) {
    MCP23017_WriteReg(MCP23017_OLATA, data & 0xFF);
    MCP23017_WriteReg(MCP23017_OLATB, (data >> 8) & 0xFF);
}

void MCP23017_WritePin(uint8_t pin, uint8_t state) {
    if (pin > 15) return;

    // Определяем, с каким портом (A или B) работаем
    uint8_t reg = (pin < 8) ? MCP23017_OLATA : MCP23017_OLATB;
    uint8_t bit = pin % 8;

    uint8_t current_state = MCP23017_ReadReg(reg);

    if (state) {
        current_state |= (1 << bit);
    } else {
        current_state &= ~(1 << bit);
    }

    MCP23017_WriteReg(reg, current_state);
}
