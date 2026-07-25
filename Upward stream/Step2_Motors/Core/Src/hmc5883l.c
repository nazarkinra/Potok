#include "hmc5883l.h"
#include <string.h>

#define REG_CRA      0x00
#define REG_CRB      0x01
#define REG_MODE     0x02
#define REG_DATA_X   0x03
#define REG_STATUS   0x09
#define REG_ID_A     0x0A
#define REG_ID_B     0x0B
#define REG_ID_C     0x0C

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id[3];
    HAL_StatusTypeDef status;

    // Чтение ID датчика
    status = HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDR, REG_ID_A, I2C_MEMADD_SIZE_8BIT, id, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) return HAL_ERROR;

    // CRA: 8 выборок усреднения, частота 15 Гц (0x70)
    uint8_t cra = 0x70;
    status = HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDR, REG_CRA, I2C_MEMADD_SIZE_8BIT, &cra, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // CRB: Усиление ±1.3 Гаусс (1090 LSB/G) (0x20)
    uint8_t crb = 0x20;
    status = HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDR, REG_CRB, I2C_MEMADD_SIZE_8BIT, &crb, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // MODE: Непрерывный режим (0x00)
    uint8_t mode = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDR, REG_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);
    return status;
}

HAL_StatusTypeDef HMC5883L_ReadData(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data) {
    uint8_t status_reg;
    HAL_StatusTypeDef status;

    // Проверка флага готовности данных
    status = HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDR, REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    if (!(status_reg & 0x01)) return HAL_BUSY; // Данные ещё не готовы

    uint8_t buf[6];
    status = HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDR, REG_DATA_X, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Порядок регистров: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
    data->x = (int16_t)((buf[0] << 8) | buf[1]);
    data->z = (int16_t)((buf[2] << 8) | buf[3]); // Z идёт перед Y в регистровой карте
    data->y = (int16_t)((buf[4] << 8) | buf[5]);

    // Опционально: приведение к правой системе координат (X, Y, Z)
    // int16_t temp = data->y; data->y = data->z; data->z = temp;

    return HAL_OK;
}
