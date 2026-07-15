#include "lis3mdl.h"

extern SPI_HandleTypeDef hspi1;

void LIS3MDL_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg & 0x7F, value};
    LIS3MDL_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    LIS3MDL_CS_HIGH();
}

uint8_t LIS3MDL_ReadReg(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0xFF};
    uint8_t rx[2] = {0};
    LIS3MDL_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    LIS3MDL_CS_HIGH();
    return rx[1]; // Первый байт - dummy, второй - данные
}

uint8_t LIS3MDL_Init(void) {
    HAL_Delay(100); // Ждём стабилизации питания датчика

    // 1. Проверяем наличие датчика
    if (LIS3MDL_ReadReg(LIS3MDL_WHO_AM_I) != 0x3D) {
        return 0;
    }

    // 2. Настройка регистров
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG2, 0x40); // ±12 Gauss
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG1, 0x7C); // UHP X/Y, ODR=10Hz, FAST_ODR=0
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG4, 0x00); // ⚠️ ВАЖНО: BLE=0 (Little Endian), UHP Z
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG3, 0x00); // Continuous mode

    // Даём датчику 15-20 мс на первую конвертацию (при 10Hz)
    HAL_Delay(20);
    return 1;
}

void LIS3MDL_Read_Mag(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t cmd = LIS3MDL_OUT_X_L | 0x80 | 0x40; // Read + Auto-increment
    uint8_t tx[7] = {cmd, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx[7] = {0};

    LIS3MDL_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
    LIS3MDL_CS_HIGH();

    // Датчик отдаёт LSB первым (Little Endian)
    // rx[0] - dummy, rx[1]=X_L, rx[2]=X_H, rx[3]=Y_L, rx[4]=Y_H, rx[5]=Z_L, rx[6]=Z_H
    *mx = (int16_t)((rx[2] << 8) | rx[1]);
    *my = (int16_t)((rx[4] << 8) | rx[3]);
    *mz = (int16_t)((rx[6] << 8) | rx[5]);
}

uint8_t LIS3MDL_IsDataReady(void) {
    return (LIS3MDL_ReadReg(LIS3MDL_STATUS_REG) & LIS3MDL_ZYXDA_MASK) ? 1 : 0;
}
