#include "lis3mdl.h"

extern SPI_HandleTypeDef hspi1;

static uint8_t SPI_WriteByte(uint8_t data) {
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

void LIS3MDL_WriteReg(uint8_t reg, uint8_t value) {
    LIS3MDL_CS_LOW();
    SPI_WriteByte(reg & 0x7F);   // Бит записи 0
    SPI_WriteByte(value);
    LIS3MDL_CS_HIGH();
}

uint8_t LIS3MDL_ReadReg(uint8_t reg) {
    uint8_t value = 0;
    LIS3MDL_CS_LOW();
    SPI_WriteByte(reg | 0x80);   // Бит чтения 1
    value = SPI_WriteByte(0xFF);
    LIS3MDL_CS_HIGH();
    return value;
}

uint8_t LIS3MDL_Init(void) {
    HAL_Delay(100); // Ждем запуска датчика

    if (LIS3MDL_ReadReg(LIS3MDL_WHO_AM_I) != 0x3D) {
        return 0; // Ошибка инициализации
    }

    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG2, 0x40); // FS = ±12 Gauss
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG1, 0xFC); // ODR = 80Hz, UHP X/Y
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG4, 0x0C); // UHP Z-axis
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG3, 0x00); // Continuous mode
    return 1;
}

void LIS3MDL_Read_Mag(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t data[6];

    LIS3MDL_CS_LOW();
    SPI_WriteByte(LIS3MDL_OUT_X_L | 0x80 | 0x40); // Чтение с автоинкрементом
    for (int i = 0; i < 6; i++) {
        data[i] = SPI_WriteByte(0xFF);
    }
    LIS3MDL_CS_HIGH();

    *mx = (int16_t)((data[1] << 8) | data[0]);
    *my = (int16_t)((data[3] << 8) | data[2]);
    *mz = (int16_t)((data[5] << 8) | data[4]);
}
