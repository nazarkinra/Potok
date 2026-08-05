#include "lis3mdl.h"
#include "mcp23017.h"
#include "serial_print.h"

extern UART_HandleTypeDef huart1;

extern SPI_HandleTypeDef hspi1;

// Массивы, которые ищет ваш компилятор для функций LIS3MDL_Read_Mag и LIS3MDL_Calibrate
float mag_offset[3] = {0.0f, 0.0f, 0.0f};
float mag_scale[3]  = {1.0f, 1.0f, 1.0f};

// Функция для записи калибровок из файла в библиотеку
void LIS3MDL_SetCalibration(float ox, float oy, float oz, float sx, float sy, float sz) {
    mag_offset[0] = ox; mag_offset[1] = oy; mag_offset[2] = oz;
    mag_scale[0] = sx;  mag_scale[1] = sy;  mag_scale[2] = sz;
}

// Функция для получения новых калибровок после процедуры LIS3MDL_Calibrate()
void LIS3MDL_GetCalibration(float *ox, float *oy, float *oz, float *sx, float *sy, float *sz) {
    *ox = mag_offset[0]; *oy = mag_offset[1]; *oz = mag_offset[2];
    *sx = mag_scale[0];  *sy = mag_scale[1];  *sz = mag_scale[2];
}

// ВАЖНО: В вашей функции LIS3MDL_Read_Mag (или там, где вы переводите сырые данные в Гауссы)
// обязательно вычитайте эти переменные из итогового результата, точно так же, как мы это
// делали для акселерометра и гироскопа.

static uint8_t SPI_WriteByte(uint8_t data) {
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

void LIS3MDL_WriteReg(uint8_t reg, uint8_t value) {
    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 0);
    SPI_WriteByte(reg & 0x7F);
    SPI_WriteByte(value);
    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 1);
}

uint8_t LIS3MDL_ReadReg(uint8_t reg) {
    uint8_t value = 0;
    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 0);
    SPI_WriteByte(reg | 0x80);
    value = SPI_WriteByte(0xFF);
    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 1);
    return value;
}

uint8_t LIS3MDL_Init(void) {
    HAL_Delay(100);

    if (LIS3MDL_ReadReg(LIS3MDL_WHO_AM_I) != 0x3D) {
        return 0;
    }

    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG2, 0x40);
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG1, 0xFC);
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG4, 0x0C);
    LIS3MDL_WriteReg(LIS3MDL_CTRL_REG3, 0x00);
    return 1;
}

void LIS3MDL_Read_Raw(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t data[6];

    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 0);
    SPI_WriteByte(LIS3MDL_OUT_X_L | 0x80 | 0x40);
    for (int i = 0; i < 6; i++) {
        data[i] = SPI_WriteByte(0xFF);
    }
    MCP23017_WritePin(LIS3MDL_CS_PIN_MCP, 1);

    *mx = (int16_t)((data[1] << 8) | data[0]);
    *my = (int16_t)((data[3] << 8) | data[2]);
    *mz = (int16_t)((data[5] << 8) | data[4]);
}

void LIS3MDL_Read_Mag(int16_t *x, int16_t *y, int16_t *z) {
    int16_t raw_x, raw_y, raw_z;

    LIS3MDL_Read_Raw(&raw_x, &raw_y, &raw_z);

    *x = (int16_t)((raw_x - mag_offset[0]) * mag_scale[0]);
    *y = (int16_t)((raw_y - mag_offset[1]) * mag_scale[1]);
    *z = (int16_t)((raw_z - mag_offset[2]) * mag_scale[2]);
}

void LIS3MDL_Calibrate(void) {
    int16_t raw_x, raw_y, raw_z;
    int16_t min_x = 32767, max_x = -32768;
    int16_t min_y = 32767, max_y = -32768;
    int16_t min_z = 32767, max_z = -32768;

    Serial_Printf(&huart1, "\r\n*** MAG CALIBRATION STARTING IN 3 SEC ***\r\n");
    HAL_Delay(3000);
    Serial_Printf(&huart1, "*** ROTATE THE BOARD IN ALL 3D AXES NOW! ***\r\n");

    uint32_t start = HAL_GetTick();
    // Даем 15 секунд на вращение платы в пространстве (крути восьмеркой)
    while (HAL_GetTick() - start < 15000) {
        LIS3MDL_Read_Raw(&raw_x, &raw_y, &raw_z);

        if (raw_x < min_x) min_x = raw_x;
        if (raw_x > max_x) max_x = raw_x;
        if (raw_y < min_y) min_y = raw_y;
        if (raw_y > max_y) max_y = raw_y;
        if (raw_z < min_z) min_z = raw_z;
        if (raw_z > max_z) max_z = raw_z;

        HAL_Delay(20);
    }

    // Вычисляем смещение центров (Hard-Iron)
    mag_offset[0] = (max_x + min_x) / 2.0f;
    mag_offset[1] = (max_y + min_y) / 2.0f;
    mag_offset[2] = (max_z + min_z) / 2.0f;

    // Вычисляем деформацию осей (Soft-Iron)
    float avg_delta_x = (max_x - min_x) / 2.0f;
    float avg_delta_y = (max_y - min_y) / 2.0f;
    float avg_delta_z = (max_z - min_z) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;

    mag_scale[0] = (avg_delta_x != 0) ? (avg_delta / avg_delta_x) : 1.0f;
    mag_scale[1] = (avg_delta_y != 0) ? (avg_delta / avg_delta_y) : 1.0f;
    mag_scale[2] = (avg_delta_z != 0) ? (avg_delta / avg_delta_z) : 1.0f;

    Serial_Printf(&huart1, "\r\n*** MAG CALIBRATION DONE ***\r\n");
    Serial_Printf(&huart1, "COPY THESE VALUES INTO lis3mdl.c:\r\n");
    Serial_Printf(&huart1, "mag_offset[3] = {%.1f, %.1f, %.1f};\r\n", mag_offset[0], mag_offset[1], mag_offset[2]);
    Serial_Printf(&huart1, "mag_scale[3] = {%.3f, %.3f, %.3f};\r\n", mag_scale[0], mag_scale[1], mag_scale[2]);
}
