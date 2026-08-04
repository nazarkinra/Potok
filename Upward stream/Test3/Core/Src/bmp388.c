#include "bmp388.h"
#include "mcp23017.h"
#include <math.h>
#include "serial_print.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
static BMP388_CalibData calib;

static void SPI_CS_Low(void) {
    MCP23017_WritePin(BMP388_CS_PIN_MCP, 0);
}

static void SPI_CS_High(void) {
    MCP23017_WritePin(BMP388_CS_PIN_MCP, 1);
}

uint8_t BMP388_ReadReg(uint8_t reg) {
    uint8_t tx[3] = {reg | 0x80, 0xFF, 0xFF};
    uint8_t rx[3] = {0};
    SPI_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
    SPI_CS_High();
    return rx[2];
}

static void BMP388_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t tx[30] = {0};
    uint8_t rx[30] = {0};

    tx[0] = reg | 0x80;
    tx[1] = 0xFF;

    memset(&tx[2], 0xFF, len);

    SPI_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 2, HAL_MAX_DELAY);
    SPI_CS_High();

    memcpy(buffer, &rx[2], len);
}

static void BMP388_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    SPI_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    SPI_CS_High();
}

static uint8_t BMP388_ReadCalib(void) {
    uint8_t buffer[21];
    BMP388_ReadRegs(0x31, buffer, 21);

    calib.T1 = (uint16_t)buffer[1] << 8 | buffer[0];
    calib.T2 = (uint16_t)buffer[3] << 8 | buffer[2];
    calib.T3 = (int8_t)buffer[4];
    calib.P1 = (int16_t)(buffer[6] << 8 | buffer[5]);
    calib.P2 = (int16_t)(buffer[8] << 8 | buffer[7]);
    calib.P3 = (int8_t)buffer[9];
    calib.P4 = (int8_t)buffer[10];
    calib.P5 = (uint16_t)buffer[12] << 8 | buffer[11];
    calib.P6 = (uint16_t)buffer[14] << 8 | buffer[13];
    calib.P7 = (int8_t)buffer[15];
    calib.P8 = (int8_t)buffer[16];
    calib.P9 = (int16_t)(buffer[18] << 8 | buffer[17]);
    calib.P10 = (int8_t)buffer[19];
    calib.P11 = (int8_t)buffer[20];

    return BMP388_OK;
}

static void BMP388_Compensate(int32_t raw_temp, int32_t raw_press, float *temp, float *press) {
    // 1. Масштабируем калибровочные коэффициенты (официальные константы Bosch)
    double par_t1 = (double)calib.T1 * 256.0;
    double par_t2 = (double)calib.T2 / 1073741824.0;
    double par_t3 = (double)calib.T3 / 281474976710656.0;

    double par_p1 = ((double)calib.P1 - 16384.0) / 1048576.0;
    double par_p2 = ((double)calib.P2 - 16384.0) / 536870912.0;
    double par_p3 = (double)calib.P3 / 4294967296.0;
    double par_p4 = (double)calib.P4 / 137438953472.0;
    double par_p5 = (double)calib.P5 * 8.0;
    double par_p6 = (double)calib.P6 / 64.0;
    double par_p7 = (double)calib.P7 / 256.0;
    double par_p8 = (double)calib.P8 / 32768.0;
    double par_p9 = (double)calib.P9 / 281474976710656.0;
    double par_p10 = (double)calib.P10 / 281474976710656.0;
    double par_p11 = (double)calib.P11 / 36893488147419103232.0;

    // 2. Компенсация температуры
    double pd1 = (double)raw_temp - par_t1;
    double pd2 = pd1 * par_t2;
    double t_lin = pd2 + (pd1 * pd1) * par_t3;

    *temp = (float)t_lin;

    // 3. Компенсация давления
    double t_lin2 = t_lin * t_lin;
    double t_lin3 = t_lin2 * t_lin;

    double po1 = par_p5 + (par_p6 * t_lin) + (par_p7 * t_lin2) + (par_p8 * t_lin3);
    double po2 = (double)raw_press * (par_p1 + (par_p2 * t_lin) + (par_p3 * t_lin2) + (par_p4 * t_lin3));

    double raw_press2 = (double)raw_press * (double)raw_press;
    double raw_press3 = raw_press2 * (double)raw_press;

    double pd3 = raw_press2 * (par_p9 + par_p10 * t_lin);
    double pd4 = pd3 + (raw_press3 * par_p11);

    double comp_press = po1 + po2 + pd4;

    // Возвращаем давление в гектопаскалях (hPa)
    *press = (float)(comp_press / 100.0);
}

uint8_t BMP388_Init(void) {
    MCP23017_WritePin(BMP388_CS_PIN_MCP, 1);

    if (BMP388_ReadReg(0x00) != 0x50) return BMP388_ERROR;

    // Сброс
    BMP388_WriteReg(0x7E, 0xB6);
    HAL_Delay(10); // Ждем перезагрузки чипа

    // 🔴 КРИТИЧНО: Пустое чтение для пробуждения SPI-интерфейса!
    BMP388_ReadReg(0x00);
    HAL_Delay(2);

    if (BMP388_ReadCalib() != BMP388_OK) return BMP388_ERROR;

    BMP388_WriteReg(0x1C, 0x00);
    BMP388_WriteReg(0x1D, 0x01);
    BMP388_WriteReg(0x1B, 0x33);

    return BMP388_OK;
}

uint8_t BMP388_Read(float *temperature, float *pressure) {
    uint8_t buffer[6];

    BMP388_ReadRegs(0x04, buffer, 6);

    uint32_t raw_press = (uint32_t)buffer[2] << 16 | (uint32_t)buffer[1] << 8 | buffer[0];
    uint32_t raw_temp  = (uint32_t)buffer[5] << 16 | (uint32_t)buffer[4] << 8 | buffer[3];

    BMP388_Compensate((int32_t)raw_temp, (int32_t)raw_press, temperature, pressure);
    return BMP388_OK;
}

float BMP388_Altitude(float sea_level_pressure, float current_pressure) {
    if (sea_level_pressure <= 0 || current_pressure <= 0) return 0;
    // Оставляем powf, так как она 32-битная и аппаратно поддерживается процессором
    float ratio = current_pressure / sea_level_pressure;
    return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}
