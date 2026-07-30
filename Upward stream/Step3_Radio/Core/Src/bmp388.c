#include "bmp388.h"
#include <math.h>
#include "serial_print.h"

static SPI_HandleTypeDef hspi2;
static BMP388_CalibData calib;

// ---- Низкоуровневые SPI функции ----
static void SPI_CS_Low(void) {
    HAL_GPIO_WritePin(BMP388_CS_PORT, BMP388_CS_PIN, GPIO_PIN_RESET);
}

static void SPI_CS_High(void) {
    HAL_GPIO_WritePin(BMP388_CS_PORT, BMP388_CS_PIN, GPIO_PIN_SET);
}

static uint8_t SPI_WriteByte(uint8_t data) {
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi2, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

// Чтение одного регистра
uint8_t BMP388_ReadReg(uint8_t reg) {
    uint8_t value = 0;
    SPI_CS_Low();
    SPI_WriteByte(reg | 0x80);  // бит чтения = 1
    value = SPI_WriteByte(0xFF);
    SPI_CS_High();
    return value;
}

// Чтение нескольких регистров подряд
static void BMP388_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t tx = reg | 0x80;
    SPI_CS_Low();
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, buffer, len, HAL_MAX_DELAY);
    SPI_CS_High();
}

// Запись одного байта в регистр
static void BMP388_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    SPI_CS_Low();
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
    SPI_CS_High();
}

// ---- Чтение калибровочных данных ----
static uint8_t BMP388_ReadCalib(void) {
    uint8_t buffer[21];
    BMP388_ReadRegs(0x31, buffer, 21);  // калибровки начинаются с адреса 0x31

    calib.T1 = (uint16_t)buffer[1] << 8 | buffer[0];
    calib.T2 = (uint16_t)buffer[3] << 8 | buffer[2];
    calib.T3 = (int8_t)buffer[4];
    calib.P1 = (uint16_t)buffer[6] << 8 | buffer[5];
    calib.P2 = (uint16_t)buffer[8] << 8 | buffer[7];
    calib.P3 = (int8_t)buffer[9];
    calib.P4 = (int8_t)buffer[10];
    calib.P5 = (uint16_t)buffer[12] << 8 | buffer[11];
    calib.P6 = (uint16_t)buffer[14] << 8 | buffer[13];
    calib.P7 = (int8_t)buffer[15];
    calib.P8 = (int8_t)buffer[16];
    calib.P9 = (int16_t)buffer[18] << 8 | buffer[17];
    calib.P10 = buffer[19];
    calib.P11 = buffer[20];

    return BMP388_OK;
}

// ---- Компенсация температуры и давления ----
static void BMP388_Compensate(int32_t raw_temp, int32_t raw_press, float *temp, float *press) {
    // Компенсация температуры (формулы из даташита BMP388)
    float t_lin = (float)raw_temp - calib.T1;
    float t_comp = calib.T2 * t_lin / 32768.0f;
    float t_comp_final = t_comp + (calib.T3 * t_lin * t_lin) / 1073741824.0f;
    *temp = t_comp_final / 100.0f;  // перевод в градусы Цельсия

    // Компенсация давления
    float p_lin = (float)raw_press - calib.P1;
    float p_comp = calib.P2 * p_lin / 1073741824.0f;
    p_comp += calib.P3 * p_lin * p_lin / 70368744177664.0f;
    p_comp += calib.P4 * p_lin * p_lin * p_lin / 1.8446744073709552e19f;
    p_comp += calib.P5 * (powf(t_comp_final, 2) - calib.P6) / 144115188075855872.0f;
    p_comp += calib.P7 * p_lin * t_comp_final / 2199023255552.0f;
    p_comp += calib.P8 * p_lin * (powf(t_comp_final, 2) - calib.P9) / 281474976710656.0f;
    p_comp += calib.P10 * powf(t_comp_final, 3) / 1.152921504606846976e18f;
    p_comp += calib.P11 * p_lin * powf(t_comp_final, 3) / 1.180591620717411303424e21f;

    *press = p_comp / 100.0f;  // давление в Паскалях
}

// ---- Публичные функции ----
uint8_t BMP388_Init(void) {
    // Настройка пина CS как выхода
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = BMP388_CS_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMP388_CS_PORT, &gpio_init);
    HAL_GPIO_WritePin(BMP388_CS_PORT, BMP388_CS_PIN, GPIO_PIN_SET);
    // Проверка связи: читаем регистр WHO_AM_I (0x00) -> должно быть 0x50
    if (BMP388_ReadReg(0x00) != 0x50) return BMP388_ERROR;

    // Сброс датчика (опционально)
    BMP388_WriteReg(0x7E, 0xB6);
    HAL_Delay(10);

    // Чтение калибровок
    if (BMP388_ReadCalib() != BMP388_OK) return BMP388_ERROR;

    // Настройка режимов (нормальный режим, oversampling, ODR)
    // Регистр 0x1B (PWR_CTRL): включение давления и температуры
    BMP388_WriteReg(0x1B, 0x33);  // press_en=1, temp_en=1, mode=11 (normal)

    // Регистр 0x1C (OSR): oversampling и фильтр
    // [7:6] - IIR filter (3 = coeff 15), [5:3] - press_os (5 = 8x), [2:0] - temp_os (2 = 2x)
    BMP388_WriteReg(0x1C, 0x35);  // 0b00110101

    // Регистр 0x1D (ODR): output data rate (3 = 50Hz)
    BMP388_WriteReg(0x1D, 0x03);

    return BMP388_OK;
}

uint8_t BMP388_Read(float *temperature, float *pressure) {
    uint8_t buffer[6];

    // Читаем сырые данные: давление (3 байта) и температура (3 байта)
    // Адрес регистра данных: 0x04 (press_XLSB)
    BMP388_ReadRegs(0x04, buffer, 6);

    uint32_t raw_press = (uint32_t)buffer[2] << 16 | (uint32_t)buffer[1] << 8 | buffer[0];
    uint32_t raw_temp  = (uint32_t)buffer[5] << 16 | (uint32_t)buffer[4] << 8 | buffer[3];

    // Проверка статуса (регистр 0x03: bit 3 = cmd_rdy, bit 2 = drdy_press, bit 1 = drdy_temp)
    uint8_t status = BMP388_ReadReg(0x03);
    if (!(status & 0x02)) return BMP388_TIMEOUT;  // данные по температуре не готовы

    BMP388_Compensate((int32_t)raw_temp, (int32_t)raw_press, temperature, pressure);
    return BMP388_OK;
}

float BMP388_Altitude(float sea_level_pressure, float current_pressure) {
    if (sea_level_pressure <= 0 || current_pressure <= 0) return 0;
    float ratio = current_pressure / sea_level_pressure;
    return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}
