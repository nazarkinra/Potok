#include "bmp280.h"
#include <math.h>

/* Регистры BMP280 */
#define REG_RESET     0xE0
#define REG_ID        0xD0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_PRESS_MSB 0xF7
#define REG_TEMP_MSB  0xFA
#define BMP280_ID     0x58

/* Внутренние функции I2C */
static HAL_StatusTypeDef bmp280_write_reg(BMP280_t *bmp, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(bmp->hi2c, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef bmp280_read_reg(BMP280_t *bmp, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(bmp->hi2c, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

// Возвращает t_fine в "сырых" единицах (нужен для давления)
static int32_t bmp280_compensate_T_int32(BMP280_t *bmp, int32_t adc_T, int32_t *t_fine) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp->dig_T1 << 1))) * ((int32_t)bmp->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp->dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->dig_T1))) >> 12) *
            ((int32_t)bmp->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8; // Температура в °C * 100 (целое)
}

static uint32_t bmp280_compensate_P_int32(BMP280_t *bmp, int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->dig_P3) >> 8) + ((var1 * (int64_t)bmp->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7) << 4);
    return (uint32_t)p; // Давление в Па * 256
}

// Компенсация влажности (только для BME280)
static int32_t bmp280_compensate_H_int32(BMP280_t *s, int32_t adc_H, int32_t t_fine) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)s->dig_H4) << 20) - (((int32_t)s->dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)s->dig_H6)) >> 10) *
                  (((v_x1_u32r * ((int32_t)s->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                  ((int32_t)2097152)) * ((int32_t)s->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                  ((int32_t)s->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    return (v_x1_u32r >> 12);  // Влажность * 1024
}

static HAL_StatusTypeDef bmp280_read_calib(BMP280_t *s) {
    uint8_t buf[26];  // 24 байта T/P + 2 байта H1/H3 для BME280

    // T/P калибровка (адрес 0x88, 24 байта)
    if (bmp280_read_reg(s, 0x88, buf, 24) != HAL_OK) return HAL_ERROR;
    s->dig_T1 = (uint16_t)(buf[0] | (buf[1] << 8));
    s->dig_T2 = (int16_t)(buf[2] | (buf[3] << 8));
    s->dig_T3 = (int16_t)(buf[4] | (buf[5] << 8));
    s->dig_P1 = (uint16_t)(buf[6] | (buf[7] << 8));
    s->dig_P2 = (int16_t)(buf[8] | (buf[9] << 8));
    s->dig_P3 = (int16_t)(buf[10] | (buf[11] << 8));
    s->dig_P4 = (int16_t)(buf[12] | (buf[13] << 8));
    s->dig_P5 = (int16_t)(buf[14] | (buf[15] << 8));
    s->dig_P6 = (int16_t)(buf[16] | (buf[17] << 8));
    s->dig_P7 = (int16_t)(buf[18] | (buf[19] << 8));
    s->dig_P8 = (int16_t)(buf[20] | (buf[21] << 8));
    s->dig_P9 = (int16_t)(buf[22] | (buf[23] << 8));

    // H калибровка (только для BME280)
    if (s->chip_id == BME280_ID) {
        bmp280_read_reg(s, 0xA1, &s->dig_H1, 1);           // 0xA1: dig_H1
        bmp280_read_reg(s, 0xE1, buf, 7);                  // 0xE1-0xE7: dig_H2..dig_H6
        s->dig_H2 = (int16_t)(buf[0] | (buf[1] << 8));
        s->dig_H3 = buf[2];
        s->dig_H4 = (buf[3] << 4) | (buf[4] & 0x0F);
        s->dig_H5 = (buf[5] << 4) | ((buf[4] >> 4) & 0x0F);
        s->dig_H6 = (int8_t)buf[6];
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMP280_Init(BMP280_t *s) {
    uint8_t id;
    if (bmp280_read_reg(s, 0xD0, &id, 1) != HAL_OK) return HAL_ERROR;

    if (id != BMP280_ID && id != BME280_ID) return HAL_ERROR;
    s->chip_id = id;

    // Сброс
    bmp280_write_reg(s, 0xE0, 0xB6);
    HAL_Delay(10);

    if (bmp280_read_calib(s) != HAL_OK) return HAL_ERROR;

    // Настройка: Normal mode, OSRS_T=1, OSRS_P=16, OSRS_H=1 (для BME280)
    if (s->chip_id == BME280_ID) {
        // Сначала ctrl_hum (0x72): OSRS_H = 1x
        bmp280_write_reg(s, 0xF2, 0x01);
        // ctrl_meas + config одним словом не получится, пишем раздельно
        bmp280_write_reg(s, 0xF4, 0x27);  // OSRS_T=1, OSRS_P=16, mode=Normal
        bmp280_write_reg(s, 0xF5, 0x24);  // IIR=4, standby=0.5ms
    } else {
        bmp280_write_reg(s, 0xF4, 0x27);
        bmp280_write_reg(s, 0xF5, 0x24);
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMP280_GetData(BMP280_t *s, float *temperature, float *pressure, float *humidity) {
    uint8_t raw[8];  // 6 байт P/T + 2 байта H (если BME280)

    // Читаем все данные сразу (регистры 0xF7-0xFE)
    if (bmp280_read_reg(s, 0xF7, raw, s->chip_id == BME280_ID ? 8 : 6) != HAL_OK)
        return HAL_ERROR;

    int32_t adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);

    // Компенсация температуры (возвращает t_fine)
    int32_t t_fine;
    int32_t t_raw = bmp280_compensate_T_int32(s, adc_T, &t_fine);
    *temperature = (float)t_raw / 100.0f;

    // Компенсация давления
    uint32_t p_raw = bmp280_compensate_P_int32(s, adc_P, t_fine);
    *pressure = (float)p_raw / 256.0f;

    // Влажность (только BME280)
    if (humidity && s->chip_id == BME280_ID) {
        int32_t adc_H = (raw[6] << 8) | raw[7];
        int32_t h_raw = bmp280_compensate_H_int32(s, adc_H, t_fine);
        *humidity = (float)h_raw / 1024.0f;  // %RH
    } else if (humidity) {
        *humidity = -1.0f;  // Не поддерживается
    }

    return HAL_OK;
}
