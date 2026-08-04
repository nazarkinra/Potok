#include "bmi088.h"

// Переменные для хранения калибровочных нулей гироскопа
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

// Переменные для акселерометра
static float acc_offset_x = 0.0f;
static float acc_offset_y = 0.0f;
static float acc_offset_z = 0.0f;

static void BMI088_ACC_CS_Low(void)  { MCP23017_WritePin(BMI088_CS_ACC_PIN_MCP, 0); }
static void BMI088_ACC_CS_High(void) { MCP23017_WritePin(BMI088_CS_ACC_PIN_MCP, 1); }
static void BMI088_GYR_CS_Low(void)  { MCP23017_WritePin(BMI088_CS_GYR_PIN_MCP, 0); }
static void BMI088_GYR_CS_High(void) { MCP23017_WritePin(BMI088_CS_GYR_PIN_MCP, 1); }

static HAL_StatusTypeDef BMI088_WriteReg(SPI_HandleTypeDef *hspi, uint8_t cs_pin, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    MCP23017_WritePin(cs_pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    MCP23017_WritePin(cs_pin, 1);
    return status;
}

static HAL_StatusTypeDef BMI088_ReadReg_Acc(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx = reg | 0x80;
    uint8_t dummy;
    BMI088_ACC_CS_Low();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &dummy, 1, HAL_MAX_DELAY); // Акселерометр требует 1 dummy байт
    HAL_StatusTypeDef status = HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
    BMI088_ACC_CS_High();
    return status;
}

static HAL_StatusTypeDef BMI088_ReadReg_Gyr(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx = reg | 0x80;
    BMI088_GYR_CS_Low();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_StatusTypeDef status = HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
    BMI088_GYR_CS_High();
    return status;
}

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi) {
    HAL_StatusTypeDef status;
    uint8_t chip_id = 0;

    BMI088_ACC_CS_High();
    HAL_Delay(1);
    BMI088_ACC_CS_Low();
    HAL_Delay(1);
    BMI088_ACC_CS_High();
    HAL_Delay(50);

    status = BMI088_ReadReg_Acc(hspi, BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (status != HAL_OK || chip_id != 0x1E) return HAL_ERROR;

    status = BMI088_ReadReg_Gyr(hspi, BMI088_GYR_CHIP_ID, &chip_id, 1);
    if (status != HAL_OK || chip_id != 0x0F) return HAL_ERROR;

    status = BMI088_WriteReg(hspi, BMI088_CS_ACC_PIN_MCP, BMI088_ACC_PWR_CONF, 0x00);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    status = BMI088_WriteReg(hspi, BMI088_CS_ACC_PIN_MCP, BMI088_ACC_PWR_CTRL, 0x04);
    if (status != HAL_OK) return status;
    HAL_Delay(50);

    status = BMI088_WriteReg(hspi, BMI088_CS_ACC_PIN_MCP, BMI088_ACC_CONF, 0xAB);
    status = BMI088_WriteReg(hspi, BMI088_CS_ACC_PIN_MCP, BMI088_ACC_RANGE, 0x01);

    status = BMI088_WriteReg(hspi, BMI088_CS_GYR_PIN_MCP, BMI088_GYR_RANGE, 0x03);
    status = BMI088_WriteReg(hspi, BMI088_CS_GYR_PIN_MCP, BMI088_GYR_BANDWIDTH, 0x01);

    return HAL_OK;
}

HAL_StatusTypeDef BMI088_CalibrateGyro(SPI_HandleTypeDef *hspi, uint16_t num_samples) {
    if (num_samples == 0) return HAL_ERROR;

    BMI088_Raw_t raw;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        if (BMI088_ReadRaw(hspi, &raw) != HAL_OK) {
            return HAL_ERROR;
        }
        // Переводим в физические величины перед усреднением
        sum_x += raw.Gyro_X / 131.072f;
        sum_y += raw.Gyro_Y / 131.072f;
        sum_z += raw.Gyro_Z / 131.072f;

        // Так как пропускная способность гироскопа настроена на 116 Гц,
        // нам нужно делать небольшую задержку между сэмплами, чтобы получать новые данные.
        HAL_Delay(10);
    }

    gyro_offset_x = sum_x / num_samples;
    gyro_offset_y = sum_y / num_samples;
    gyro_offset_z = sum_z / num_samples;

    return HAL_OK;
}

HAL_StatusTypeDef BMI088_CalibrateAccel(SPI_HandleTypeDef *hspi, uint16_t num_samples) {
    if (num_samples == 0) return HAL_ERROR;

    BMI088_Raw_t raw;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        if (BMI088_ReadRaw(hspi, &raw) != HAL_OK) {
            return HAL_ERROR;
        }

        // Переводим в G (ускорение свободного падения)
        sum_x += raw.Accel_X / 5461.33f;
        sum_y += raw.Accel_Y / 5461.33f;
        sum_z += raw.Accel_Z / 5461.33f;

        HAL_Delay(10);
    }

    acc_offset_x = sum_x / num_samples;
    acc_offset_y = sum_y / num_samples;

    // ВАЖНО: Вычитаем 1.0G из оси Z, так как гравитация тянет вниз
    // Предполагается, что датчик установлен ровно и смотрит маркировкой вверх.
    acc_offset_z = (sum_z / num_samples) - 1.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadRaw(SPI_HandleTypeDef *hspi, BMI088_Raw_t *raw) {
    uint8_t acc_data[6];
    uint8_t gyr_data[6];
    uint8_t temp_data[2];
    HAL_StatusTypeDef status;

    status = BMI088_ReadReg_Acc(hspi, BMI088_ACC_DATA, acc_data, 6);
    if (status != HAL_OK) return status;

    status = BMI088_ReadReg_Gyr(hspi, BMI088_GYR_DATA, gyr_data, 6);
    if (status != HAL_OK) return status;

    status = BMI088_ReadReg_Acc(hspi, BMI088_ACC_TEMP_DATA, temp_data, 2);
    if (status != HAL_OK) return status;

    raw->Accel_X = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    raw->Accel_Y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    raw->Accel_Z = (int16_t)((acc_data[5] << 8) | acc_data[4]);

    raw->Gyro_X  = (int16_t)((gyr_data[1] << 8) | gyr_data[0]);
    raw->Gyro_Y  = (int16_t)((gyr_data[3] << 8) | gyr_data[2]);
    raw->Gyro_Z  = (int16_t)((gyr_data[5] << 8) | gyr_data[4]);

    uint16_t temp_uint = (temp_data[0] << 3) | (temp_data[1] >> 5);
    if (temp_uint > 1023) {
        raw->Temp_Raw = (int16_t)temp_uint - 2048;
    } else {
        raw->Temp_Raw = (int16_t)temp_uint;
    }

    return HAL_OK;
}

void BMI088_ToPhysical(const BMI088_Raw_t *raw, BMI088_Physical_t *phys) {
    // Применяем калибровку акселерометра
    phys->Accel_X = (raw->Accel_X / 5461.33f) - acc_offset_x;
    phys->Accel_Y = (raw->Accel_Y / 5461.33f) - acc_offset_y;
    phys->Accel_Z = (raw->Accel_Z / 5461.33f) - acc_offset_z;

    // Применяем калибровку гироскопа (остается как было)
    phys->Gyro_X = (raw->Gyro_X / 131.072f) - gyro_offset_x;
    phys->Gyro_Y = (raw->Gyro_Y / 131.072f) - gyro_offset_y;
    phys->Gyro_Z = (raw->Gyro_Z / 131.072f) - gyro_offset_z;

    phys->Temp_C = (raw->Temp_Raw * 0.125f) + 23.0f;
}
