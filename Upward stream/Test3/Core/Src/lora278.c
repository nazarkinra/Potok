#include "lora278.h"
#include "mcp23017.h"
#include "gpio.h"
#include <math.h>

extern SPI_HandleTypeDef hspi2;

int  LoRa_GetSpreadingFactor(void);
long LoRa_GetSignalBandwidth(void);

static inline void LoRa_SPI_Begin(void) {
    MCP23017_WritePin(LORA_NSS_PIN_MCP, 0);
    for(volatile int i=0; i<40; i++);
}
static inline void LoRa_SPI_End(void) {
    for(volatile int i=0; i<40; i++);
    MCP23017_WritePin(LORA_NSS_PIN_MCP, 1);
}

static void (*_onReceiveCb)(int) = NULL;

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t rx = 0;
    uint8_t tx_addr = addr & 0x7F;
    uint8_t tx_dummy = 0x00;

    LoRa_SPI_Begin();
    HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 10);
    HAL_SPI_TransmitReceive(&hspi2, &tx_dummy, &rx, 1, 10);
    LoRa_SPI_End();

    return rx;
}

void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t tx_addr = addr | 0x80;
    uint8_t tx_data = val;
    uint8_t rx_dummy;

    LoRa_SPI_Begin();
    HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 10);
    HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_dummy, 1, 10);
    LoRa_SPI_End();
}

void LoRa_Sleep(void) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}
void LoRa_Idle(void) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

static void LoRa_SetLdoFlag(void) {
    long symbolDuration = 1000 / ( (long)LoRa_GetSignalBandwidth() / (1L << LoRa_GetSpreadingFactor()) );
    uint8_t config3 = LoRa_ReadReg(REG_MODEM_CONFIG_3);
    if (symbolDuration > 16) {
        config3 |= 0x08;
    } else {
        config3 &= ~0x08;
    }
    LoRa_WriteReg(REG_MODEM_CONFIG_3, config3);
}

void LoRa_SetTxPower(int level, int outputPin) {
    if (outputPin == 0) {
        if (level < 0) level = 0;
        else if (level > 14) level = 14;
        LoRa_WriteReg(REG_PA_CONFIG, 0x70 | level);
    } else {
        if (level > 17) {
            if (level > 20) level = 20;
            level -= 3;
            LoRa_WriteReg(REG_PA_DAC, 0x87);
            LoRa_WriteReg(REG_OCP, 0x20 | ((140 - 45) / 5));
        } else {
            if (level < 2) level = 2;
            LoRa_WriteReg(REG_PA_DAC, 0x84);
            LoRa_WriteReg(REG_OCP, 0x20 | ((100 - 45) / 5));
        }
        LoRa_WriteReg(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRa_SetFrequency(long frequency_hz) {
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000;
    LoRa_WriteReg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    LoRa_WriteReg(REG_FRF_MID, (frf >> 8)  & 0xFF);
    LoRa_WriteReg(REG_FRF_LSB, frf & 0xFF);
}

int LoRa_GetSpreadingFactor(void) {
    return LoRa_ReadReg(REG_MODEM_CONFIG_2) >> 4;
}

void LoRa_SetSpreadingFactor(int sf) {
    if (sf < 6) sf = 6; else if (sf > 12) sf = 12;
    if (sf == 6) {
        LoRa_WriteReg(REG_DETECTION_OPTIMIZE, 0xC5);
        LoRa_WriteReg(REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        LoRa_WriteReg(REG_DETECTION_OPTIMIZE, 0xC3);
        LoRa_WriteReg(REG_DETECTION_THRESHOLD, 0x0A);
    }
    uint8_t config2 = LoRa_ReadReg(REG_MODEM_CONFIG_2);
    LoRa_WriteReg(REG_MODEM_CONFIG_2, (config2 & 0x0F) | ((sf << 4) & 0xF0));
    LoRa_SetLdoFlag();
}

long LoRa_GetSignalBandwidth(void) {
    uint8_t bw = (LoRa_ReadReg(REG_MODEM_CONFIG_1) >> 4);
    const long bws[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
    return (bw < 10) ? bws[bw] : -1;
}

void LoRa_SetSignalBandwidth(long sbw) {
    int bw = 0;
    if (sbw <= 7800) bw = 0;
    else if (sbw <= 10400) bw = 1;
    else if (sbw <= 15600) bw = 2;
    else if (sbw <= 20800) bw = 3;
    else if (sbw <= 31250) bw = 4;
    else if (sbw <= 41700) bw = 5;
    else if (sbw <= 62500) bw = 6;
    else if (sbw <= 125000) bw = 7;
    else if (sbw <= 250000) bw = 8;
    else bw = 9;

    uint8_t config1 = LoRa_ReadReg(REG_MODEM_CONFIG_1);
    LoRa_WriteReg(REG_MODEM_CONFIG_1, (config1 & 0x0F) | (bw << 4));
    LoRa_SetLdoFlag();
}

void LoRa_SetCodingRate4(int denominator) {
    if (denominator < 5) denominator = 5; else if (denominator > 8) denominator = 8;
    int cr = denominator - 4;
    uint8_t config1 = LoRa_ReadReg(REG_MODEM_CONFIG_1);
    LoRa_WriteReg(REG_MODEM_CONFIG_1, (config1 & 0xF1) | (cr << 1));
}

void LoRa_SetPreambleLength(long length) {
    LoRa_WriteReg(REG_PREAMBLE_MSB, (length >> 8) & 0xFF);
    LoRa_WriteReg(REG_PREAMBLE_LSB, length & 0xFF);
}

void LoRa_SetSyncWord(int sw) {
    LoRa_WriteReg(REG_SYNC_WORD, sw);
}

void LoRa_EnableCrc(void) {
    LoRa_WriteReg(REG_MODEM_CONFIG_2, LoRa_ReadReg(REG_MODEM_CONFIG_2) | 0x04);
}
void LoRa_DisableCrc(void) {
    LoRa_WriteReg(REG_MODEM_CONFIG_2, LoRa_ReadReg(REG_MODEM_CONFIG_2) & 0xFB);
}

bool LoRa_Begin(long frequency_hz) {
    MCP23017_WritePin(LORA_RST_PIN_MCP, 0);
    HAL_Delay(10);
    MCP23017_WritePin(LORA_RST_PIN_MCP, 1);
    HAL_Delay(10);

    if (LoRa_ReadReg(REG_VERSION) != 0x12) return false;

    LoRa_Sleep();
    LoRa_SetFrequency(frequency_hz);
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0);
    LoRa_WriteReg(REG_LNA, LoRa_ReadReg(REG_LNA) | 0x03);
    LoRa_WriteReg(REG_MODEM_CONFIG_3, 0x04);
    LoRa_SetTxPower(17, 1);
    LoRa_Idle();
    return true;
}

bool LoRa_BeginPacket(int implicitHeader) {
    LoRa_Idle();
    if (implicitHeader) {
        LoRa_WriteReg(REG_MODEM_CONFIG_1, LoRa_ReadReg(REG_MODEM_CONFIG_1) | 0x01);
    } else {
        LoRa_WriteReg(REG_MODEM_CONFIG_1, LoRa_ReadReg(REG_MODEM_CONFIG_1) & 0xFE);
    }
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0);
    LoRa_WriteReg(REG_PAYLOAD_LENGTH, 0);
    return true;
}

size_t LoRa_Write(const uint8_t *buffer, size_t size) {
    if (size == 0) return 0;
    int currentLength = LoRa_ReadReg(REG_PAYLOAD_LENGTH);
    if ((currentLength + size) > MAX_PKT_LENGTH) size = MAX_PKT_LENGTH - currentLength;

    uint8_t spi_buf[MAX_PKT_LENGTH + 1];
    spi_buf[0] = 0x80;
    memcpy(&spi_buf[1], buffer, size);

    LoRa_SPI_Begin();
    HAL_SPI_Transmit(&hspi2, spi_buf, size + 1, 50);
    LoRa_SPI_End();

    LoRa_WriteReg(REG_PAYLOAD_LENGTH, currentLength + size);
    return size;
}

int LoRa_EndPacket(bool async) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    if (!async) {
        uint32_t start = HAL_GetTick();
        while ((LoRa_ReadReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
            if (HAL_GetTick() - start > 2000) return 0;
            HAL_Delay(1);
        }
        LoRa_WriteReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    return 1;
}

static int _implicitHeaderMode = 0;
static int _packetIndex = 0;

int LoRa_ParsePacket(int size) {
    int packetLength = 0;
    int irqFlags = LoRa_ReadReg(REG_IRQ_FLAGS);

    if (size > 0) {
        _implicitHeaderMode = 1;
        LoRa_WriteReg(REG_PAYLOAD_LENGTH, size & 0xFF);
    } else {
        _implicitHeaderMode = 0;
    }

    LoRa_WriteReg(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && !(irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
        _packetIndex = 0;
        if (_implicitHeaderMode) {
            packetLength = LoRa_ReadReg(REG_PAYLOAD_LENGTH);
        } else {
            packetLength = LoRa_ReadReg(REG_RX_NB_BYTES);
        }
        LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_CURRENT_ADDR));
        LoRa_Idle();
    } else if (LoRa_ReadReg(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0);
        LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
    return packetLength;
}

int LoRa_Available(void) {
    return (LoRa_ReadReg(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_Read(void) {
    if (!LoRa_Available()) return -1;
    _packetIndex++;
    return LoRa_ReadReg(REG_FIFO);
}

int LoRa_PacketRssi(void) {
    return (LoRa_ReadReg(REG_PKT_RSSI_VALUE) - (LoRa_ReadReg(REG_FRF_MSB) < (RF_MID_BAND_THRESHOLD >> 16) ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRa_PacketSnr(void) {
    return ((int8_t)LoRa_ReadReg(REG_PKT_SNR_VALUE)) * 0.25f;
}

bool LoRa_Init(void) {
    MCP23017_WritePin(LORA_NSS_PIN_MCP, 1);
    return true;
}

void LoRa_End(void) {
    LoRa_Sleep();
}

void LoRa_OnReceive(void(*callback)(int)) {
    _onReceiveCb = callback;
    if (callback) {
        uint8_t dioMap = LoRa_ReadReg(REG_DIO_MAPPING_1) & 0x3F;
        LoRa_WriteReg(REG_DIO_MAPPING_1, dioMap);
    }
}

void LoRa_Receive(int size) {
    if (size > 0) {
        _implicitHeaderMode = 1;
        LoRa_WriteReg(REG_MODEM_CONFIG_1, LoRa_ReadReg(REG_MODEM_CONFIG_1) | 0x01);
        LoRa_WriteReg(REG_PAYLOAD_LENGTH, size & 0xFF);
    } else {
        _implicitHeaderMode = 0;
        LoRa_WriteReg(REG_MODEM_CONFIG_1, LoRa_ReadReg(REG_MODEM_CONFIG_1) & 0xFE);
    }
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa_HandleDio0Rise(void) {
    uint8_t irqFlags = LoRa_ReadReg(REG_IRQ_FLAGS);
    LoRa_WriteReg(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && !(irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
        _packetIndex = 0;
        int packetLength = _implicitHeaderMode ?
            LoRa_ReadReg(REG_PAYLOAD_LENGTH) :
            LoRa_ReadReg(REG_RX_NB_BYTES);
        LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_CURRENT_ADDR));
        if (_onReceiveCb) {
            _onReceiveCb(packetLength);
        }
    }
}
