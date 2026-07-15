#ifndef LORA278_H
#define LORA278_H

#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 🔧 Пины (совпадают с твоим проектом)
#define LORA_NSS_PORT   GPIOB
#define LORA_NSS_PIN    GPIO_PIN_13
#define LORA_RST_PORT   GPIOB
#define LORA_RST_PIN    GPIO_PIN_5
#define LORA_DIO0_PORT  GPIOA
#define LORA_DIO0_PIN   GPIO_PIN_0

// 📜 Регистры (1:1 как в Arduino LoRa.h)
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_PKT_SNR_VALUE        0x19
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// ⚙️ Режимы
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// 📡 Па и IRQ
#define PA_BOOST                 0x80
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK         0x40

#define RF_MID_BAND_THRESHOLD    525000000UL
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164
#define MAX_PKT_LENGTH           255
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37

// 📦 Статусы
typedef enum {
    LORA_OK = 0,
    LORA_BUSY,
    LORA_TIMEOUT,
    LORA_CRC_ERROR
} LoRa_Status_t;

// 🚀 Инициализация и ядро
bool LoRa_Init(void);
bool LoRa_Begin(long frequency_hz);
void LoRa_End(void);

// 📤 Отправка
bool LoRa_BeginPacket(int implicitHeader);
size_t LoRa_Write(const uint8_t *buffer, size_t size);
int  LoRa_EndPacket(bool async);

// 📥 Приём
int  LoRa_ParsePacket(int size);
int  LoRa_Available(void);
int  LoRa_Read(void);
int  LoRa_PacketRssi(void);
float LoRa_PacketSnr(void);
// Callback для асинхронного приёма (аналог LoRa.onReceive())
void LoRa_OnReceive(void(*callback)(int));
// Запуск непрерывного приёма с DIO0 (аналог LoRa.receive())
void LoRa_Receive(int size);
// Обработчик прерывания — вызывать из HAL_GPIO_EXTI_Callback()
void LoRa_HandleDio0Rise(void);

// ⚙️ Конфигурация
void LoRa_Idle(void);
void LoRa_Sleep(void);
void LoRa_SetTxPower(int level, int outputPin); // 0=RFO, 1=PA_BOOST
void LoRa_SetFrequency(long frequency_hz);
void LoRa_SetSpreadingFactor(int sf);
void LoRa_SetSignalBandwidth(long sbw);
void LoRa_SetCodingRate4(int denominator);
void LoRa_SetPreambleLength(long length);
void LoRa_SetSyncWord(int sw);
void LoRa_EnableCrc(void);
void LoRa_DisableCrc(void);

// 🔽 Низкоуровневый SPI (атомарный)
uint8_t LoRa_ReadReg(uint8_t addr);
void    LoRa_WriteReg(uint8_t addr, uint8_t val);

#endif
