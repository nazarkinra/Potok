#include <SPI.h>
#include <LoRa.h>

// Пины подключения
const int csPin = 10;    // NSS / CS
const int resetPin = 9;  // RESET
const int irqPin = 2;    // DIO0

void setup() {
  Serial.begin(9600);
  Serial.println(F("LoRa TX init..."));

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println(F("LoRa init failed!"));
    while (1);
  }

  // ⚠️ Параметры для стабильной передачи
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(125E3);  // BW 125 кГц
  LoRa.setCodingRate4(5);          // CR 4/5
  LoRa.enableCrc();                // CRC ON
  LoRa.setSyncWord(0x12);          // Публичная сеть (0x34 для приватной)
  LoRa.setTxPower(5);             // Мощность 17 дБм

  Serial.println(F("Ready. Sending every 2s..."));
}

void loop() {
  // Статический буфер → 0 фрагментации SRAM
  char payload[64];
  snprintf(payload, sizeof(payload), "hello_by_nano_TX | T=%lu", millis());

  Serial.print(F("TX: "));
  Serial.println(payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  int status = LoRa.endPacket();

  if (status > 0) Serial.println(F("Sent OK"));
  else Serial.println(F("TX Error"));

  delay(2000);
}
