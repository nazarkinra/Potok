#include <SPI.h>
#include <LoRa.h>

// Пины подключения
const int csPin = 10;    // NSS / CS
const int resetPin = 9;  // RESET
const int irqPin = 2;    // DIO0

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("LoRa Ground Station Init..."));

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println(F("LoRa init failed!"));
    while (1);
  }

  // Параметры (должны совпадать со станцией)
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(125E3);  // BW 125 кГц
  LoRa.setCodingRate4(5);          // CR 4/5
  LoRa.enableCrc();                // CRC ON
  LoRa.setSyncWord(0x12);          // SyncWord
  LoRa.setTxPower(17);             // Мощность

  Serial.println(F("Ready. Waiting for telemetry and commands..."));
}

void loop() {
  // 1. Проверяем данные от STM32 по LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    // Выводим полученную телеметрию в Serial
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
  }

  // 2. Проверяем команды от ПК по Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      // Отправляем команду на STM32 через LoRa
      LoRa.beginPacket();
      LoRa.print(command);
      int status = LoRa.endPacket();

      if (status == 0) {
        Serial.println(F("TX Error"));
      }
    }
  }
}
