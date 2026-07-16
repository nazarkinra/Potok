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

  // Параметры для стабильной передачи
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(125E3);  // BW 125 кГц
  LoRa.setCodingRate4(5);          // CR 4/5
  LoRa.enableCrc();                // CRC ON
  LoRa.setSyncWord(0x12);          // Публичная сеть (0x34 для приватной)
  LoRa.setTxPower(5);              // Мощность 17 дБм (уточните для вашего модуля)

  Serial.println(F("Ready. Enter text to send via LoRa:"));
}

void loop() {
  // Если есть данные от Serial
  if (Serial.available() > 0) {
    // Читаем строку до символа '\n' (перевод строки)
    String message = Serial.readStringUntil('\n');
    
    // Удаляем возможный символ '\r' в конце (если используется)
    message.trim();
    
    // Если строка не пустая
    if (message.length() > 0) {
      Serial.print(F("TX: "));
      Serial.println(message);
      
      // Отправка через LoRa
      LoRa.beginPacket();
      LoRa.print(message);
      int status = LoRa.endPacket();
      
      if (status > 0) {
        Serial.println(F("Sent OK"));
      } else {
        Serial.println(F("TX Error"));
      }
    }
  }
}
