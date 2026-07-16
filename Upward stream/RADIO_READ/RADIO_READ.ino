#include <SPI.h>
#include <LoRa.h>

// Пины подключения (те же, что и для передатчика)
const int csPin = 10;    // NSS / CS
const int resetPin = 9;  // RESET
const int irqPin = 2;    // DIO0

void setup() {
  Serial.begin(9600);
  Serial.println(F("LoRa Receiver initializing..."));

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println(F("Ошибка инициализации LoRa! Проверьте подключение."));
    while (1);
  }

  // ⚠️ Параметры ДОЛЖНЫ совпадать с передатчиком (E22-400MM22S)
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(125E3);  // BW 125 кГц
  LoRa.enableCrc();                // CRC ON
  LoRa.setSyncWord(0x12);          // SyncWord (стандарт LoRaWAN/public)

  Serial.println(F("Приёмник готов. Ожидание пакетов на 433 МГц..."));
}

void loop() {
  // Проверяем, пришёл ли пакет
  int packetSize = LoRa.parsePacket();
  
  if (packetSize > 0) {
    // Читаем полезные данные посимвольно (без String → 0 фрагментации SRAM)
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
  }
}
