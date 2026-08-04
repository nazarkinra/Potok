#include <SPI.h>
#include <LoRa.h>

// Пины подключения
const int csPin = 10;    
const int resetPin = 9;  
const int irqPin = 2;    

// Наш ID, который мы ожидаем слушать
const uint16_t MY_TARGET_ID = 0xBBB9;

// Структура телеметрии 
struct __attribute__((packed)) TelemetryPacket_t {
  uint16_t node_id;
  int16_t  accel_x;
  int16_t  accel_y;
  int16_t  accel_z;
  int16_t  gyro_x;
  int16_t  gyro_y;
  int16_t  gyro_z;
  int16_t  mag_x;
  int16_t  mag_y;
  int16_t  mag_z;
  uint8_t  pressure[3];
  int16_t  temperature;
  uint8_t  state_flags;
  uint8_t  checksum;       // <--- Контрольная сумма
};

// Структура команды 
struct __attribute__((packed)) CommandPacket_t {
  uint16_t target_id;
  uint8_t  cmd_id;
  int16_t  param;
  uint8_t  checksum;       // <--- Контрольная сумма
};

// Функция setup() остается без изменений
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("LoRa Ground Station Init..."));

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(441E6)) {
    Serial.println(F("LoRa init failed!"));
    while (1);
  }

  LoRa.setSpreadingFactor(7);      
  LoRa.setSignalBandwidth(125E3);  
  LoRa.setCodingRate4(5);          
  LoRa.enableCrc();                
  LoRa.setSyncWord(0x12);          
  LoRa.setTxPower(20);             

  Serial.println(F("Ready. Waiting for telemetry..."));
}

void loop() {
  // --- 1. Проверяем данные от STM32 по LoRa ---
  int packetSize = LoRa.parsePacket();
  
  if (packetSize == sizeof(TelemetryPacket_t)) {
    TelemetryPacket_t rx_pkt;
    
    // Считываем пакет 
    uint8_t* ptr = (uint8_t*)&rx_pkt;
    for (size_t i = 0; i < sizeof(TelemetryPacket_t); i++) {
      ptr[i] = LoRa.read();
    }

    // Рассчитываем контрольную сумму полученных данных
    uint8_t calc_crc = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket_t) - 1; i++) {
      calc_crc ^= ptr[i];
    }

    // ПРОВЕРКА 1: Совпадает ли контрольная сумма?
    // ПРОВЕРКА 2: Наш ли это планер?
    if (calc_crc == rx_pkt.checksum && rx_pkt.node_id == MY_TARGET_ID) {
      
      uint32_t press_int = ((uint32_t)rx_pkt.pressure[0] << 16) | 
                           ((uint32_t)rx_pkt.pressure[1] << 8) | 
                           rx_pkt.pressure[2];

      // Выводим ВСЮ полученную телеметрию 
      Serial.print(F("Node: ")); Serial.print(rx_pkt.node_id, HEX);
      
      Serial.print(F(" | Acc: ")); 
      Serial.print(rx_pkt.accel_x / 100.0); Serial.print(F(","));
      Serial.print(rx_pkt.accel_y / 100.0); Serial.print(F(","));
      Serial.print(rx_pkt.accel_z / 100.0);
      
      Serial.print(F(" | Gyr: ")); 
      Serial.print(rx_pkt.gyro_x / 100.0); Serial.print(F(","));
      Serial.print(rx_pkt.gyro_y / 100.0); Serial.print(F(","));
      Serial.print(rx_pkt.gyro_z / 100.0);
      
      Serial.print(F(" | Mag: ")); 
      Serial.print(rx_pkt.mag_x); Serial.print(F(","));
      Serial.print(rx_pkt.mag_y); Serial.print(F(","));
      Serial.print(rx_pkt.mag_z);
      
      Serial.print(F(" | Press: ")); Serial.print(press_int / 100.0); Serial.print(F(" Pa"));
      Serial.print(F(" | Temp: ")); Serial.print(rx_pkt.temperature / 100.0); Serial.print(F(" C"));
      
      Serial.print(F(" | Flags: 0x")); Serial.println(rx_pkt.state_flags, HEX);

    } else if (rx_pkt.node_id != MY_TARGET_ID) {
        // Пакет от чужого устройства - тихо игнорируем
        // Можно раскомментировать строку ниже для отладки, если кто-то рядом тоже вещает:
        // Serial.println(F("Ignored: Foreign Node ID"));
    } else {
        Serial.println(F("Error: Checksum mismatch!"));
    }
    
  } else if (packetSize > 0) {
    while (LoRa.available()) LoRa.read();
    // Пакет неверного размера - это точно не наша телеметрия, игнорируем
  }

  // --- 2. Проверяем команды от ПК по Serial ---
  // (Здесь код отправки команд, если он еще нужен)
}
