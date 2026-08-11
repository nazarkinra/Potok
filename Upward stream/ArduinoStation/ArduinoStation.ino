#include <SPI.h>
#include <LoRa.h>

// Пины подключения
const int csPin = 10;    
const int resetPin = 9;  
const int irqPin = 2;    

// Наш ID, который мы ожидаем слушать
const uint16_t MY_TARGET_ID = 0xBBB9;

// Структура телеметрии (Обновленная: altitude вместо pressure)
typedef struct __attribute__((packed)) {
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
  int16_t  altitude; // <-- Теперь здесь высота в дециметрах
  int16_t  temperature;
  uint16_t photo;
  uint8_t  state_flags;
  uint8_t  checksum;
} TelemetryPacket_t;

// Структура команды
typedef struct __attribute__((packed)) {
  uint16_t target_id;
  uint8_t  cmd_id;
  int16_t  param;
  uint8_t  checksum;
} CommandPacket_t;

// Функция setup()
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("LoRa Ground Station Init..."));

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(441E6)) {
    Serial.println(F("LoRa init failed!"));
    while (1);
  }

  LoRa.setSpreadingFactor(7);      
  LoRa.setSignalBandwidth(250E3);  
  LoRa.setCodingRate4(5);          
  LoRa.enableCrc();                
  LoRa.setSyncWord(0x12);          
  
  // Мощность снижена до 10, чтобы избежать просадок напряжения (Brownout)
  LoRa.setTxPower(15);             

  Serial.println(F("Ready. Waiting for telemetry & logs..."));
}

void loop() {
  // --- 1. Проверяем данные от STM32 по LoRa ---
  int packetSize = LoRa.parsePacket();
  
  // Любой валидный пакет от нас (лог или телеметрия) должен весить хотя бы 2 байта (размер ID)
  if (packetSize >= 2) {
    uint8_t buffer[256];
    int len = 0;
    
    // Считываем весь пакет в буфер
    while (LoRa.available() && len < 256) {
      buffer[len++] = LoRa.read();
    }

    // --- ПЕРВАЯ ПРОВЕРКА: НАШ ЛИ ЭТО ПЛАНЕР? ---
    // Читаем первые 2 байта буфера как uint16_t
    uint16_t received_id = *((uint16_t*)buffer);
    
    if (received_id == MY_TARGET_ID) {
      // ВАРИАНТ А: Это бинарная телеметрия
      if (len == sizeof(TelemetryPacket_t)) {
        TelemetryPacket_t* rx_pkt = (TelemetryPacket_t*)buffer;
        
        // Проверяем контрольную сумму
        uint8_t calc_crc = 0;
        for (size_t i = 0; i < sizeof(TelemetryPacket_t) - 1; i++) {
          calc_crc ^= buffer[i];
        }

        if (calc_crc == rx_pkt->checksum) {
          // Выводим телеметрию для Python
          Serial.print(F("Node: ")); Serial.print(rx_pkt->node_id, HEX);
          
          Serial.print(F(" | Acc: ")); 
          Serial.print(rx_pkt->accel_x / 100.0); Serial.print(F(","));
          Serial.print(rx_pkt->accel_y / 100.0); Serial.print(F(","));
          Serial.print(rx_pkt->accel_z / 100.0);
          
          Serial.print(F(" | Gyr: ")); 
          Serial.print(rx_pkt->gyro_x / 100.0); Serial.print(F(","));
          Serial.print(rx_pkt->gyro_y / 100.0); Serial.print(F(","));
          Serial.print(rx_pkt->gyro_z / 100.0);
          
          Serial.print(F(" | Mag: ")); 
          Serial.print(rx_pkt->mag_x); Serial.print(F(","));
          Serial.print(rx_pkt->mag_y); Serial.print(F(","));
          Serial.print(rx_pkt->mag_z);
          
          // Выводим высоту (делим на 10.0, так как STM32 шлет дециметры)
          Serial.print(F(" | Alt: ")); Serial.print(rx_pkt->altitude / 10.0); Serial.print(F(" m"));
          Serial.print(F(" | Temp: ")); Serial.print(rx_pkt->temperature / 100.0); Serial.print(F(" C"));
          Serial.print(F(" | Photo: ")); Serial.print(rx_pkt->photo);
          
          Serial.print(F(" | Flags: 0x")); Serial.println(rx_pkt->state_flags, HEX);
        } else {
          Serial.println(F("Error: Telemetry Checksum mismatch!"));
        }
      } 
      // ВАРИАНТ Б: Это текстовый лог (проверяем по префиксу "LOG:")
      else if (len >= 6 && buffer[2] == 'L' && buffer[3] == 'O' && buffer[4] == 'G') {
        buffer[len] = '\0'; // Обезопасим строку нуль-терминатором
        
        Serial.print(F("Node: ")); 
        Serial.print(received_id, HEX);
        Serial.print(F(" | "));
        
        // Печатаем саму строку, пропуская первые 2 байта ID
        Serial.println((char*)(buffer + 2));
      } 
      else {
        Serial.print(F("Unknown payload size from our node: "));
        Serial.println(len);
      }
    } 
    
  } else if (packetSize > 0) {
    // Вычитываем из буфера обрывки или случайные байты, чтобы очистить FIFO
    while (LoRa.available()) LoRa.read();
  }

  // --- 2. Проверяем команды от ПК по Serial ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Ожидаем текстовую команду в формате: "CMD <cmd_id> <param>"
    if (input.startsWith("CMD ")) {
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > 0) {
        // Парсим команду и параметр
        uint8_t cmd_id = input.substring(firstSpace + 1, secondSpace).toInt();
        int16_t param = input.substring(secondSpace + 1).toInt();

        // Заполняем структуру пакета
        CommandPacket_t tx_cmd;
        tx_cmd.target_id = MY_TARGET_ID; // 0xBBB9
        tx_cmd.cmd_id = cmd_id;
        tx_cmd.param = param;

        // Рассчитываем контрольную сумму перед отправкой (XOR всех байт)
        uint8_t calc_crc = 0;
        uint8_t *ptr = (uint8_t*)&tx_cmd;
        for (size_t i = 0; i < sizeof(CommandPacket_t) - 1; i++) {
          calc_crc ^= ptr[i];
        }
        tx_cmd.checksum = calc_crc;

        // Отправляем пакет по LoRa
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&tx_cmd, sizeof(CommandPacket_t));
        LoRa.endPacket();

        // ВНИМАНИЕ: Вызова LoRa.receive() здесь быть НЕ ДОЛЖНО, 
        // иначе будет конфликт с LoRa.parsePacket() в начале цикла!

        // Выводим подтверждение в лог
        Serial.print(F("Node: "));
        Serial.print(MY_TARGET_ID, HEX);
        Serial.print(F(" | LOG: Sent CMD 0x"));
        Serial.print(cmd_id, HEX);
        Serial.print(F(" with Param "));
        Serial.println(param);
      }
    }
  }
}
