import serial
import time
import sys

# Порт на Ubuntu 24
SERIAL_PORT = '/dev/ttyACM0' # Измени на /dev/ttyACM0, если нужно
BAUD_RATE = 9600           # Скорость, настроенная в STM32
OUTPUT_FILE = 'glider_telemetry.csv'

def read_sd_dump():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Слушаем порт {SERIAL_PORT}...")
    except Exception as e:
        print(f"Ошибка открытия порта: {e}")
        sys.exit(1)

    recording = False
    
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        while True:
            try:
                # Читаем строку и декодируем
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if "--- SD DUMP START ---" in line:
                    print(">>> Начало приема дампа с SD карты...")
                    recording = True
                    # Записываем заголовки столбцов для удобства анализа в Julius
                    f.write("node_id,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,altitude,temperature,photo,state_flags\n")
                    continue

                if "--- SD DUMP END ---" in line:
                    print(f">>> Прием окончен. Телеметрия сохранена в файл: {OUTPUT_FILE}")
                    break

                if recording:
                    # Записываем сырую CSV строку с телеметрией в файл
                    f.write(line + '\n')
                    print(f"Принято: {line}")

            except KeyboardInterrupt:
                print("\nЧтение прервано.")
                break

    ser.close()

if __name__ == '__main__':
    read_sd_dump()
