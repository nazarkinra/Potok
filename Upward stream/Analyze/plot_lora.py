import re
import pandas as pd
import matplotlib.pyplot as plt
import datetime

# --- НАСТРОЙКИ ---
# Укажи точное имя твоего файла (если файлы разделены, можно их склеить в один текст)
FILE_NAME = "telemetry_20260808_150652.txt"

# Регулярное выражение для поиска строк с телеметрией.
# Ищет строки вида: [15:07:11.117] Node: BBB9 | Acc: -0.10,0.59,0.79 | Gyr: ...
TELEMETRY_PATTERN = re.compile(
    r"\[(\d{2}:\d{2}:\d{2}\.\d{3})\].*?"
    r"Acc:\s*([-\d\.]+),([-\d\.]+),([-\d\.]+)\s*\|\s*"
    r"Gyr:\s*([-\d\.]+),([-\d\.]+),([-\d\.]+)\s*\|\s*"
    r"Mag:\s*([-\d\.]+),([-\d\.]+),([-\d\.]+)\s*\|\s*"
    r"Alt:\s*([-\d\.]+)\s*m\s*\|\s*"
    r"Temp:\s*([-\d\.]+)\s*C\s*\|\s*"
    r"Photo:\s*(\d+)"
)

def parse_telemetry(filename):
    data = []
    
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            match = TELEMETRY_PATTERN.search(line)
            if match:
                # Извлекаем время (для оси X)
                time_str = match.group(1)
                # Конвертируем в datetime (дата берется сегодняшняя, нам важно только время)
                dt = datetime.datetime.strptime(time_str, "%H:%M:%S.%f")
                
                # Сохраняем строку данных
                data.append({
                    "Time": dt,
                    "Acc_X": float(match.group(2)),
                    "Acc_Y": float(match.group(3)),
                    "Acc_Z": float(match.group(4)),
                    "Gyr_X": float(match.group(5)),
                    "Gyr_Y": float(match.group(6)),
                    "Gyr_Z": float(match.group(7)),
                    "Mag_X": float(match.group(8)),
                    "Mag_Y": float(match.group(9)),
                    "Mag_Z": float(match.group(10)),
                    "Alt": float(match.group(11)),
                    "Temp": float(match.group(12)),
                    "Photo": int(match.group(13))
                })
    return pd.DataFrame(data)

def plot_telemetry(df):
    if df.empty:
        print("Телеметрия не найдена! Проверь имя файла или формат данных.")
        return

    # Настраиваем стиль и размер графиков
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(12, 18), sharex=True)
    fig.suptitle("Анализ телеметрии планера", fontsize=16)

    # 1. Высота (Альтуда)
    axes[0].plot(df['Time'], df['Alt'], color='purple', linewidth=2)
    axes[0].set_ylabel('Высота (м)')
    axes[0].set_title('Профиль высоты (BMP388)')

    # 2. Акселерометр
    axes[1].plot(df['Time'], df['Acc_X'], label='X')
    axes[1].plot(df['Time'], df['Acc_Y'], label='Y')
    axes[1].plot(df['Time'], df['Acc_Z'], label='Z')
    axes[1].set_ylabel('Ускорение (G)')
    axes[1].set_title('Акселерометр')
    axes[1].legend(loc='upper right')

    # 3. Гироскоп
    axes[2].plot(df['Time'], df['Gyr_X'], label='X')
    axes[2].plot(df['Time'], df['Gyr_Y'], label='Y')
    axes[2].plot(df['Time'], df['Gyr_Z'], label='Z')
    axes[2].set_ylabel('Вращение (deg/s)')
    axes[2].set_title('Гироскоп')
    axes[2].legend(loc='upper right')

    # 4. Магнитометр
    axes[3].plot(df['Time'], df['Mag_X'], label='X')
    axes[3].plot(df['Time'], df['Mag_Y'], label='Y')
    axes[3].plot(df['Time'], df['Mag_Z'], label='Z')
    axes[3].set_ylabel('Магн. поле')
    axes[3].set_title('Магнитометр (LIS3MDL)')
    axes[3].legend(loc='upper right')

    # 5. Освещенность (Фоторезистор)
    axes[4].plot(df['Time'], df['Photo'], color='orange')
    axes[4].set_ylabel('Освещенность')
    axes[4].set_title('Датчик света (Photo)')

    # 6. Температура
    axes[5].plot(df['Time'], df['Temp'], color='red')
    axes[5].set_ylabel('Температура (C)')
    axes[5].set_xlabel('Время')
    axes[5].set_title('Температура внутри корпуса')

    # Форматирование оси X (чтобы время не слипалось)
    import matplotlib.dates as mdates
    axes[5].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    plt.xticks(rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.95) # Место для заголовка
    plt.show()

if __name__ == "__main__":
    # Запуск
    print(f"Парсинг файла {FILE_NAME}...")
    df_telemetry = parse_telemetry(FILE_NAME)
    
    print(f"Найдено {len(df_telemetry)} строк с телеметрией. Строим графики...")
    plot_telemetry(df_telemetry)
