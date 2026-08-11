import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re
import os
from datetime import datetime
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

# Установим стиль для графиков
plt.style.use('seaborn-v0_8-whitegrid')

LORA_FILE = "telemetry_20260808_150652.txt" 
SD_FILE = "glider_recovered_data.csv"

TELEMETRY_PATTERN = re.compile(
    r"\[(\d{2}:\d{2}:\d{2}\.\d{3})\].*?"
    r"Acc:\s*([-\d\.]+).*?"
    r"Alt:\s*([-\d\.]+)\s*m.*?"
    r"Photo:\s*(\d+)"
)

class SimpleMadgwick:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) 

    def update(self, g, a, dt):
        q = self.q
        if np.linalg.norm(a) == 0: return self.q
        a = a / np.linalg.norm(a) 
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - a[0],
            2*(q[0]*q[1] + q[2]*q[3]) - a[1],
            2*(0.5 - q[1]**2 - q[2]**2) - a[2]
        ])
        J = np.array([
            [-2*q[2],  2*q[3], -2*q[0],  2*q[1]],
            [ 2*q[1],  2*q[0],  2*q[3],  2*q[2]],
            [ 0,      -4*q[1], -4*q[2],  0     ]
        ])
        step = J.T.dot(f)
        step = step / np.linalg.norm(step)
        qDot = 0.5 * np.array([
            -q[1]*g[0] - q[2]*g[1] - q[3]*g[2],
             q[0]*g[0] + q[2]*g[2] - q[3]*g[1],
             q[0]*g[1] - q[1]*g[2] + q[3]*g[0],
             q[0]*g[2] + q[1]*g[1] - q[2]*g[0]
        ]) - self.beta * step
        q = q + qDot * dt
        self.q = q / np.linalg.norm(q)
        return self.q

def parse_lora(filename):
    data = []
    if not os.path.exists(filename): 
        return pd.DataFrame()
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            match = TELEMETRY_PATTERN.search(line)
            if match:
                dt = datetime.strptime(match.group(1), "%H:%M:%S.%f")
                data.append({
                    "Time": dt,
                    "Acc_X": float(match.group(2)),
                    "Alt": float(match.group(3)),
                    "Photo": int(match.group(4))
                })
    return pd.DataFrame(data)

def generate_plots():
    print(f"1. Чтение логов...")
    lora_df = parse_lora(LORA_FILE)
    if lora_df.empty:
        print(f"Ошибка: Не удалось прочитать логи LoRa.")
        return

    col_names = [
        "Node_ID", "Acc_X", "Acc_Y", "Acc_Z", 
        "Gyr_X", "Gyr_Y", "Gyr_Z", "Mag_X", "Mag_Y", "Mag_Z", 
        "Altitude", "Temperature", "Photo", "State_Flags"
    ]
    if not os.path.exists(SD_FILE):
        print(f"Ошибка: Файл {SD_FILE} не найден.")
        return
        
    sd_df = pd.read_csv(SD_FILE, header=None, names=col_names, on_bad_lines='skip', engine='python')
    for col in col_names[1:]:
        sd_df[col] = pd.to_numeric(sd_df[col], errors='coerce')
    sd_df = sd_df.dropna(subset=['Acc_Z']).reset_index(drop=True)
    
    sd_df['Acc_X'] /= 100.0; sd_df['Acc_Y'] /= 100.0; sd_df['Acc_Z'] /= 100.0
    sd_df['Gyr_X'] /= 100.0; sd_df['Gyr_Y'] /= 100.0; sd_df['Gyr_Z'] /= 100.0
    sd_df['Altitude'] /= 10.0
    sd_df['Flight_State'] = sd_df['State_Flags'].fillna(0).astype(int) // 16

    print("2. Синхронизация времени...")
    sd_df['Exact_Time'] = pd.NaT
    last_sd_idx, last_valid_match_idx = 0, -1
    
    for _, lora_row in lora_df.iterrows():
        search_window = sd_df.iloc[last_sd_idx : last_sd_idx + 100]
        if search_window.empty: break
        dist = np.abs(search_window['Altitude'] - lora_row['Alt']) * 10 + \
               np.abs(search_window['Photo'] - lora_row['Photo']) + \
               np.abs(search_window['Acc_X'] - lora_row['Acc_X']) * 10
        best_match_idx = dist.idxmin()
        sd_df.loc[best_match_idx, 'Exact_Time'] = lora_row['Time']
        last_sd_idx = best_match_idx + 1 
        last_valid_match_idx = best_match_idx

    sd_df['Exact_Time_num'] = sd_df['Exact_Time'].astype('int64').replace(-9223372036854775808, np.nan).interpolate(method='linear')
    sd_df['Time'] = pd.to_datetime(sd_df['Exact_Time_num'])
    
    if last_valid_match_idx != -1 and last_valid_match_idx < len(sd_df) - 1:
        last_time = sd_df.loc[last_valid_match_idx, 'Time']
        for i in range(last_valid_match_idx + 1, len(sd_df)):
            sd_df.loc[i, 'Time'] = last_time + pd.Timedelta(milliseconds=100 * (i - last_valid_match_idx))

    first_valid = sd_df['Exact_Time'].first_valid_index()
    if first_valid is not None: sd_df = sd_df.loc[first_valid:].reset_index(drop=True)
    sd_df['Time_s'] = (sd_df['Time'] - sd_df['Time'].iloc[0]).dt.total_seconds()

    print("3. Расчет скоростей...")
    df = sd_df[(sd_df['Altitude'] > -200) & (sd_df['Altitude'] < 2500)].copy()

    df['Acc_Mag'] = np.sqrt(df['Acc_X']**2 + df['Acc_Y']**2 + df['Acc_Z']**2)
    
    df['dt'] = df['Time_s'].diff().fillna(0.1)
    df.loc[df['dt'] <= 0, 'dt'] = 0.001
    
    df['Vert_Speed_Raw'] = df['Altitude'].diff() / df['dt']
    df.loc[df['Vert_Speed_Raw'].abs() > 100, 'Vert_Speed_Raw'] = np.nan
    df['Vert_Speed_Raw'] = df['Vert_Speed_Raw'].interpolate(method='linear').fillna(0)
    
    try:
        df['Vert_Speed_Smooth'] = savgol_filter(df['Vert_Speed_Raw'], window_length=31, polyorder=2)
    except Exception:
        df['Vert_Speed_Smooth'] = df['Vert_Speed_Raw'].rolling(window=15, center=True, min_periods=1).mean()
    
    madgwick = SimpleMadgwick(beta=1.0)
    acc0 = np.array([df['Acc_X'].iloc[0], df['Acc_Y'].iloc[0], df['Acc_Z'].iloc[0]])
    for _ in range(2000): madgwick.update(np.array([0,0,0]), acc0, 0.01)
    madgwick.beta = 0.1 

    vx_imu, vy_imu = [0.0], [0.0]

    for i in range(len(df)):
        dt = df['dt'].iloc[i]
        acc = np.array([df['Acc_X'].iloc[i], df['Acc_Y'].iloc[i], df['Acc_Z'].iloc[i]])
        gyr = np.array([df['Gyr_X'].iloc[i], df['Gyr_Y'].iloc[i], df['Gyr_Z'].iloc[i]]) * np.pi / 180.0
        
        q = madgwick.update(gyr, acc, dt)
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        
        if i > 0:
            global_acc = r.apply(acc) * 9.81
            global_acc[2] -= 9.81
            vx = vx_imu[-1] + global_acc[0] * dt
            vy = vy_imu[-1] + global_acc[1] * dt
            vx_imu.append(vx)
            vy_imu.append(vy)

    df['V_x'] = vx_imu
    df['V_y'] = vy_imu
    df['V_horiz_total'] = np.sqrt(df['V_x']**2 + df['V_y']**2)
    
    print("4. Генерация графиков по сегментам...")
    
    intervals = [
        ("overall", 0, df['Time_s'].max()),
        ("0_24.429", 0, 24.429),
        ("24.429_43.627", 24.429, 43.627),
        ("43.627_59.496", 43.627, 59.496),
        ("59.496_102.552", 59.496, 102.552),
        ("102.552_125.438", 102.552, 125.438)
    ]
    
    out_dir = "flight_plots"
    os.makedirs(out_dir, exist_ok=True)
    
    for name, t_start, t_end in intervals:
        sub_df = df[(df['Time_s'] >= t_start) & (df['Time_s'] <= t_end)]
        if sub_df.empty:
            continue
            
        t = sub_df['Time_s']
        
        # Настройка дашборда 3x2
        fig, axs = plt.subplots(3, 2, figsize=(16, 12))
        fig.suptitle(f'Телеметрия полета (интервал {t_start:.1f} - {t_end:.1f} сек)', fontsize=16)
        
        # 1. Ускорение по осям + результирующее
        ax = axs[0, 0]
        ax.plot(t, sub_df['Acc_X'], label='X')
        ax.plot(t, sub_df['Acc_Y'], label='Y')
        ax.plot(t, sub_df['Acc_Z'], label='Z')
        ax.plot(t, sub_df['Acc_Mag'], label='|Сумма|', color='k', linestyle='--')
        ax.set_title('Ускорение')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Ускорение, g')
        ax.legend(loc='upper right')
        
        # 2. Гироскоп по осям
        ax = axs[0, 1]
        ax.plot(t, sub_df['Gyr_X'], label='X')
        ax.plot(t, sub_df['Gyr_Y'], label='Y')
        ax.plot(t, sub_df['Gyr_Z'], label='Z')
        ax.set_title('Угловая скорость (Гироскоп)')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Угловая скорость, °/с')
        ax.legend(loc='upper right')

        # 3. Магнитометр по осям
        ax = axs[1, 0]
        ax.plot(t, sub_df['Mag_X'], label='X')
        ax.plot(t, sub_df['Mag_Y'], label='Y')
        ax.plot(t, sub_df['Mag_Z'], label='Z')
        ax.set_title('Магнитная индукция (Магнитометр)')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Магнитная индукция, мГс')
        ax.legend(loc='upper right')

        # 4. Высота
        ax = axs[1, 1]
        ax.plot(t, sub_df['Altitude'], color='purple')
        ax.set_title('Высота')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Высота, м')

        # 5. Вертикальная скорость
        ax = axs[2, 0]
        ax.plot(t, sub_df['Vert_Speed_Smooth'], color='red')
        ax.set_title('Вертикальная скорость (Vz)')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Вертикальная скорость, м/с')

        # 6. Горизонтальная скорость
        ax = axs[2, 1]
        ax.plot(t, sub_df['V_x'], label='Vx')
        ax.plot(t, sub_df['V_y'], label='Vy')
        ax.plot(t, sub_df['V_horiz_total'], label='|V гориз|', color='k', linestyle='--')
        ax.set_title('Горизонтальная скорость (по IMU)')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Горизонтальная скорость, м/с')
        ax.legend(loc='upper right')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        filename = os.path.join(out_dir, f'telemetry_{name}.png')
        plt.savefig(filename, dpi=150)
        plt.close()
        
    print(f"Графики сохранены в папку: {out_dir}")

if __name__ == "__main__":
    generate_plots()
