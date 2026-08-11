import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import re
import os
from datetime import datetime
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

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

def analyze_and_sync_telemetry():
    print(f"1. Чтение эталонного времени из {LORA_FILE}...")
    lora_df = parse_lora(LORA_FILE)
    if lora_df.empty:
        print(f"Ошибка: Не удалось прочитать логи LoRa.")
        return

    print(f"2. Чтение данных с SD-карты из {SD_FILE}...")
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
    sd_df['Temperature'] /= 100.0
    sd_df['Flight_State'] = sd_df['State_Flags'].fillna(0).astype(int) // 16

    print("3. Синхронизация (Time Warping) логов...")
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

    print("4. Математические расчеты...")
    df = sd_df[(sd_df['Altitude'] > -200) & (sd_df['Altitude'] < 2500)].copy()

    df['Acc_Mag'] = np.sqrt(df['Acc_X']**2 + df['Acc_Y']**2 + df['Acc_Z']**2)
    
    # --- 4.1 ВЕРТИКАЛЬНАЯ СКОРОСТЬ (со сглаживанием) ---
    df['dt'] = df['Time_s'].diff().fillna(0.1)
    df.loc[df['dt'] <= 0, 'dt'] = 0.001
    
    df['Vert_Speed_Raw'] = df['Altitude'].diff() / df['dt']
    df.loc[df['Vert_Speed_Raw'].abs() > 100, 'Vert_Speed_Raw'] = np.nan
    df['Vert_Speed_Raw'] = df['Vert_Speed_Raw'].interpolate(method='linear').fillna(0)
    
    try:
        df['Vert_Speed_Smooth'] = savgol_filter(df['Vert_Speed_Raw'], window_length=31, polyorder=2)
    except Exception:
        df['Vert_Speed_Smooth'] = df['Vert_Speed_Raw'].rolling(window=15, center=True, min_periods=1).mean()
    
    # --- 4.2 ГОРИЗОНТАЛЬНАЯ СКОРОСТЬ (интегрирование IMU) ---
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
            # Ускорение в глобальной системе координат (в м/с^2)
            global_acc = r.apply(acc) * 9.81
            
            # Вычитаем гравитацию по глобальной оси Z
            global_acc[2] -= 9.81
            
            # Интегрируем горизонт
            vx = vx_imu[-1] + global_acc[0] * dt
            vy = vy_imu[-1] + global_acc[1] * dt
                
            vx_imu.append(vx)
            vy_imu.append(vy)

    df['V_x'] = vx_imu
    df['V_y'] = vy_imu
    df['V_horiz_total'] = np.sqrt(df['V_x']**2 + df['V_y']**2)
    
    # --- 4.3 ТЕМПЕРАТУРА ---
    df['Temp_Smooth'] = df['Temperature'].rolling(window=30, center=True, min_periods=1).mean()

    print(f"Готово! Строим графики по {len(df)} строкам.")

    fig = make_subplots(
        rows=10, cols=1, 
        vertical_spacing=0.02,
        subplot_titles=(
            "Высота (BMP388), метры", 
            "Вертикальная скорость, м/с",
            "Горизонтальная скорость (V_x, V_y), м/с",
            "Акселерометр (оси), G", 
            "Результирующее ускорение, G",
            "Гироскоп (BMI088), град/сек", 
            "Магнитометр (LIS3MDL)", 
            "Освещенность (Фоторезистор)", 
            "Температура внутри корпуса, °C", 
            "Стадия полета"
        )
    )

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Altitude'], name='Высота', line=dict(color='purple'), legend='legend'), row=1, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Vert_Speed_Raw'], name='Vz (сырая)', line=dict(color='lightgray', width=1), opacity=0.6, legend='legend2'), row=2, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Vert_Speed_Smooth'], name='Vz (сглаж.)', line=dict(color='red', width=2), legend='legend2'), row=2, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['V_x'], name='Vx', line=dict(color='blue'), legend='legend3'), row=3, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['V_y'], name='Vy', line=dict(color='green'), legend='legend3'), row=3, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['V_horiz_total'], name='|V_horiz|', line=dict(color='black', dash='dash'), legend='legend3'), row=3, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Acc_X'], name='Acc X', legend='legend4'), row=4, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Acc_Y'], name='Acc Y', legend='legend4'), row=4, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Acc_Z'], name='Acc Z', legend='legend4'), row=4, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Acc_Mag'], name='|Acc|', line=dict(color='brown'), legend='legend5'), row=5, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Gyr_X'], name='Gyr X', legend='legend6'), row=6, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Gyr_Y'], name='Gyr Y', legend='legend6'), row=6, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Gyr_Z'], name='Gyr Z', legend='legend6'), row=6, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Mag_X'], name='Mag X', legend='legend7'), row=7, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Mag_Y'], name='Mag Y', legend='legend7'), row=7, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Mag_Z'], name='Mag Z', legend='legend7'), row=7, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Photo'], name='Свет', line=dict(color='orange'), legend='legend8'), row=8, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Temperature'], name='Т (сырая)', line=dict(color='lightcoral', width=1), opacity=0.4, legend='legend9'), row=9, col=1)
    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Temp_Smooth'], name='Т (сглаж.)', line=dict(color='red', width=2), legend='legend9'), row=9, col=1)

    fig.add_trace(go.Scatter(x=df['Time_s'], y=df['Flight_State'], name='Режим', line=dict(color='black', shape='hv'), legend='legend10'), row=10, col=1)

    fig.update_layout(
        height=3500, 
        title_text="Синхронизированная телеметрия планера",
        hovermode="x unified",
        showlegend=True,
        legend=dict(y=0.985, x=1.01, xanchor="left", yanchor="top", title_text="Барометр"),
        legend2=dict(y=0.885, x=1.01, xanchor="left", yanchor="top", title_text="Верт. скорость"),
        legend3=dict(y=0.785, x=1.01, xanchor="left", yanchor="top", title_text="Гориз. скорость"),
        legend4=dict(y=0.685, x=1.01, xanchor="left", yanchor="top", title_text="Ускорение (оси)"),
        legend5=dict(y=0.585, x=1.01, xanchor="left", yanchor="top", title_text="Ускорение (рез)"),
        legend6=dict(y=0.485, x=1.01, xanchor="left", yanchor="top", title_text="Вращение"),
        legend7=dict(y=0.385, x=1.01, xanchor="left", yanchor="top", title_text="Магн. поле"),
        legend8=dict(y=0.285, x=1.01, xanchor="left", yanchor="top", title_text="Сенсор"),
        legend9=dict(y=0.185, x=1.01, xanchor="left", yanchor="top", title_text="Термометр"),
        legend10=dict(y=0.085, x=1.01, xanchor="left", yanchor="bottom", title_text="Статус")
    )
    
    for idx, name in enumerate(['legend', 'legend2', 'legend2', 'legend3', 'legend3', 'legend3', 'legend4', 'legend4', 'legend4', 'legend5', 'legend6', 'legend6', 'legend6', 'legend7', 'legend7', 'legend7', 'legend8', 'legend9', 'legend9', 'legend10']):
        fig.data[idx].legend = name

    fig.update_xaxes(matches='x', showticklabels=True, title_text="Абсолютное время полета (секунды)", nticks=30, showgrid=True, gridcolor='lightgray', minor=dict(showgrid=True, gridcolor='whitesmoke'))
    fig.update_yaxes(nticks=10, showgrid=True, gridcolor='lightgray', minor=dict(showgrid=True, gridcolor='whitesmoke'))
    fig.update_yaxes(tickvals=[0, 1, 2, 3, 4, 5], ticktext=['0:IDLE', '1:WAIT', '2:DROP', '3:REC', '4:GLIDE', '5:PARACHUTE'], row=10, col=1)

    fig.show()

if __name__ == "__main__":
    analyze_and_sync_telemetry()
