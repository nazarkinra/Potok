import pandas as pd
import numpy as np
import plotly.graph_objects as go
import re
import os
from datetime import datetime
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
                    "Time": dt, "Acc_X": float(match.group(2)),
                    "Alt": float(match.group(3)), "Photo": int(match.group(4))
                })
    return pd.DataFrame(data)

def animate_flight():
    print(f"1. Ищем эталонный лог '{LORA_FILE}'...")
    lora_df = parse_lora(LORA_FILE)
    if lora_df.empty:
        print(f"[ОШИБКА] Не удалось загрузить {LORA_FILE}")
        return

    print(f"2. Ищем файл SD-карты '{SD_FILE}'...")
    if not os.path.exists(SD_FILE):
        print(f"[ОШИБКА] Файл {SD_FILE} не найден!")
        return

    col_names = ["Node_ID", "Acc_X", "Acc_Y", "Acc_Z", "Gyr_X", "Gyr_Y", "Gyr_Z", "Mag_X", "Mag_Y", "Mag_Z", "Altitude", "Temperature", "Photo", "State_Flags"]
    sd_df = pd.read_csv(SD_FILE, header=None, names=col_names, on_bad_lines='skip', engine='python')

    print("   Очистка данных...")
    for col in col_names[1:]:
        sd_df[col] = pd.to_numeric(sd_df[col], errors='coerce')
    sd_df = sd_df.dropna(subset=['Acc_Z']).reset_index(drop=True)
    
    sd_df['Acc_X'] /= 100.0; sd_df['Acc_Y'] /= 100.0; sd_df['Acc_Z'] /= 100.0
    sd_df['Gyr_X'] /= 100.0; sd_df['Gyr_Y'] /= 100.0; sd_df['Gyr_Z'] /= 100.0
    sd_df['Altitude'] /= 10.0; sd_df['Flight_State'] = sd_df['State_Flags'].fillna(0).astype(int) // 16

    print("3. Синхронизация времени...")
    sd_df['Exact_Time'] = pd.NaT
    last_sd_idx = 0
    last_valid_match_idx = -1
    
    for _, lora_row in lora_df.iterrows():
        search_window = sd_df.iloc[last_sd_idx : last_sd_idx + 100]
        if search_window.empty: break
        dist = np.abs(search_window['Altitude'] - lora_row['Alt']) * 10 + np.abs(search_window['Photo'] - lora_row['Photo']) + np.abs(search_window['Acc_X'] - lora_row['Acc_X']) * 10
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
    
    df = sd_df[(sd_df['Altitude'] > -100) & (sd_df['Altitude'] < 2500)].copy()

    print("4. Прогоняем фильтр Маджвика (расчет 3D)...")
    madgwick = SimpleMadgwick(beta=0.1)
    
    acc0 = np.array([df['Acc_X'].iloc[0], df['Acc_Y'].iloc[0], df['Acc_Z'].iloc[0]])
    for _ in range(500): madgwick.update(np.array([0,0,0]), acc0, 0.01)

    curr_x, curr_y = 0.0, 0.0
    scale = 60.0 # Длина красного вектора, показывающего нос планера

    frames = []
    x_vals, y_vals, z_vals = [], [], []

    for i in range(len(df)):
        dt = df['Time_s'].diff().iloc[i] if i > 0 else 0.1
        if pd.isna(dt) or dt <= 0: dt = 0.001
        
        acc = np.array([df['Acc_X'].iloc[i], df['Acc_Y'].iloc[i], df['Acc_Z'].iloc[i]])
        gyr = np.array([df['Gyr_X'].iloc[i], df['Gyr_Y'].iloc[i], df['Gyr_Z'].iloc[i]]) * np.pi / 180.0
        
        q = madgwick.update(gyr, acc, dt)
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        
        nose = r.apply([1, 0, 0]) 

        # Псевдо-кинематика для смещения по осям X, Y
        speed = 15.0 if df['Flight_State'].iloc[i] >= 2 else 0.0
        horiz_len = np.sqrt(nose[0]**2 + nose[1]**2)
        
        if horiz_len > 0.1:
            curr_x += (nose[0] / horiz_len) * speed * dt
            curr_y += (nose[1] / horiz_len) * speed * dt
            
        z_alt = df['Altitude'].iloc[i]
        x_vals.append(curr_x)
        y_vals.append(curr_y)
        z_vals.append(z_alt)
        
        # Сохраняем кадр анимации каждую 5-ю итерацию
        if i % 5 == 0:
            nx = curr_x + nose[0] * scale
            ny = curr_y + nose[1] * scale
            nz = z_alt + nose[2] * scale
            
            frames.append({
                'x': x_vals.copy(),
                'y': y_vals.copy(),
                'z': z_vals.copy(),
                'nx': [curr_x, nx],
                'ny': [curr_y, ny],
                'nz': [z_alt, nz]
            })

    print(f"5. Генерация анимации ({len(frames)} кадров)...")
    
    # Жесткие рамки осей, чтобы камера не "прыгала" во время анимации
    x_min, x_max = min(x_vals) - 100, max(x_vals) + 100
    y_min, y_max = min(y_vals) - 100, max(y_vals) + 100
    z_min, z_max = 0, max(z_vals) + 100

    fig = go.Figure(
        data=[
            # След от траектории
            go.Scatter3d(
                x=frames[0]['x'], y=frames[0]['y'], z=frames[0]['z'],
                mode="lines",
                line=dict(color="purple", width=4),
                name="Траектория"
            ),
            # Вектор носа планера
            go.Scatter3d(
                x=frames[0]['nx'], y=frames[0]['ny'], z=frames[0]['nz'],
                mode="lines",
                line=dict(color="red", width=8),
                name="Ориентация (Нос)"
            )
        ],
        layout=go.Layout(
            title="Интерактивная анимация полета (фильтр Маджвика)",
            scene=dict(
                xaxis=dict(range=[x_min, x_max], autorange=False, title="X (м)"),
                yaxis=dict(range=[y_min, y_max], autorange=False, title="Y (м)"),
                zaxis=dict(range=[z_min, z_max], autorange=False, title="Высота (м)"),
                aspectmode="manual",
                aspectratio=dict(x=1, y=1, z=1) # Единый масштаб для правильной отрисовки вектора
            ),
            height=900,
            margin=dict(l=0, r=0, b=0, t=40),
            updatemenus=[dict(
                type="buttons",
                showactive=False,
                buttons=[dict(
                    label="▶ Play",
                    method="animate",
                    args=[None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True}]
                )]
            )]
        ),
        # Создаем покадровую раскадровку
        frames=[go.Frame(
            data=[
                go.Scatter3d(x=f['x'], y=f['y'], z=f['z']),
                go.Scatter3d(x=f['nx'], y=f['ny'], z=f['nz'])
            ],
            name=str(k)
        ) for k, f in enumerate(frames)]
    )

    html_file = "flight_animation.html"
    fig.write_html(html_file, auto_open=False, include_plotlyjs=True)
    
    print(f"\n✅ [ГОТОВО] Анимация сохранена в файл: {html_file}")
    print("Открой его в браузере и нажми кнопку 'Play'!")

if __name__ == "__main__":
    animate_flight()
