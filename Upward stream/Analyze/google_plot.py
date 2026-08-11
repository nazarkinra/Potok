import pandas as pd
import numpy as np
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

# Перевод координат из D°M'S" в десятичные градусы
def dms2dd(degrees, minutes, seconds, direction):
    dd = float(degrees) + float(minutes)/60 + float(seconds)/(60*60)
    if direction in ['S', 'W', 'Ю', 'З']:
        dd *= -1
    return dd

def export_to_google_earth():
    print("1. Подготовка и синхронизация телеметрии...")
    lora_df = parse_lora(LORA_FILE)
    col_names = ["Node_ID", "Acc_X", "Acc_Y", "Acc_Z", "Gyr_X", "Gyr_Y", "Gyr_Z", "Mag_X", "Mag_Y", "Mag_Z", "Altitude", "Temperature", "Photo", "State_Flags"]
    sd_df = pd.read_csv(SD_FILE, header=None, names=col_names, on_bad_lines='skip', engine='python')

    for col in col_names[1:]: sd_df[col] = pd.to_numeric(sd_df[col], errors='coerce')
    sd_df = sd_df.dropna(subset=['Acc_Z']).reset_index(drop=True)
    sd_df['Acc_X'] /= 100.0; sd_df['Acc_Y'] /= 100.0; sd_df['Acc_Z'] /= 100.0
    sd_df['Gyr_X'] /= 100.0; sd_df['Gyr_Y'] /= 100.0; sd_df['Gyr_Z'] /= 100.0
    sd_df['Altitude'] /= 10.0; sd_df['Flight_State'] = sd_df['State_Flags'].fillna(0).astype(int) // 16

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

    print("2. Расчет кинематики (Фильтр Маджвика)...")
    madgwick = SimpleMadgwick(beta=0.1)
    acc0 = np.array([df['Acc_X'].iloc[0], df['Acc_Y'].iloc[0], df['Acc_Z'].iloc[0]])
    for _ in range(500): madgwick.update(np.array([0,0,0]), acc0, 0.01)

    pos_x, pos_y = [], []
    curr_x, curr_y = 0.0, 0.0

    for i in range(len(df)):
        dt = df['Time_s'].diff().iloc[i] if i > 0 else 0.1
        if pd.isna(dt) or dt <= 0: dt = 0.001
        
        acc = np.array([df['Acc_X'].iloc[i], df['Acc_Y'].iloc[i], df['Acc_Z'].iloc[i]])
        gyr = np.array([df['Gyr_X'].iloc[i], df['Gyr_Y'].iloc[i], df['Gyr_Z'].iloc[i]]) * np.pi / 180.0
        
        q = madgwick.update(gyr, acc, dt)
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        nose = r.apply([1, 0, 0]) 

        speed = 15.0 if df['Flight_State'].iloc[i] >= 2 else 0.0
        horiz_len = np.sqrt(nose[0]**2 + nose[1]**2)
        
        if horiz_len > 0.1:
            curr_x += (nose[0] / horiz_len) * speed * dt
            curr_y += (nose[1] / horiz_len) * speed * dt
            
        pos_x.append(curr_x)
        pos_y.append(curr_y)

    df['Pos_X'] = pos_x
    df['Pos_Y'] = pos_y

    print("3. Геопространственная привязка к координатам...")
    
    lat_start = dms2dd(56, 24, 41.37, 'N')
    lon_start = dms2dd(40, 58, 33.01, 'E')
    
    lat_end = dms2dd(56, 24, 46.63, 'N')
    lon_end = dms2dd(40, 58, 59.30, 'E')

    # Конвертация одного градуса в метры на данной широте
    lat_to_m = 111132.92 - 559.82 * np.cos(2 * np.radians(lat_start))
    lon_to_m = 111319.9 * np.cos(np.radians(lat_start))

    delta_lat_m = (lat_end - lat_start) * lat_to_m
    delta_lon_m = (lon_end - lon_start) * lon_to_m

    loc_x_end = df['Pos_X'].iloc[-1]
    loc_y_end = df['Pos_Y'].iloc[-1]

    # Вычисление вектора масштаба и угла поворота траектории
    loc_vec = np.array([loc_x_end, loc_y_end])
    real_vec = np.array([delta_lon_m, delta_lat_m]) 

    scale = np.linalg.norm(real_vec) / np.linalg.norm(loc_vec)
    
    angle_loc = np.arctan2(loc_y_end, loc_x_end)
    angle_real = np.arctan2(delta_lat_m, delta_lon_m)
    rotation_angle = angle_real - angle_loc

    cos_a = np.cos(rotation_angle)
    sin_a = np.sin(rotation_angle)

    geo_lat = []
    geo_lon = []

    # Трансформируем каждую точку в GPS-координаты
    for _, row in df.iterrows():
        x_rot = (row['Pos_X'] * cos_a - row['Pos_Y'] * sin_a) * scale
        y_rot = (row['Pos_X'] * sin_a + row['Pos_Y'] * cos_a) * scale
        
        point_lon = lon_start + (x_rot / lon_to_m)
        point_lat = lat_start + (y_rot / lat_to_m)
        
        geo_lon.append(point_lon)
        geo_lat.append(point_lat)

    df['Geo_Lon'] = geo_lon
    df['Geo_Lat'] = geo_lat

    print("4. Формирование файла KML...")
    
    # Шаблон KML файла для Google Earth
    kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Траектория ПОТОК</name>
    <description>Привязанная к местности траектория полета планера.</description>
    
    <Style id="trackStyle">
      <LineStyle>
        <color>ff0000ff</color> <!-- Красный цвет линии (формат AABBGGRR) -->
        <width>5</width>
      </LineStyle>
      <PolyStyle>
        <color>3f0000ff</color> <!-- Полупрозрачная "шторка" до земли -->
      </PolyStyle>
    </Style>

    <Placemark>
      <name>Старт</name>
      <Point>
        <coordinates>{lon_start},{lat_start},0</coordinates>
      </Point>
    </Placemark>

    <Placemark>
      <name>Посадка</name>
      <Point>
        <coordinates>{lon_end},{lat_end},0</coordinates>
      </Point>
    </Placemark>

    <Placemark>
      <name>Линия полета</name>
      <styleUrl>#trackStyle</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
"""
    # Добавляем все вычисленные точки с высотой
    for _, row in df.iterrows():
        alt = row['Altitude'] if row['Altitude'] > 0 else 0
        kml_content += f"          {row['Geo_Lon']:.6f},{row['Geo_Lat']:.6f},{alt:.1f}\n"

    kml_content += """        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>"""

    file_name = "potok_flight_trajectory.kml"
    with open(file_name, "w", encoding="utf-8") as f:
        f.write(kml_content)

    print(f"✅ Готово! Файл '{file_name}' сохранен. Открой его в Google Earth.")

if __name__ == "__main__":
    export_to_google_earth()
