import pandas as pd #[cite: 1]
import numpy as np #[cite: 1]
import re #[cite: 1]
import os #[cite: 1]
from datetime import datetime #[cite: 1]
from scipy.spatial.transform import Rotation, Slerp #[cite: 1]
from scipy.interpolate import interp1d #[cite: 1]
import matplotlib.pyplot as plt #[cite: 1]
import matplotlib.animation as animation #[cite: 1]
from mpl_toolkits.mplot3d.art3d import Poly3DCollection #[cite: 1]
from stl import mesh # Новый импорт для работы с STL

LORA_FILE = "telemetry_20260808_150652.txt" #[cite: 1]
SD_FILE = "glider_recovered_data.csv" #[cite: 1]
STL_FILE = "glider_model_simplified.stl" # Имя вашего сохраненного файла модели

TELEMETRY_PATTERN = re.compile(
    r"\[(\d{2}:\d{2}:\d{2}\.\d{3})\].*?"
    r"Acc:\s*([-\d\.]+).*?"
    r"Alt:\s*([-\d\.]+)\s*m.*?"
    r"Photo:\s*(\d+)"
) #[cite: 1]

class SimpleMadgwick: #[cite: 1]
    def __init__(self, beta=0.1): #[cite: 1]
        self.beta = beta #[cite: 1]
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) #[cite: 1]

    def update(self, g, a, dt): #[cite: 1]
        q = self.q #[cite: 1]
        if np.linalg.norm(a) == 0: return self.q #[cite: 1]
        a = a / np.linalg.norm(a) #[cite: 1]
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - a[0],
            2*(q[0]*q[1] + q[2]*q[3]) - a[1],
            2*(0.5 - q[1]**2 - q[2]**2) - a[2]
        ]) #[cite: 1]
        J = np.array([
            [-2*q[2],  2*q[3], -2*q[0],  2*q[1]],
            [ 2*q[1],  2*q[0],  2*q[3],  2*q[2]],
            [ 0,      -4*q[1], -4*q[2],  0     ]
        ]) #[cite: 1]
        step = J.T.dot(f) #[cite: 1]
        step = step / np.linalg.norm(step) #[cite: 1]
        qDot = 0.5 * np.array([
            -q[1]*g[0] - q[2]*g[1] - q[3]*g[2],
             q[0]*g[0] + q[2]*g[2] - q[3]*g[1],
             q[0]*g[1] - q[1]*g[2] + q[3]*g[0],
             q[0]*g[2] + q[1]*g[1] - q[2]*g[0]
        ]) - self.beta * step #[cite: 1]
        q = q + qDot * dt #[cite: 1]
        self.q = q / np.linalg.norm(q) #[cite: 1]
        return self.q #[cite: 1]

def parse_lora(filename): #[cite: 1]
    data = [] #[cite: 1]
    if not os.path.exists(filename): return pd.DataFrame() #[cite: 1]
    with open(filename, 'r', encoding='utf-8') as f: #[cite: 1]
        for line in f: #[cite: 1]
            match = TELEMETRY_PATTERN.search(line) #[cite: 1]
            if match: #[cite: 1]
                dt = datetime.strptime(match.group(1), "%H:%M:%S.%f") #[cite: 1]
                data.append({
                    "Time": dt, "Acc_X": float(match.group(2)),
                    "Alt": float(match.group(3)), "Photo": int(match.group(4))
                }) #[cite: 1]
    return pd.DataFrame(data) #[cite: 1]

def animate_full_flight(): #[cite: 1]
    print("1. Подготовка телеметрии...") #[cite: 1]
    lora_df = parse_lora(LORA_FILE) #[cite: 1]
    col_names = ["Node_ID", "Acc_X", "Acc_Y", "Acc_Z", "Gyr_X", "Gyr_Y", "Gyr_Z", "Mag_X", "Mag_Y", "Mag_Z", "Altitude", "Temperature", "Photo", "State_Flags"] #[cite: 1]
    sd_df = pd.read_csv(SD_FILE, header=None, names=col_names, on_bad_lines='skip', engine='python') #[cite: 1]

    for col in col_names[1:]: sd_df[col] = pd.to_numeric(sd_df[col], errors='coerce') #[cite: 1]
    sd_df = sd_df.dropna(subset=['Acc_Z']).reset_index(drop=True) #[cite: 1]
    sd_df['Acc_X'] /= 100.0; sd_df['Acc_Y'] /= 100.0; sd_df['Acc_Z'] /= 100.0 #[cite: 1]
    sd_df['Gyr_X'] /= 100.0; sd_df['Gyr_Y'] /= 100.0; sd_df['Gyr_Z'] /= 100.0 #[cite: 1]
    sd_df['Altitude'] /= 10.0; sd_df['Flight_State'] = sd_df['State_Flags'].fillna(0).astype(int) // 16 #[cite: 1]

    # Синхронизация времени[cite: 1]
    sd_df['Exact_Time'] = pd.NaT #[cite: 1]
    last_sd_idx, last_valid_match_idx = 0, -1 #[cite: 1]
    for _, lora_row in lora_df.iterrows(): #[cite: 1]
        search_window = sd_df.iloc[last_sd_idx : last_sd_idx + 100] #[cite: 1]
        if search_window.empty: break #[cite: 1]
        dist = np.abs(search_window['Altitude'] - lora_row['Alt']) * 10 + np.abs(search_window['Photo'] - lora_row['Photo']) + np.abs(search_window['Acc_X'] - lora_row['Acc_X']) * 10 #[cite: 1]
        best_match_idx = dist.idxmin() #[cite: 1]
        sd_df.loc[best_match_idx, 'Exact_Time'] = lora_row['Time'] #[cite: 1]
        last_sd_idx = best_match_idx + 1 #[cite: 1]
        last_valid_match_idx = best_match_idx #[cite: 1]

    sd_df['Exact_Time_num'] = sd_df['Exact_Time'].astype('int64').replace(-9223372036854775808, np.nan).interpolate(method='linear') #[cite: 1]
    sd_df['Time'] = pd.to_datetime(sd_df['Exact_Time_num']) #[cite: 1]

    if last_valid_match_idx != -1 and last_valid_match_idx < len(sd_df) - 1: #[cite: 1]
        last_time = sd_df.loc[last_valid_match_idx, 'Time'] #[cite: 1]
        for i in range(last_valid_match_idx + 1, len(sd_df)): #[cite: 1]
            sd_df.loc[i, 'Time'] = last_time + pd.Timedelta(milliseconds=100 * (i - last_valid_match_idx)) #[cite: 1]

    first_valid = sd_df['Exact_Time'].first_valid_index() #[cite: 1]
    if first_valid is not None: sd_df = sd_df.loc[first_valid:].reset_index(drop=True) #[cite: 1]
    sd_df['Time_s'] = (sd_df['Time'] - sd_df['Time'].iloc[0]).dt.total_seconds() #[cite: 1]
    df = sd_df[(sd_df['Altitude'] > -100) & (sd_df['Altitude'] < 2500)].copy() #[cite: 1]

    print("2. Расчет ориентации (Фильтр Маджвика)...") #[cite: 1]
    madgwick = SimpleMadgwick(beta=1.0) #[cite: 1]
    acc0 = np.array([df['Acc_X'].iloc[0], df['Acc_Y'].iloc[0], df['Acc_Z'].iloc[0]]) #[cite: 1]
    for _ in range(2000): madgwick.update(np.array([0,0,0]), acc0, 0.01) #[cite: 1]
    madgwick.beta = 0.1 #[cite: 1]

    quats = [] #[cite: 1]
    for i in range(len(df)): #[cite: 1]
        dt = df['Time_s'].diff().iloc[i] if i > 0 else 0.1 #[cite: 1]
        if pd.isna(dt) or dt <= 0: dt = 0.001 #[cite: 1]
        acc = np.array([df['Acc_X'].iloc[i], df['Acc_Y'].iloc[i], df['Acc_Z'].iloc[i]]) #[cite: 1]
        gyr = np.array([df['Gyr_X'].iloc[i], df['Gyr_Y'].iloc[i], df['Gyr_Z'].iloc[i]]) * np.pi / 180.0 #[cite: 1]
        q = madgwick.update(gyr, acc, dt) #[cite: 1]
        quats.append(q) #[cite: 1]

    df['q'] = quats #[cite: 1]

    print("3. Интерполяция кадров (замедляем в 2.5 раза)...") #[cite: 1]
    times = df['Time_s'].values #[cite: 1]
    _, unique_indices = np.unique(times, return_index=True) #[cite: 1]
    times = times[unique_indices] #[cite: 1]
    
    quats_arr = np.array([quats[i] for i in unique_indices]) #[cite: 1]
    quats_scipy = np.column_stack((quats_arr[:, 1], quats_arr[:, 2], quats_arr[:, 3], quats_arr[:, 0])) #[cite: 1]

    rotations = Rotation.from_quat(quats_scipy) #[cite: 1]
    slerp = Slerp(times, rotations) #[cite: 1]

    dt_flight = 0.2 #[cite: 1]
    playback_interval_ms = 200 #[cite: 1]

    interp_times = np.arange(times[0], times[-1], dt_flight) #[cite: 1]
    interp_rotations = slerp(interp_times) #[cite: 1]
    alt_interp = interp1d(times, df['Altitude'].values[unique_indices])(interp_times) #[cite: 1]
    state_interp = interp1d(times, df['Flight_State'].values[unique_indices], kind='previous')(interp_times) #[cite: 1]

    print("4. Загрузка 3D моделей для разных фаз...")

    # ВАЖНО: Укажите имена ваших STL файлов!
    FILE_CLOSED = "glider_closed.stl"
    FILE_OPEN = "glider_open.stl" # Переименуйте ваш текущий файл или укажите старое имя
    FILE_PARACHUTE = "glider_parachute.stl"

    # Загружаем базовую модель (открытые крылья)
    mesh_open = mesh.Mesh.from_file(FILE_OPEN)
    
    # Считаем общий масштаб 1 раз, чтобы планер не "уменьшался" при появлении огромного парашюта
    scale_factor = 1.5 / np.max(np.abs(mesh_open.vectors))

    # Функция подготовки векторов (масштаб + стартовый поворот)
    def prep_vectors(mesh_obj):
        vecs = mesh_obj.vectors * scale_factor
        # Стартовая ориентация (при необходимости измените 180 на нужный угол)
        pre_rot = Rotation.from_euler('xyz', [180, 180, 0], degrees=True)
        flat_verts = vecs.reshape(-1, 3)
        flat_verts = pre_rot.apply(flat_verts)
        return flat_verts.reshape(-1, 3, 3)

    # 1. Модель: ОТКРЫТАЯ
    vectors_open = prep_vectors(mesh_open)
    
    # 2. Модель: ЗАКРЫТАЯ
    try:
        vectors_closed = prep_vectors(mesh.Mesh.from_file(FILE_CLOSED))
    except Exception as e:
        print(f"⚠️ Ошибка загрузки {FILE_CLOSED}, использую открытую модель. Ошибка: {e}")
        vectors_closed = vectors_open
        
    # 3. Модель: ПАРАШЮТ
    try:
        vectors_parachute = prep_vectors(mesh.Mesh.from_file(FILE_PARACHUTE))
    except Exception as e:
        print(f"⚠️ Ошибка загрузки {FILE_PARACHUTE}, использую открытую модель. Ошибка: {e}")
        vectors_parachute = vectors_open


    print("Сборка 3D сцены...")
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_title('Анимация полета: от старта до приземления')
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])

    # Создаем стартовый полигон (темно-серый цвет)
    poly3d = Poly3DCollection(vectors_closed, facecolors='#202020', edgecolors='none', alpha=0.9)
    ax.add_collection3d(poly3d)

    # Тексты телеметрии
    states_map = {0: "ОЖИДАНИЕ", 1: "В РАКЕТЕ (ВЗЛЕТ)", 2: "СБРОС", 3: "РАСКРЫТИЕ", 4: "ШТОПОР (GLIDE)", 5: "ПАРАШЮТ"}
    state_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, color='blue', fontweight='bold')
    alt_text = ax.text2D(0.05, 0.90, "", transform=ax.transAxes, fontsize=12)
    time_text = ax.text2D(0.05, 0.85, "", transform=ax.transAxes, fontsize=12)

    def update(frame_idx):
        if frame_idx % 10 == 0:
            print(f"⏳ Рендеринг кадра {frame_idx} из {len(interp_times)}...")

        r = interp_rotations[frame_idx]
        state_val = int(state_interp[frame_idx])
        
        # --- ПЕРЕКЛЮЧЕНИЕ МОДЕЛЕЙ ---
        # Фазы 0, 1, 2, 3 -> Закрытые крылья
        if state_val <= 3:
            current_vectors = vectors_closed
        # Фаза 4 (Штопор/Glide) -> Раскрытые крылья
        elif state_val == 4:
            current_vectors = vectors_open
        # Фаза 5 (Парашют) и далее -> Модель с парашютом
        else:
            current_vectors = vectors_parachute
            
        # Обновляем поворот выбранной сетки
        flat_verts = current_vectors.reshape(-1, 3)
        rot_verts = r.apply(flat_verts)
        new_vectors = rot_verts.reshape(-1, 3, 3)
        
        # Подменяем геометрию в отрисовщике
        poly3d.set_verts(new_vectors)
        
        # Телеметрия
        state_text.set_text(f"Фаза: {states_map.get(state_val, 'НЕИЗВЕСТНО')}")
        alt_text.set_text(f"Высота: {alt_interp[frame_idx]:.1f} м")
        time_text.set_text(f"Время полета: {interp_times[frame_idx]:.1f} с")
        
        return poly3d, state_text, alt_text, time_text

    print(f"5. Рендеринг GIF (всего кадров: {len(interp_times)}). Это займет какое-то время...")
    ani = animation.FuncAnimation(fig, update, frames=len(interp_times), interval=playback_interval_ms, blit=False)
    
    gif_name = 'full_flight.gif'
    ani.save(gif_name, writer='pillow')
    print(f"✅ [ГОТОВО] Анимация успешно сохранена в файл: {gif_name}")

if __name__ == "__main__":
    animate_full_flight()
