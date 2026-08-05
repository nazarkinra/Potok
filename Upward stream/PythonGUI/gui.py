import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
from datetime import datetime
import pygame

class ArduinoLoRaGUI:
    def __init__(self, master):
        self.master = master
        master.title("LoRa Ground Station - ПОТОК")
        master.geometry("1050x850") # Увеличили высоту для всех кнопок

        self.serial_port = None
        self.is_connected = False
        self.log_file = None

        self.create_widgets()
        self.init_gamepad()

    def create_widgets(self):
        # 1. Connection Frame
        conn_frame = ttk.LabelFrame(self.master, text="Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").pack(side="left", padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side="left", padx=5)
        self.refresh_ports()

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side="right", padx=10)

        # 2. Dashboard Frame
        dashboard_frame = ttk.LabelFrame(self.master, text="Glider Dashboard")
        dashboard_frame.pack(fill="x", padx=10, pady=5)

        self.node_var = tk.StringVar(value="Node: --")
        self.temp_var = tk.StringVar(value="Temp: -- C")
        self.alt_var = tk.StringVar(value="Alt: -- m")
        self.photo_var = tk.StringVar(value="Photo: --")
        self.flags_var = tk.StringVar(value="State: --")
        self.acc_var = tk.StringVar(value="Acc: --")
        self.gyr_var = tk.StringVar(value="Gyr: --")
        self.mag_var = tk.StringVar(value="Mag: --")

        # Основные параметры
        ttk.Label(dashboard_frame, textvariable=self.node_var, font=("Arial", 12, "bold"), width=15).grid(row=0, column=0, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.temp_var, font=("Arial", 12), width=20).grid(row=0, column=1, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.alt_var, font=("Arial", 12), width=25).grid(row=0, column=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.photo_var, font=("Arial", 12), width=25).grid(row=0, column=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.flags_var, font=("Arial", 12, "bold"), foreground="blue").grid(row=0, column=3, padx=10, pady=5, sticky="w")

        # Инерциальные датчики (IMU)
        ttk.Label(dashboard_frame, textvariable=self.acc_var, font=("Arial", 12)).grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.gyr_var, font=("Arial", 12)).grid(row=1, column=2, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.mag_var, font=("Arial", 12)).grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # 3. Управление сервоприводами
        control_frame = ttk.LabelFrame(self.master, text="Glider Control (Servos)")
        control_frame.pack(fill="x", padx=10, pady=5)

        # Servo 1
        ttk.Label(control_frame, text="Servo 1:").pack(side="left", padx=5)
        self.servo1_var = tk.IntVar(value=90)
        self.servo1_slider = ttk.Scale(control_frame, from_=0, to=180, variable=self.servo1_var, orient="horizontal", command=lambda v: self.servo1_label.config(text=f"{int(float(v))}°"))
        self.servo1_slider.pack(side="left", padx=5)
        self.servo1_label = ttk.Label(control_frame, text="90°", width=4)
        self.servo1_label.pack(side="left", padx=2)
        ttk.Button(control_frame, text="Send", command=self.send_servo1).pack(side="left", padx=5)

        ttk.Separator(control_frame, orient='vertical').pack(side="left", fill='y', padx=10)

        # Servo 2
        ttk.Label(control_frame, text="Servo 2:").pack(side="left", padx=5)
        self.servo2_var = tk.IntVar(value=90)
        self.servo2_slider = ttk.Scale(control_frame, from_=0, to=180, variable=self.servo2_var, orient="horizontal", command=lambda v: self.servo2_label.config(text=f"{int(float(v))}°"))
        self.servo2_slider.pack(side="left", padx=5)
        self.servo2_label = ttk.Label(control_frame, text="90°", width=4)
        self.servo2_label.pack(side="left", padx=2)
        ttk.Button(control_frame, text="Send", command=self.send_servo2).pack(side="left", padx=5)
        
        # 3.5. Системные команды (Re-Init)
        sys_frame = ttk.LabelFrame(self.master, text="System Control (Re-Initialization)")
        sys_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(sys_frame, text="Re-init BMI088", command=lambda: self.send_sys_cmd(32)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init LIS3MDL", command=lambda: self.send_sys_cmd(33)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init BMP388", command=lambda: self.send_sys_cmd(34)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init SD Card", command=lambda: self.send_sys_cmd(35)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init I2C Exp", command=lambda: self.send_sys_cmd(36)).pack(side="left", padx=5, pady=5)

        # 4. Управление режимами полета
        # 4. Управление режимами полета
        flight_frame = ttk.LabelFrame(self.master, text="Flight Control (Manual Override)")
        flight_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(flight_frame, text="0. IDLE (Отключен)", command=lambda: self.send_sys_cmd(64)).pack(side="left", padx=3, pady=5)
        ttk.Button(flight_frame, text="1. ARMED (В ракете)", command=lambda: self.send_sys_cmd(65)).pack(side="left", padx=3, pady=5)
        ttk.Button(flight_frame, text="2. DROP (Падение)", command=lambda: self.send_sys_cmd(66)).pack(side="left", padx=3, pady=5)
        ttk.Button(flight_frame, text="3. RECOVERY", command=lambda: self.send_sys_cmd(67)).pack(side="left", padx=3, pady=5)
        ttk.Button(flight_frame, text="4. GLIDE", command=lambda: self.send_sys_cmd(68)).pack(side="left", padx=3, pady=5)
        ttk.Button(flight_frame, text="5. PARACHUTE", command=lambda: self.send_sys_cmd(69)).pack(side="left", padx=3, pady=5)
        
        # 4.5. Оперативное изменение углов
        angles_frame = ttk.LabelFrame(self.master, text="Live Target Angles (Temporary)")
        angles_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(angles_frame, text="Target Pitch (°):").pack(side="left", padx=5)
        self.live_pitch_var = tk.StringVar(value="-5")
        ttk.Entry(angles_frame, textvariable=self.live_pitch_var, width=5).pack(side="left", padx=5)
        ttk.Button(angles_frame, text="Set Pitch", command=lambda: self.send_live_angle(113, self.live_pitch_var.get())).pack(side="left", padx=5)
        
        ttk.Label(angles_frame, text="Target Roll (°):").pack(side="left", padx=15)
        self.live_roll_var = tk.StringVar(value="0")
        ttk.Entry(angles_frame, textvariable=self.live_roll_var, width=5).pack(side="left", padx=5)
        ttk.Button(angles_frame, text="Set Roll", command=lambda: self.send_live_angle(112, self.live_roll_var.get())).pack(side="left", padx=5)

        # 5. Калибровка датчиков
        calib_frame = ttk.LabelFrame(self.master, text="Sensor Calibration")
        calib_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(calib_frame, text="Zero Altitude", command=lambda: self.send_sys_cmd(50)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Calibrate Gyro", command=lambda: self.send_sys_cmd(48)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Calibrate Accel", command=lambda: self.send_sys_cmd(51)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Calibrate Mag", command=lambda: self.send_sys_cmd(49)).pack(side="left", padx=5, pady=5)
        
        # 6. PID Tuning
        pid_frame = ttk.LabelFrame(self.master, text="PID Tuning (Saved to SD Card)")
        pid_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(pid_frame, text="Parameter:").pack(side="left", padx=5)
        self.pid_param_var = tk.StringVar(value="0x60 (Roll Kp)")
        pid_combo = ttk.Combobox(pid_frame, textvariable=self.pid_param_var, state="readonly", width=15)
        pid_combo['values'] = [
            "0x60 (Roll Kp)", "0x61 (Roll Ki)", "0x62 (Roll Kd)",
            "0x63 (Pitch Kp)", "0x64 (Pitch Ki)", "0x65 (Pitch Kd)",
            "0x66 (Yaw Kp)", "0x67 (Yaw Ki)", "0x68 (Yaw Kd)"
        ]
        pid_combo.pack(side="left", padx=5, pady=5)
        
        ttk.Label(pid_frame, text="Value (float):").pack(side="left", padx=5)
        self.pid_val_var = tk.StringVar(value="1.00")
        ttk.Entry(pid_frame, textvariable=self.pid_val_var, width=10).pack(side="left", padx=5)
        
        ttk.Button(pid_frame, text="Send & Save", command=self.send_pid).pack(side="left", padx=10)

        # 6. Telemetry Log
        telemetry_frame = ttk.LabelFrame(self.master, text="Telemetry (Raw Log)")
        telemetry_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.telemetry_text = scrolledtext.ScrolledText(telemetry_frame, wrap=tk.WORD, height=10)
        self.telemetry_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.telemetry_text.config(state=tk.DISABLED)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_var.get()
            try:
                # Пытаемся открыть порт
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                self.is_connected = True
                
                # Обновляем интерфейс (Зеленый статус)
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text=f"Connected to {port}", foreground="green")
                self.port_combo.config(state="disabled")

                # Создаем файл логов с текущей датой
                filename = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
                self.log_file = open(filename, "a", encoding="utf-8")
                self.log_telemetry(f"=== Session Started. Logging to {filename} ===")

                # Запускаем чтение
                self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.read_thread.start()
            except Exception as e:
                # Если порт занят, выводим ошибку в окно
                self.log_telemetry(f"Error connecting to {port}: {e}")
        else:
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
                
            # Закрываем файл логов
            if self.log_file:
                self.log_file.write(f"=== Session Ended ===\n")
                self.log_file.close()
                self.log_file = None

            # Обновляем интерфейс (Красный статус)
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
            self.port_combo.config(state="normal")

    def read_serial(self):
        while self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.master.after(0, self.log_telemetry, line)
            except Exception as e:
                self.master.after(0, self.log_telemetry, f"Serial read error: {e}")
                self.master.after(0, self.toggle_connection)
                break

    def log_telemetry(self, text):
        # 1. Запись в файл с меткой времени
        if self.log_file:
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.log_file.write(f"[{timestamp}] {text}\n")
            self.log_file.flush() # Принудительное сохранение на диск

        # 2. Вывод логов и ошибок в текстовое поле
        if " | LOG:" in text or text.startswith("===") or text.startswith("Error"):
            self.telemetry_text.config(state=tk.NORMAL)
            self.telemetry_text.insert(tk.END, text + "\n")
            self.telemetry_text.see(tk.END)
            self.telemetry_text.config(state=tk.DISABLED)
            
        # 3. Парсинг данных телеметрии на панель
        elif " | Alt:" in text: 
            try:
                parts = text.split(" | ")
                for p in parts:
                    if p.startswith("Node:"): self.node_var.set(p.strip())
                    elif p.startswith("Acc:"): self.acc_var.set(p.strip())
                    elif p.startswith("Gyr:"): self.gyr_var.set(p.strip())
                    elif p.startswith("Mag:"): self.mag_var.set(p.strip())
                    elif p.startswith("Alt:"): self.alt_var.set(p.strip())
                    elif p.startswith("Temp:"): self.temp_var.set(p.strip())
                    elif p.startswith("Photo:"): self.photo_var.set(p.strip())
                    elif p.startswith("Flags:"):
                        # Добавили пробел после двоеточия: "Flags: 0x"
                        val_str = p.replace("Flags: 0x", "").strip()
                        state_id = (int(val_str, 16) >> 4) & 0x0F
                        
                        states = {
                            0: "IDLE (Отключен/На столе)",
                            1: "Взведен (Ждет выброса)",
                            2: "Выпадание (Падение)",
                            3: "Выход из падения (Тангаж+)",
                            4: "Планирование (Крейсер)",
                            5: "Раскрытие парашюта - Есть"
                        }
                        self.flags_var.set(f"State: {states.get(state_id, 'Неизвестно')}")
            except Exception as e:
                pass 

    # --- Функции отправки команд ---
    def send_servo1(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                angle = int(self.servo1_var.get())
                cmd_str = f"CMD 16 {angle}\n"
                self.serial_port.write(cmd_str.encode('utf-8'))
                self.log_telemetry(f">> PC Request: Sent CMD 16 {angle} (Servo 1 -> {angle}°)")
            except Exception as e:
                self.log_telemetry(f"Error sending command: {e}")

    def send_servo2(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                angle = int(self.servo2_var.get())
                cmd_str = f"CMD 17 {angle}\n"
                self.serial_port.write(cmd_str.encode('utf-8'))
                self.log_telemetry(f">> PC Request: Sent CMD 17 {angle} (Servo 2 -> {angle}°)")
            except Exception as e:
                self.log_telemetry(f"Error sending command: {e}")

    def send_sys_cmd(self, cmd_id):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                cmd_str = f"CMD {cmd_id} 0\n"
                self.serial_port.write(cmd_str.encode('utf-8'))
                self.log_telemetry(f">> PC Request: Sent SYS CMD {cmd_id}")
            except Exception as e:
                self.log_telemetry(f"Error sending command: {e}")
        
    def send_pid(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                cmd_hex = self.pid_param_var.get().split()[0]
                cmd_id = int(cmd_hex, 16)
                
                val_float = float(self.pid_val_var.get())
                param_int = int(val_float * 1000)
                
                if param_int > 32767: param_int = 32767
                if param_int < -32768: param_int = -32768
                
                cmd_str = f"CMD {cmd_id} {param_int}\n"
                self.serial_port.write(cmd_str.encode('utf-8'))
                
                self.log_telemetry(f">> PC Request: Sent PID CMD {cmd_hex} with float value {val_float} (Raw: {param_int})")
            except ValueError:
                self.log_telemetry("Error: Invalid PID value format! Please enter a number.")
            except Exception as e:
                self.log_telemetry(f"Error sending command: {e}")
    
    def send_live_angle(self, cmd_id, val_str):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                angle = int(val_str)
                cmd_str_tx = f"CMD {cmd_id} {angle}\n"
                self.serial_port.write(cmd_str_tx.encode('utf-8'))
                self.log_telemetry(f">> PC Request: Sent Live Angle CMD {hex(cmd_id)} -> {angle}°")
            except ValueError:
                self.log_telemetry("Error: Angle must be an integer!")
            except Exception as e:
                self.log_telemetry(f"Error sending command: {e}")
                
    # --- ФУНКЦИИ ГЕЙМПАДА ---
    def init_gamepad(self):
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.log_telemetry(f"=== Gamepad connected: {self.joystick.get_name()} ===")
                # Запускаем цикл опроса событий
                self.poll_gamepad()
            else:
                self.log_telemetry("=== Gamepad not found. UI control only. ===")
        except Exception as e:
            self.log_telemetry(f"Gamepad error: {e}")

    def poll_gamepad(self):
        try:
            # Обработка всех событий pygame (кнопки, крестовина)
            for event in pygame.event.get():
                
                # Крестовина (D-pad) управляет целевыми углами
                if event.type == pygame.JOYHATMOTION:
                    dx, dy = event.value
                    
                    # Изменение тангажа (Pitch) - Вверх/Вниз
                    if dy != 0:
                        try:
                            current_pitch = int(self.live_pitch_var.get())
                            new_pitch = current_pitch + dy 
                            self.live_pitch_var.set(str(new_pitch))
                            self.send_live_angle(113, str(new_pitch))
                        except ValueError: pass
                        
                    # Изменение крена (Roll) - Влево/Вправо
                    if dx != 0:
                        try:
                            current_roll = int(self.live_roll_var.get())
                            new_roll = current_roll + dx
                            self.live_roll_var.set(str(new_roll))
                            self.send_live_angle(112, str(new_roll))
                        except ValueError: pass
                        
                # Обычные кнопки управляют режимами
                elif event.type == pygame.JOYBUTTONDOWN:
                    # Кнопка 0 (Обычно A / Крестик) - Режим GLIDE (Планирование)
                    if event.button == 0:
                        self.send_sys_cmd(68) 
                        self.log_telemetry(">> Gamepad: Switched to GLIDE")
                        
                    # Кнопка 1 (Обычно B / Кружок) - Режим RECOVERY (Выход из пике)
                    elif event.button == 1:
                        self.send_sys_cmd(67) 
                        self.log_telemetry(">> Gamepad: Switched to RECOVERY")
                        
        except Exception as e:
            pass
            
        # Tkinter запустит эту функцию снова через 100 миллисекунд (10 раз в секунду)
        self.master.after(100, self.poll_gamepad)

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoLoRaGUI(root)
    root.mainloop()
