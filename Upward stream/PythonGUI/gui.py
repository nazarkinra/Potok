import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading

class ArduinoLoRaGUI:
    def __init__(self, master):
        self.master = master
        master.title("LoRa Ground Station")
        master.geometry("1000x650") # Чуть увеличили окно, чтобы точно влезли ползунки

        self.serial_port = None
        self.is_connected = False

        self.create_widgets()

    def create_widgets(self):
        # Connection Frame[cite: 14]
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

        # Dashboard Frame[cite: 14]
        dashboard_frame = ttk.LabelFrame(self.master, text="Glider Dashboard")
        dashboard_frame.pack(fill="x", padx=10, pady=5)

        self.node_var = tk.StringVar(value="Node: --")
        self.temp_var = tk.StringVar(value="Temp: -- C")
        self.alt_var = tk.StringVar(value="Alt: -- m")
        self.flags_var = tk.StringVar(value="Flags: --")
        self.acc_var = tk.StringVar(value="Acc: --")
        self.gyr_var = tk.StringVar(value="Gyr: --")
        self.mag_var = tk.StringVar(value="Mag: --")

        # Первая строка: Основные параметры[cite: 14]
        ttk.Label(dashboard_frame, textvariable=self.node_var, font=("Arial", 12, "bold"), width=15).grid(row=0, column=0, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.temp_var, font=("Arial", 12), width=20).grid(row=0, column=1, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.alt_var, font=("Arial", 12), width=25).grid(row=0, column=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.flags_var, font=("Arial", 12)).grid(row=0, column=3, padx=10, pady=5, sticky="w")

        # Вторая строка: Инерциальные датчики (IMU) и Магнитометр[cite: 14]
        ttk.Label(dashboard_frame, textvariable=self.acc_var, font=("Arial", 12)).grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.gyr_var, font=("Arial", 12)).grid(row=1, column=2, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.mag_var, font=("Arial", 12)).grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # --- Управление сервоприводами ---
        control_frame = ttk.LabelFrame(self.master, text="Glider Control (Servos)")
        control_frame.pack(fill="x", padx=10, pady=5)

        # Servo 1 (cmd_id = 16)
        ttk.Label(control_frame, text="Servo 1:").pack(side="left", padx=5)
        self.servo1_var = tk.IntVar(value=90)
        self.servo1_slider = ttk.Scale(control_frame, from_=0, to=180, variable=self.servo1_var, orient="horizontal", command=lambda v: self.servo1_label.config(text=f"{int(float(v))}°"))
        self.servo1_slider.pack(side="left", padx=5)
        self.servo1_label = ttk.Label(control_frame, text="90°", width=4)
        self.servo1_label.pack(side="left", padx=2)
        ttk.Button(control_frame, text="Send", command=self.send_servo1).pack(side="left", padx=5)

        # --- Системные команды (Re-Init) ---
        sys_frame = ttk.LabelFrame(self.master, text="System Control (Re-Initialization)")
        sys_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(sys_frame, text="Re-init BMI088", command=lambda: self.send_sys_cmd(32)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init LIS3MDL", command=lambda: self.send_sys_cmd(33)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init BMP388", command=lambda: self.send_sys_cmd(34)).pack(side="left", padx=5, pady=5)
        ttk.Button(sys_frame, text="Re-init SD Card", command=lambda: self.send_sys_cmd(35)).pack(side="left", padx=5, pady=5)

        # --- Калибровка датчиков планера ПОТОК ---
        calib_frame = ttk.LabelFrame(self.master, text="ПОТОК Sensor Calibration")
        calib_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(calib_frame, text="Calibrate Gyro (BMI088)", 
                   command=lambda: self.send_sys_cmd(48)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Calibrate Accel (BMI088)", 
                   command=lambda: self.send_sys_cmd(51)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Calibrate Mag (LIS3MDL)", 
                   command=lambda: self.send_sys_cmd(49)).pack(side="left", padx=5, pady=5)
        ttk.Button(calib_frame, text="Zero Altitude (BMP388)", 
                   command=lambda: self.send_sys_cmd(50)).pack(side="left", padx=5, pady=5)

        # Разделитель
        ttk.Separator(control_frame, orient='vertical').pack(side="left", fill='y', padx=10)

        # Servo 2 (cmd_id = 17)
        ttk.Label(control_frame, text="Servo 2:").pack(side="left", padx=5)
        self.servo2_var = tk.IntVar(value=90)
        self.servo2_slider = ttk.Scale(control_frame, from_=0, to=180, variable=self.servo2_var, orient="horizontal", command=lambda v: self.servo2_label.config(text=f"{int(float(v))}°"))
        self.servo2_slider.pack(side="left", padx=5)
        self.servo2_label = ttk.Label(control_frame, text="90°", width=4)
        self.servo2_label.pack(side="left", padx=2)
        ttk.Button(control_frame, text="Send", command=self.send_servo2).pack(side="left", padx=5)

        # Telemetry Text Area[cite: 14]
        telemetry_frame = ttk.LabelFrame(self.master, text="Telemetry (Raw Log)")
        telemetry_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.telemetry_text = scrolledtext.ScrolledText(telemetry_frame, wrap=tk.WORD, height=15)
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
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text=f"Connected to {port}", foreground="green")
                self.port_combo.config(state="disabled")

                # Start reading thread[cite: 14]
                self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.read_thread.start()
            except Exception as e:
                self.log_telemetry(f"Error connecting: {e}")
        else:
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
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
        # 1. Если это текстовый лог (содержит маркер " | LOG:")[cite: 14]
        if " | LOG:" in text:
            if text.startswith("Node:"):
                parts = text.split(" | ")
                self.node_var.set(parts[0].strip())

            self.telemetry_text.config(state=tk.NORMAL)
            self.telemetry_text.insert(tk.END, text + "\n")
            self.telemetry_text.see(tk.END)
            self.telemetry_text.config(state=tk.DISABLED)
            
        # 2. Если это регулярная телеметрия (содержит маркер " | Acc:")[cite: 14]
        elif "Node:" in text and " | Acc:" in text:
            try:
                parts = text.split(" | ")
                for p in parts:
                    if p.startswith("Node:"):
                        self.node_var.set(p.strip())
                    elif p.startswith("Acc:"):
                        self.acc_var.set(p.strip())
                    elif p.startswith("Gyr:"):
                        self.gyr_var.set(p.strip())
                    elif p.startswith("Mag:"):
                        self.mag_var.set(p.strip())
                    elif p.startswith("Alt:"):
                        self.alt_var.set(p.strip())
                    elif p.startswith("Temp:"):
                        self.temp_var.set(p.strip())
                    elif p.startswith("Flags:"):
                        self.flags_var.set(p.strip())
            except Exception as e:
                pass 

        # 3. Любые другие системные сообщения[cite: 14]
        else:
            self.telemetry_text.config(state=tk.NORMAL)
            self.telemetry_text.insert(tk.END, text + "\n")
            self.telemetry_text.see(tk.END)
            self.telemetry_text.config(state=tk.DISABLED)

    # --- НОВЫЕ ФУНКЦИИ ОТПРАВКИ КОМАНД ---
    def send_servo1(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            angle = int(self.servo1_var.get())
            cmd_str = f"CMD 16 {angle}\n"
            self.serial_port.write(cmd_str.encode('utf-8'))
            self.log_telemetry(f">> PC Request: Sent CMD 16 {angle} (Servo 1 -> {angle}°)")

    def send_servo2(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            angle = int(self.servo2_var.get())
            cmd_str = f"CMD 17 {angle}\n"
            self.serial_port.write(cmd_str.encode('utf-8'))
            self.log_telemetry(f">> PC Request: Sent CMD 17 {angle} (Servo 2 -> {angle}°)")

    def send_sys_cmd(self, cmd_id):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            # Отправляем команду, параметр (0) нам здесь не важен, но он нужен для парсера
            cmd_str = f"CMD {cmd_id} 0\n"
            self.serial_port.write(cmd_str.encode('utf-8'))
            self.log_telemetry(f">> PC Request: Sent SYS CMD {cmd_id} (Re-init Module)")

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoLoRaGUI(root)
    root.mainloop()