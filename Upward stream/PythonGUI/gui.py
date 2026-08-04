import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading

class ArduinoLoRaGUI:
    def __init__(self, master):
        self.master = master
        master.title("LoRa Ground Station")
        master.geometry("1000x550") # Чуть увеличили окно под новые строки данных

        self.serial_port = None
        self.is_connected = False

        self.create_widgets()

    def create_widgets(self):
        # Connection Frame[cite: 11]
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

        # Dashboard Frame (Обновлен под сетку и новые параметры)
        dashboard_frame = ttk.LabelFrame(self.master, text="Glider Dashboard")
        dashboard_frame.pack(fill="x", padx=10, pady=5)

        self.node_var = tk.StringVar(value="Node: --")
        self.temp_var = tk.StringVar(value="Temp: -- C")
        self.press_var = tk.StringVar(value="Press: -- Pa")
        self.flags_var = tk.StringVar(value="Flags: --")
        self.acc_var = tk.StringVar(value="Acc: --")
        self.gyr_var = tk.StringVar(value="Gyr: --")
        self.mag_var = tk.StringVar(value="Mag: --")

        # Первая строка: Основные параметры
        ttk.Label(dashboard_frame, textvariable=self.node_var, font=("Arial", 12, "bold"), width=15).grid(row=0, column=0, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.temp_var, font=("Arial", 12), width=20).grid(row=0, column=1, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.press_var, font=("Arial", 12), width=25).grid(row=0, column=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.flags_var, font=("Arial", 12)).grid(row=0, column=3, padx=10, pady=5, sticky="w")

        # Вторая строка: Инерциальные датчики (IMU) и Магнитометр
        ttk.Label(dashboard_frame, textvariable=self.acc_var, font=("Arial", 12)).grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.gyr_var, font=("Arial", 12)).grid(row=1, column=2, columnspan=2, padx=10, pady=5, sticky="w")
        ttk.Label(dashboard_frame, textvariable=self.mag_var, font=("Arial", 12)).grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # Telemetry Text Area[cite: 11]
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

                # Start reading thread[cite: 11]
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
        # Парсинг новой строки от Arduino вида:
        # "Node: 1 | Acc: 1.23,4.56,9.81 | Gyr: 0.12,0.34,0.56 | Mag: 100,-200,300 | Press: 101325.00 Pa | Temp: 24.50 C | Flags: 0x7"
        if "Node:" in text and "|" in text:
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
                    elif p.startswith("Press:"):
                        self.press_var.set(p.strip())
                    elif p.startswith("Temp:"):
                        self.temp_var.set(p.strip())
                    elif p.startswith("Flags:"):
                        self.flags_var.set(p.strip())
            except Exception as e:
                pass # В случае ошибки парсинга строка все равно попадет в лог ниже

        self.telemetry_text.config(state=tk.NORMAL)
        self.telemetry_text.insert(tk.END, text + "\n")
        self.telemetry_text.see(tk.END)
        self.telemetry_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoLoRaGUI(root)
    root.mainloop()
