import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading

class ArduinoLoRaGUI:
    def __init__(self, master):
        self.master = master
        master.title("LoRa Ground Station")
        master.geometry("800x600")

        self.serial_port = None
        self.is_connected = False

        self.create_widgets()

    def create_widgets(self):
        # Connection Frame
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

        # Telemetry Text Area
        telemetry_frame = ttk.LabelFrame(self.master, text="Telemetry (Sensors)")
        telemetry_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.telemetry_text = scrolledtext.ScrolledText(telemetry_frame, wrap=tk.WORD, height=15)
        self.telemetry_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.telemetry_text.config(state=tk.DISABLED)

        # Control Frame
        control_frame = ttk.LabelFrame(self.master, text="Control (Motors)")
        control_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(control_frame, text="Command (dir speed time):").pack(side="left", padx=5)
        self.cmd_entry = ttk.Entry(control_frame, width=30)
        self.cmd_entry.pack(side="left", padx=5)

        ttk.Button(control_frame, text="Send", command=self.send_custom_command).pack(side="left", padx=5)

        # Presets
        presets_frame = ttk.Frame(control_frame)
        presets_frame.pack(side="left", padx=20)
        ttk.Button(presets_frame, text="Forward", command=lambda: self.send_cmd("forward 80 1000")).pack(side="left", padx=2)
        ttk.Button(presets_frame, text="Backward", command=lambda: self.send_cmd("backward 80 1000")).pack(side="left", padx=2)
        ttk.Button(presets_frame, text="Left", command=lambda: self.send_cmd("left 80 500")).pack(side="left", padx=2)
        ttk.Button(presets_frame, text="Right", command=lambda: self.send_cmd("right 80 500")).pack(side="left", padx=2)
        ttk.Button(presets_frame, text="STOP", command=lambda: self.send_cmd("stop 0 0")).pack(side="left", padx=2)

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

                # Start reading thread
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
        self.telemetry_text.config(state=tk.NORMAL)
        self.telemetry_text.insert(tk.END, text + "\n")
        self.telemetry_text.see(tk.END)
        self.telemetry_text.config(state=tk.DISABLED)

    def send_cmd(self, cmd):
        if self.is_connected and self.serial_port:
            try:
                # Add newline because Arduino code (readStringUntil('\n')) expects it
                self.serial_port.write((cmd + '\n').encode('utf-8'))
                self.log_telemetry(f"> Sent: {cmd}")
            except Exception as e:
                self.log_telemetry(f"Send error: {e}")
        else:
            self.log_telemetry(f"> Cannot send '{cmd}': Not connected")

    def send_custom_command(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send_cmd(cmd)
            self.cmd_entry.delete(0, tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoLoRaGUI(root)
    root.mainloop()
