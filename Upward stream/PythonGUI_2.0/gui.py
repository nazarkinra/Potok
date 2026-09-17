import math
import tkinter as tk
import serial
import serial.tools.list_ports
import threading
from datetime import datetime
from time import monotonic
import pygame

from dashboard import GroundStationView
from theme import COLORS


FLIGHT_STATES = {
    0: ("IDLE", "Ожидание"), 1: ("ARMED", "В ракете"),
    2: ("DROP", "Свободное падение"), 3: ("RECOVERY", "Восстановление"),
    4: ("GLIDE", "Планирование"), 5: ("PARACHUTE", "Спуск на парашюте"),
}


class ArduinoLoRaGUI(GroundStationView):
    def __init__(self, master):
        self.master = master
        master.title("ПОТОК · Наземная станция LoRa")
        width = min(1360, master.winfo_screenwidth() - 60)
        height = min(940, master.winfo_screenheight() - 80)
        master.geometry(f"{width}x{height}")
        master.minsize(min(1024, width), min(680, height))
        self.serial_port = None
        self.is_connected = False
        self.log_file = None
        self.cmd_queue = []
        self.packet_count = 0
        self.last_packet_time = None
        self._closing = False
        self.gamepad_after_id = None
        self.status_after_id = None
        self.create_widgets()
        self.init_gamepad()
        master.protocol("WM_DELETE_WINDOW", self.on_close)
        self.update_freshness()

    def refresh_ports(self):
        selected = self.port_var.get()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if selected:
            self.port_var.set(selected)
        else:
            self.port_var.set(ports[0] if ports else "")

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self.log_telemetry("Error: Выберите доступный последовательный порт.")
            return
        connection = None
        try:
            connection = serial.Serial(port, 115200, timeout=1)
            filename = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            log_file = open(filename, "a", encoding="utf-8")
        except (serial.SerialException, OSError) as error:
            if connection:
                connection.close()
            self.log_telemetry(f"Error connecting to {port}: {error}")
            return
        self.serial_port = connection
        self.log_file = log_file
        self.is_connected = True
        self.cmd_queue.clear()
        self.packet_count = 0
        self.last_packet_time = None
        for variable in (self.node_var, self.alt_var, self.temp_var, self.photo_var):
            variable.set("—")
        for variable in (self.acc_var, self.gyr_var, self.mag_var):
            variable.set("—  /  —  /  —")
        self.flags_var.set("НЕТ ДАННЫХ")
        self.state_hint.configure(text="Ожидание данных с борта")
        self.packet_var.set("Пакетов: 0")
        self.queue_var.set("Очередь: 0")
        self.altitude_chart.clear()
        self.connect_btn.configure(text="Отключить", style="TButton")
        self.status_label.configure(text=f"●  {port}", foreground=COLORS["accent"])
        self.port_combo.configure(state="disabled")
        self.refresh_btn.configure(state="disabled")
        self.recording_var.set("● Запись в файл")
        self.log_telemetry(f"=== Session Started. Logging to {filename} ===")
        self.read_thread = threading.Thread(target=self.read_serial, args=(connection,), daemon=True)
        self.read_thread.start()
        self.update_freshness_label()

    def disconnect(self):
        self.is_connected = False
        log_error = None
        if self.serial_port:
            try:
                self.serial_port.close()
            except OSError:
                pass
            self.serial_port = None
        if self.log_file:
            log_file = self.log_file
            self.log_file = None
            try:
                log_file.write("=== Session Ended ===\n")
            except OSError as error:
                log_error = error
            finally:
                try:
                    log_file.close()
                except OSError as error:
                    log_error = error
        self.cmd_queue.clear()
        self.queue_var.set("Очередь: 0")
        self.connect_btn.configure(text="Подключить", style="Accent.TButton")
        self.status_label.configure(text="●  Не подключено", foreground=COLORS["muted"])
        self.port_combo.configure(state="normal")
        self.refresh_btn.configure(state="normal")
        self.recording_var.set("Запись остановлена")
        self.update_freshness_label()
        if log_error and not self._closing:
            self.log_telemetry(f"Error closing telemetry log: {log_error}")

    def read_serial(self, connection):
        # Capture the connection: an old reader must not consume a new session.
        while not self._closing and self.is_connected and self.serial_port is connection and connection.is_open:
            try:
                line = connection.readline().decode("utf-8", errors="ignore").strip()
                if line and not self._closing:
                    self.master.after(0, self.receive_line, connection, line)
            except Exception as error:
                if not self._closing:
                    try:
                        self.master.after(0, self.serial_failed, connection, str(error))
                    except (RuntimeError, tk.TclError):
                        pass
                break

    def receive_line(self, connection, line):
        if not self._closing and self.is_connected and self.serial_port is connection:
            self.log_telemetry(line)

    def serial_failed(self, connection, message):
        if not self._closing and self.serial_port is connection:
            self.log_telemetry(f"Serial read error: {message}")
            self.disconnect()

    def log_telemetry(self, text):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if self.log_file:
            self.log_file.write(f"[{timestamp}] {text}\n")
            self.log_file.flush()
        if " | Alt:" in text and " | LOG:" not in text:
            self.update_telemetry(text)
        else:
            tag = ("error" if "error" in text.lower() else "command" if text.startswith(">>")
                   else "session" if text.startswith("===") else "")
            follow_tail = self.telemetry_text.yview()[1] >= 0.98
            self.telemetry_text.configure(state="normal")
            self.telemetry_text.insert("end", f"{timestamp}  ", "time")
            self.telemetry_text.insert("end", text + "\n", tag)
            # Keep the display bounded; the full session is still recorded to disk.
            line_count = int(self.telemetry_text.index("end-1c").split(".")[0])
            if line_count > 1500:
                self.telemetry_text.delete("1.0", f"{line_count - 1500}.0")
            if follow_tail:
                self.telemetry_text.see("end")
            self.telemetry_text.configure(state="disabled")
        # Retain the existing queue/drain behavior and command wire format.
        if self.is_connected and self.cmd_queue and self.serial_port and self.serial_port.is_open:
            try:
                cmd_str, log_msg = self.cmd_queue.pop(0)
                self.serial_port.write(cmd_str.encode("utf-8"))
                self.log_telemetry(log_msg)
            except (serial.SerialException, OSError) as error:
                self.log_telemetry(f"Error sending command: {error}")
        self.queue_var.set(f"Очередь: {len(self.cmd_queue)}")

    def update_telemetry(self, text):
        fields = {}
        for part in text.split(" | "):
            name, separator, value = part.partition(":")
            if separator:
                fields[name.strip()] = value.strip()
        try:
            altitude = float(fields["Alt"].split()[0])
            temperature = float(fields["Temp"].split()[0])
            if not (math.isfinite(altitude) and math.isfinite(temperature)):
                return
        except (KeyError, ValueError, IndexError):
            return
        self.alt_var.set(fields["Alt"].split()[0])
        self.temp_var.set(fields["Temp"].split()[0])
        self.node_var.set(fields.get("Node", "—"))
        self.photo_var.set(fields.get("Photo", "—"))
        for key, variable in (("Acc", self.acc_var), ("Gyr", self.gyr_var), ("Mag", self.mag_var)):
            variable.set(fields.get(key, "—, —, —").replace(",", " / "))
        try:
            state_id = (int(fields["Flags"], 16) >> 4) & 0x0F
            name, description = FLIGHT_STATES.get(state_id, ("НЕИЗВЕСТНО", "Неизвестный режим"))
        except (KeyError, ValueError):
            name, description = "НЕТ ДАННЫХ", "Режим не получен"
        self.flags_var.set(name)
        self.state_hint.configure(text=description)
        self.altitude_chart.add_sample(altitude)
        self.packet_count += 1
        self.packet_var.set(f"Пакетов: {self.packet_count}")
        self.last_packet_time = monotonic()
        self.update_freshness_label()

    def update_freshness_label(self):
        if self.last_packet_time is None:
            text, color = "Ожидание телеметрии", "muted"
        elif not self.is_connected:
            text, color = "Связь отключена · последние данные", "amber"
        else:
            age = int(monotonic() - self.last_packet_time)
            text, color = ("● Телеметрия поступает", "accent") if age < 3 else (f"Нет данных {age} с", "amber")
        self.last_packet_var.set(text)
        self.freshness_label.configure(foreground=COLORS[color])

    def update_freshness(self):
        if not self._closing:
            self.update_freshness_label()
            self.status_after_id = self.master.after(1000, self.update_freshness)

    def on_close(self):
        self._closing = True
        for timer in (self.gamepad_after_id, self.status_after_id):
            if timer is not None:
                self.master.after_cancel(timer)
        try:
            self.disconnect()
        finally:
            pygame.quit()
            self.master.destroy()

    def queue_command(self, cmd_str, log_msg, urgent=False):
        if self.is_connected:
            if not hasattr(self, 'cmd_queue'): self.cmd_queue = []
            
            prefix = " ".join(cmd_str.split()[0:2]) 
            
            if urgent:
                self.cmd_queue = [c for c in self.cmd_queue if not c[0].startswith(prefix)]
                self.cmd_queue.insert(0, (cmd_str, log_msg))
            else:
                if prefix in ["CMD 16", "CMD 17", "CMD 19", "CMD 20", "CMD 21", "CMD 22"]:
                    self.cmd_queue = [c for c in self.cmd_queue if not c[0].startswith(prefix)]
                self.cmd_queue.append((cmd_str, log_msg))
            self.queue_var.set(f"Очередь: {len(self.cmd_queue)}")
        else:
            self.log_telemetry("Error: Для отправки команды подключитесь к борту.")
            
    def send_servo1(self):
        angle = int(self.servo1_var.get())
        self.queue_command(f"CMD 16 {angle}\n", f">> Queued: Servo 1 -> {angle}°")

    def send_servo2(self):
        angle = int(self.servo2_var.get())
        self.queue_command(f"CMD 17 {angle}\n", f">> Queued: Servo 2 -> {angle}°")

    def send_sys_cmd(self, cmd_id):
        self.queue_command(f"CMD {cmd_id} 0\n", f">> Queued: SYS CMD {cmd_id}")
                
    def send_sys_cmd_param(self, cmd_id, param, urgent=False):
        self.queue_command(f"CMD {cmd_id} {param}\n", f">> Queued: CMD {cmd_id} | Param: {param}", urgent=urgent)
        
    def send_pid(self):
        try:
            cmd_hex = self.pid_param_var.get().split()[0]
            cmd_id = int(cmd_hex, 16)
            val_float = float(self.pid_val_var.get())
            param_int = int(val_float * 1000)
            if param_int > 32767: param_int = 32767
            if param_int < -32768: param_int = -32768
            self.queue_command(f"CMD {cmd_id} {param_int}\n", f">> Queued: PID {cmd_hex} -> {val_float}")
        except (ValueError, OverflowError):
            self.log_telemetry("Error: Invalid PID value format!")
    
    def send_live_angle(self, cmd_id, val_str):
        try:
            angle = int(val_str)
            self.queue_command(f"CMD {cmd_id} {angle}\n", f">> Queued: Live Angle {hex(cmd_id)} -> {angle}°")
        except ValueError:
            self.log_telemetry("Error: Angle must be an integer!")

    def send_yaw_test(self, direction):
        try:
            time_ms = int(self.yaw_time_var.get())
            param = time_ms * direction
            self.send_sys_cmd_param(21, param)
        except ValueError:
            self.log_telemetry("Error: Yaw time must be an integer!")
                
    def init_gamepad(self):
        try:
            # Event and joystick subsystems only; no audio device is needed.
            pygame.display.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.gamepad_var.set(self.joystick.get_name())
                self.log_telemetry(f"=== Gamepad connected: {self.joystick.get_name()} ===")
                self.poll_gamepad()
            else:
                self.gamepad_var.set("Не подключён · управление с ПК")
                self.log_telemetry("=== Gamepad not found. UI control only. ===")
        except Exception as e:
            self.gamepad_var.set("Недоступен · управление с ПК")
            self.log_telemetry(f"Gamepad error: {e}")

    def poll_gamepad(self):
        try:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:   # A
                        self.send_sys_cmd(68) 
                        self.log_telemetry(">> Gamepad: Switched to GLIDE")
                    elif event.button == 1: # B 
                        self.send_sys_cmd(67) 
                        self.log_telemetry(">> Gamepad: Switched to RECOVERY")
                    elif event.button == 3: # Y 
                        self.send_sys_cmd_param(19, 750) 
                        self.log_telemetry(">> Gamepad: Wings Deploy")
                    elif event.button == 2: # X 
                        self.send_sys_cmd_param(19, 0, urgent=True)   
                        self.log_telemetry(">> Gamepad: Wings Stop")

            if pygame.joystick.get_count() > 0:
                axis_x = self.joystick.get_axis(0) 
                axis_y = self.joystick.get_axis(1) 
                
                # Увеличена мертвая зона для отсечения аппаратного дребезга стиков
                deadzone = 0.25 

                yaw_speed = 0
                if abs(axis_x) > deadzone:
                    sign_x = 1 if axis_x > 0 else -1
                    mapped_x = (abs(axis_x) - deadzone) / (1.0 - deadzone)
                    
                    # Ограничиваем скорость мотора до 60%, чтобы предотвратить перезагрузку STM32
                    yaw_speed = int(mapped_x * 60 * sign_x)
                    # Округляем до десятков
                    yaw_speed = round(yaw_speed / 10) * 10

                servo_angle = 90
                if abs(axis_y) > deadzone:
                    sign_y = 1 if axis_y > 0 else -1
                    mapped_y = (abs(axis_y) - deadzone) / (1.0 - deadzone)
                    servo_angle = 90 + int(mapped_y * 85 * sign_y)
                    # Округляем угол кратно 5 градусам
                    servo_angle = round(servo_angle / 5) * 5

                # Отправляем команды только если значение реально изменилось
                if yaw_speed != getattr(self, 'last_yaw_speed', 0):
                    # ВОССТАНОВЛЕНА ПЕРЕМЕННАЯ:
                    is_stop = (yaw_speed == 0)
                    self.send_sys_cmd_param(20, yaw_speed, urgent=is_stop)
                    self.last_yaw_speed = yaw_speed

                if servo_angle != getattr(self, 'last_servo_angle', 90):
                    self.send_sys_cmd_param(22, servo_angle)
                    self.last_servo_angle = servo_angle

        except Exception as e:
            self.log_telemetry(f"Error in gamepad: {e}")
            
        # ОСТАВЛЕН ТОЛЬКО ОДИН ВЫЗОВ ТАЙМЕРА!
        if not self._closing:
            self.gamepad_after_id = self.master.after(150, self.poll_gamepad)

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoLoRaGUI(root)
    root.mainloop()
