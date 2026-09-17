"""Integration checks for the dashboard and the existing LoRa command protocol.

Run with a display (or xvfb-run): python -m unittest discover -s tests -v
Only port discovery and gamepad initialization are mocked; Tk widgets are real.
"""
import io
import sys
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui import ArduinoLoRaGUI


PACKET = (
    "Node: BBB9 | Acc: -0.10,0.59,0.79 | Gyr: 0.64,-0.62,1.24 | "
    "Mag: -2,-823,-662 | Alt: 132.00 m | Temp: 30.92 C | "
    "Photo: 2594 | Flags: 0x47"
)


def all_widgets(parent):
    for child in parent.winfo_children():
        yield child
        yield from all_widgets(child)


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        with patch("gui.serial.tools.list_ports.comports", return_value=[]), \
                patch.object(ArduinoLoRaGUI, "init_gamepad"):
            self.app = ArduinoLoRaGUI(self.root)
        self.root.update()

    def tearDown(self):
        try:
            exists = self.root.winfo_exists()
        except tk.TclError:
            exists = False
        if not exists:
            return
        self.app.is_connected = False
        if self.app.serial_port:
            self.app.serial_port.close()
        self.app.log_file = None
        self.app.on_close()

    def press(self, text):
        buttons = [w for w in all_widgets(self.root)
                   if w.winfo_class() == "TButton" and w.cget("text") == text]
        self.assertEqual(len(buttons), 1, f"Expected one action: {text}")
        buttons[0].invoke()

    def test_telemetry_updates_values_and_decodes_flight_state(self):
        self.app.log_telemetry(PACKET)
        self.assertEqual(self.app.alt_var.get(), "132.00")
        self.assertEqual(self.app.temp_var.get(), "30.92")
        self.assertEqual(self.app.node_var.get(), "BBB9")
        self.assertEqual(self.app.photo_var.get(), "2594")
        self.assertEqual(self.app.flags_var.get(), "GLIDE")
        self.assertIn("-0.10", self.app.acc_var.get())

    def test_outgoing_commands_and_serial_errors_are_visible_in_log(self):
        for message in (">> Queued: SYS CMD 68", "Serial read error: unplugged"):
            self.app.log_telemetry(message)
            self.assertIn(message, self.app.telemetry_text.get("1.0", "end"))

    def test_flight_buttons_preserve_command_ids(self):
        self.app.is_connected = True
        for label, expected in (("IDLE", 64), ("ARMED", 65), ("DROP", 66),
                                ("RECOVERY", 67), ("GLIDE", 68), ("PARACHUTE", 69)):
            self.press(label)
            self.assertEqual(self.app.cmd_queue[-1][0], f"CMD {expected} 0\n")

    def test_servo_pid_and_angle_wire_format(self):
        self.app.is_connected = True
        self.app.servo1_var.set(45)
        self.app.servo2_var.set(135)
        self.app.send_servo1()
        self.app.send_servo2()
        self.app.pid_param_var.set("0x63 (Pitch Kp)")
        self.app.pid_val_var.set("1.25")
        self.app.send_pid()
        self.app.send_live_angle(113, "-5")
        self.assertEqual([c[0] for c in self.app.cmd_queue], [
            "CMD 16 45\n", "CMD 17 135\n", "CMD 99 1250\n", "CMD 113 -5\n"])

    def test_urgent_stop_replaces_pending_motor_command(self):
        self.app.is_connected = True
        self.app.send_sys_cmd_param(19, 850)
        self.app.send_sys_cmd_param(20, 20)
        self.app.send_sys_cmd_param(19, 0, urgent=True)
        self.assertEqual([c[0] for c in self.app.cmd_queue], [
            "CMD 19 0\n", "CMD 20 20\n"])

    def test_received_packet_flushes_command_to_serial(self):
        self.app.serial_port = serial.serial_for_url("loop://", timeout=0.2)
        self.app.is_connected = True
        self.app.send_sys_cmd(68)
        self.app.log_telemetry(PACKET)
        self.assertEqual(self.app.serial_port.readline(), b"CMD 68 0\n")
        self.assertFalse(self.app.cmd_queue)

    def test_clearing_log_keeps_recording_and_dashboard(self):
        self.app.log_file = io.StringIO()
        self.app.log_telemetry(PACKET)
        self.app.log_telemetry("=== Session marker ===")
        recorded = self.app.log_file.getvalue()
        self.press("Очистить")
        self.assertEqual(self.app.telemetry_text.get("1.0", "end").strip(), "")
        self.assertEqual(self.app.log_file.getvalue(), recorded)
        self.assertEqual(self.app.alt_var.get(), "132.00")

    def test_refresh_keeps_selected_port(self):
        from serial.tools.list_ports_common import ListPortInfo
        self.app.port_var.set("COM9")
        with patch("gui.serial.tools.list_ports.comports", return_value=[
                ListPortInfo("COM3"), ListPortInfo("COM9")]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_var.get(), "COM9")

    def test_custom_port_can_be_typed_and_survives_refresh(self):
        self.app.port_combo.delete(0, "end")
        self.app.port_combo.insert(0, "/dev/serial/by-id/potok")
        with patch("gui.serial.tools.list_ports.comports", return_value=[]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_var.get(), "/dev/serial/by-id/potok")

    def test_closing_window_releases_resources_if_log_write_fails(self):
        class FailedWriter(io.StringIO):
            def write(self, text):
                raise OSError("disk full")
        writer = FailedWriter()
        connection = serial.serial_for_url("loop://", timeout=0.2)
        self.app.log_file = writer
        self.app.serial_port = connection
        self.app.is_connected = True
        self.app.on_close()
        self.assertTrue(writer.closed)
        self.assertFalse(connection.is_open)
        with self.assertRaises(tk.TclError):
            self.root.winfo_exists()

    def test_log_open_failure_closes_new_serial_connection(self):
        connection = serial.serial_for_url("loop://", timeout=0.2)
        self.app.port_var.set("COM9")
        with patch("gui.serial.Serial", return_value=connection), \
                patch("builtins.open", side_effect=OSError("read-only folder")):
            self.app.toggle_connection()
        self.assertFalse(connection.is_open)
        self.assertFalse(self.app.is_connected)
        self.assertIsNone(self.app.log_file)

    def test_stale_reader_callbacks_do_not_touch_new_connection(self):
        old = serial.serial_for_url("loop://", timeout=0.2)
        old.close()
        self.app.serial_port = serial.serial_for_url("loop://", timeout=0.2)
        self.app.is_connected = True
        self.app.receive_line(old, PACKET)
        self.app.serial_failed(old, "old reader closed")
        self.assertEqual(self.app.alt_var.get(), "—")
        self.assertTrue(self.app.is_connected)
        self.assertTrue(self.app.serial_port.is_open)


if __name__ == "__main__":
    unittest.main()
