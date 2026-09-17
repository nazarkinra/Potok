"""Tkinter view for the ground station; command handlers live in gui.py."""
import tkinter as tk
from tkinter import ttk

from theme import AltitudeChart, COLORS, ScrollablePage, configure_theme


class GroundStationView:
    def create_widgets(self):
        self.font_family, self.mono_family = configure_theme(self.master)
        self.master.columnconfigure(1, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.node_var = tk.StringVar(value="—")
        self.temp_var = tk.StringVar(value="—")
        self.alt_var = tk.StringVar(value="—")
        self.photo_var = tk.StringVar(value="—")
        self.flags_var = tk.StringVar(value="НЕТ ДАННЫХ")
        self.acc_var = tk.StringVar(value="—  /  —  /  —")
        self.gyr_var = tk.StringVar(value="—  /  —  /  —")
        self.mag_var = tk.StringVar(value="—  /  —  /  —")
        self.packet_var = tk.StringVar(value="Пакетов: 0")
        self.last_packet_var = tk.StringVar(value="Ожидание телеметрии")
        self.queue_var = tk.StringVar(value="Очередь: 0")
        self.recording_var = tk.StringVar(value="Запись не начата")
        self.gamepad_var = tk.StringVar(value="Проверка геймпада…")
        self._build_sidebar()

        main = ttk.Frame(self.master, padding=(24, 20, 24, 10))
        main.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)
        header = ttk.Frame(main)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="Полётная консоль", style="Hero.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="Телеметрия и управление планером", style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(4, 0))
        self.freshness_label = ttk.Label(header, textvariable=self.last_packet_var, style="Muted.TLabel")
        self.freshness_label.grid(row=0, column=1, rowspan=2, sticky="e")

        metrics = ttk.Frame(main)
        metrics.grid(row=1, column=0, sticky="ew", pady=(0, 20))
        for index, (title, variable, unit, color) in enumerate((
                ("ВЫСОТА", self.alt_var, "м  /  BMP388", "accent"),
                ("ТЕМПЕРАТУРА", self.temp_var, "°C  /  корпус", "amber"),
                ("ОСВЕЩЁННОСТЬ", self.photo_var, "отсчёты АЦП", "blue"),
                ("БОРТ", self.node_var, "идентификатор узла", "text"))):
            metrics.columnconfigure(index, weight=1, uniform="metrics")
            card = tk.Frame(metrics, bg=COLORS["panel"], highlightthickness=1,
                            highlightbackground=COLORS["line"])
            card.grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 10, 0))
            tk.Frame(card, bg=COLORS[color], height=2).pack(fill="x")
            inner = ttk.Frame(card, style="Panel.TFrame", padding=(16, 12))
            inner.pack(fill="both", expand=True)
            ttk.Label(inner, text=title, style="Panel.Muted.TLabel").pack(anchor="w")
            ttk.Label(inner, textvariable=variable, style="Metric.TLabel", foreground=COLORS[color]).pack(anchor="w", pady=(4, 2))
            ttk.Label(inner, text=unit, style="Panel.Muted.TLabel").pack(anchor="w")

        self.notebook = ttk.Notebook(main)
        self.notebook.grid(row=2, column=0, sticky="nsew")
        self.pages = {}
        for key, title in (("overview", "Обзор"), ("control", "Управление"), ("settings", "Датчики и PID")):
            page = ScrollablePage(self.notebook)
            self.notebook.add(page, text=title)
            self.pages[key] = page
        self._build_overview(self.pages["overview"].body)
        self._build_controls(self.pages["control"].body)
        self._build_settings(self.pages["settings"].body)
        for page in self.pages.values():
            page.bind_scrolling()
        self.notebook.enable_traversal()
        self._build_log(main)

        footer = ttk.Frame(main)
        footer.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(footer, text="ПОТОК  /  НАЗЕМНАЯ СТАНЦИЯ", style="Muted.TLabel").pack(side="left")
        ttk.Label(footer, textvariable=self.queue_var, style="Muted.TLabel").pack(side="right")
        ttk.Label(footer, textvariable=self.packet_var, style="Muted.TLabel").pack(side="right", padx=20)

    def _build_sidebar(self):
        side = ttk.Frame(self.master, style="Side.TFrame", padding=(20, 20), width=240)
        side.grid(row=0, column=0, sticky="ns")
        side.grid_propagate(False)
        side.columnconfigure(0, weight=1)
        side.rowconfigure(5, weight=1)
        brand = ttk.Frame(side, style="Side.TFrame")
        brand.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        icon = tk.Canvas(brand, width=40, height=42, bg=COLORS["sidebar"], highlightthickness=0)
        icon.pack(side="left", padx=(0, 10))
        icon.create_polygon(3, 28, 20, 7, 37, 28, 20, 21, fill="", outline=COLORS["accent"], width=2)
        icon.create_line(20, 21, 20, 38, fill=COLORS["accent"], width=2)
        ttk.Label(brand, text="ПОТОК", style="Brand.TLabel").pack(anchor="w")
        ttk.Label(brand, text="GROUND STATION", style="Side.Muted.TLabel").pack(anchor="w")

        link = ttk.Frame(side, style="Side.TFrame")
        link.grid(row=1, column=0, sticky="ew")
        ttk.Label(link, text="КАНАЛ СВЯЗИ", style="Side.Muted.TLabel").pack(anchor="w")
        self.status_label = ttk.Label(link, text="●  Не подключено", style="Side.TLabel", foreground=COLORS["muted"])
        self.status_label.pack(anchor="w", pady=(10, 18))
        ttk.Label(link, text="Последовательный порт", style="Side.Muted.TLabel").pack(anchor="w", pady=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(link, textvariable=self.port_var, width=17)
        self.port_combo.pack(fill="x")
        self.refresh_ports()
        self.refresh_btn = ttk.Button(link, text="Обновить порты", command=self.refresh_ports, style="Quiet.TButton")
        self.refresh_btn.pack(fill="x", pady=(8, 12))
        self.connect_btn = ttk.Button(link, text="Подключить", command=self.toggle_connection, style="Accent.TButton")
        self.connect_btn.pack(fill="x")
        ttk.Label(link, text="LoRa  ·  115 200 бод", style="Side.Muted.TLabel").pack(anchor="w", pady=(12, 22))
        ttk.Separator(side).grid(row=2, column=0, sticky="ew", pady=(0, 20))

        state = ttk.Frame(side, style="Side.TFrame")
        state.grid(row=3, column=0, sticky="ew")
        ttk.Label(state, text="РЕЖИМ ПОЛЁТА", style="Side.Muted.TLabel").pack(anchor="w")
        ttk.Label(state, textvariable=self.flags_var, style="State.TLabel").pack(anchor="w", pady=(8, 8))
        self.state_hint = ttk.Label(state, text="Ожидание данных с борта", style="Side.Muted.TLabel", wraplength=190)
        self.state_hint.pack(anchor="w")
        ttk.Separator(state).pack(fill="x", pady=20)
        ttk.Label(state, text="ЗАПИСЬ СЕССИИ", style="Side.Muted.TLabel").pack(anchor="w")
        ttk.Label(state, textvariable=self.recording_var, style="Side.TLabel", wraplength=190).pack(anchor="w", pady=(8, 0))

        gamepad = ttk.Frame(side, style="Side.TFrame")
        gamepad.grid(row=6, column=0, sticky="sew", pady=(20, 0))
        ttk.Label(gamepad, text="ГЕЙМПАД", style="Side.Muted.TLabel").pack(anchor="w")
        ttk.Label(gamepad, textvariable=self.gamepad_var, style="Side.TLabel", wraplength=190).pack(anchor="w", pady=(8, 12))
        shortcuts = ttk.Frame(gamepad, style="Side.TFrame")
        for key, description in (("A / B", "GLIDE / RECOVERY"), ("Y / X", "Крылья / Стоп")):
            ttk.Label(shortcuts, text=f"{key}    {description}", style="Side.Muted.TLabel").pack(anchor="w", pady=2)
        def fit_sidebar(event):
            if event.height >= 780:
                shortcuts.pack(anchor="w")
            else:
                shortcuts.pack_forget()
        side.bind("<Configure>", fit_sidebar)

    def _panel(self, parent, title, subtitle=None):
        outer = tk.Frame(parent, bg=COLORS["panel"], highlightthickness=1,
                         highlightbackground=COLORS["line"])
        inner = ttk.Frame(outer, style="Panel.TFrame", padding=12)
        inner.pack(fill="both", expand=True)
        ttk.Label(inner, text=title, style="Panel.Title.TLabel").pack(anchor="w")
        if subtitle:
            ttk.Label(inner, text=subtitle, style="Panel.Muted.TLabel").pack(anchor="w", pady=(4, 0))
        body = ttk.Frame(inner, style="Panel.TFrame")
        body.pack(fill="both", expand=True, pady=(8, 0))
        return outer, body

    def _build_overview(self, parent):
        panel, body = self._panel(parent, "Профиль высоты", "Последние 120 принятых значений · м")
        panel.pack(fill="x", pady=(0, 12))
        self.altitude_chart = AltitudeChart(body, self.font_family)
        self.altitude_chart.pack(fill="x")
        imu = ttk.Frame(parent)
        imu.pack(fill="x", pady=(0, 12))
        for index, (title, variable, unit) in enumerate((
                ("Акселерометр", self.acc_var, "X / Y / Z · g"),
                ("Гироскоп", self.gyr_var, "X / Y / Z · °/с"),
                ("Магнитометр", self.mag_var, "X / Y / Z · сырые значения"))):
            imu.columnconfigure(index, weight=1, uniform="imu")
            panel, body = self._panel(imu, title, unit)
            panel.grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 10, 0))
            ttk.Label(body, textvariable=variable, style="Mono.TLabel").pack(anchor="w")
        panel, body = self._panel(parent, "Режим полёта", "Ручное переключение · текущий режим приходит в телеметрии")
        panel.pack(fill="x", pady=(0, 4))
        mode_buttons = []
        for index, (name, command) in enumerate((("IDLE", 64), ("ARMED", 65), ("DROP", 66),
                                                ("RECOVERY", 67), ("GLIDE", 68), ("PARACHUTE", 69))):
            mode_buttons.append(ttk.Button(body, text=name, command=lambda cmd=command: self.send_sys_cmd(cmd)))
        mode_columns = None
        def fit_modes(event):
            nonlocal mode_columns
            columns = 6 if event.width >= 880 else 3
            if columns == mode_columns:
                return
            mode_columns = columns
            for index in range(6):
                body.columnconfigure(index, weight=1 if index < columns else 0,
                                     uniform="modes" if index < columns else "")
            for index, button in enumerate(mode_buttons):
                button.grid(row=index // columns, column=index % columns, sticky="ew",
                            padx=(0, 6), pady=(0, 6))
        body.bind("<Configure>", fit_modes)

    def _build_controls(self, parent):
        parent.columnconfigure((0, 1), weight=1, uniform="controls")
        self.servo1_var = tk.IntVar(value=90)
        self.servo2_var = tk.IntVar(value=90)
        for index, (variable, handler) in enumerate(((self.servo1_var, self.send_servo1),
                                                    (self.servo2_var, self.send_servo2)), 1):
            panel, body = self._panel(parent, f"Сервопривод {index}", "Положение · 0–180°")
            panel.grid(row=0, column=index - 1, sticky="nsew", padx=(0, 12 if index == 1 else 0), pady=(0, 12))
            slider_row = ttk.Frame(body, style="Panel.TFrame")
            slider_row.pack(fill="x", pady=(0, 12))
            label = ttk.Label(slider_row, text="90°", style="Value.TLabel", width=4, anchor="e")
            label.pack(side="right", padx=(12, 0))
            slider = ttk.Scale(slider_row, from_=0, to=180, variable=variable,
                               command=lambda value, target=label: target.configure(text=f"{int(float(value))}°"))
            slider.pack(side="left", fill="x", expand=True)
            setattr(self, f"servo{index}_slider", slider)
            setattr(self, f"servo{index}_label", label)
            ttk.Button(body, text=f"Отправить · Серво {index}", command=handler).pack(fill="x")
        panel, body = self._panel(parent, "Крылья", "Мотор F · раскрытие")
        panel.grid(row=1, column=0, sticky="nsew", padx=(0, 12), pady=(0, 12))
        body.columnconfigure((0, 1), weight=1)
        ttk.Button(body, text="Раскрыть", command=lambda: self.send_sys_cmd_param(19, 850)).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(body, text="Стоп F", style="Stop.TButton", command=lambda: self.send_sys_cmd_param(19, 0, urgent=True)).grid(row=0, column=1, sticky="ew")
        panel, body = self._panel(parent, "Рысканье", "Мотор E · ручной тест")
        panel.grid(row=1, column=1, sticky="nsew", pady=(0, 12))
        body.columnconfigure((0, 1, 2), weight=1)
        for index, (text, value) in enumerate((("Влево", -20), ("Вправо", 20), ("Стоп E", 0))):
            ttk.Button(body, text=text, style="Stop.TButton" if value == 0 else "TButton",
                       command=lambda val=value: self.send_sys_cmd_param(20, val, urgent=val == 0)).grid(row=0, column=index, sticky="ew", padx=(0, 6 if index < 2 else 0))
        panel, body = self._panel(parent, "Целевые углы", "Временные значения для текущего полёта")
        panel.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        self.live_pitch_var = tk.StringVar(value="-5")
        self.live_roll_var = tk.StringVar(value="0")
        self.live_yaw_var = tk.StringVar(value="0")
        for index, (title, variable, cmd) in enumerate((("Тангаж", self.live_pitch_var, 113),
                ("Крен", self.live_roll_var, 112), ("Рысканье", self.live_yaw_var, 114))):
            body.columnconfigure(index, weight=1, uniform="angles")
            field = ttk.Frame(body, style="Panel.TFrame")
            field.grid(row=0, column=index, sticky="ew", padx=(0, 12 if index < 2 else 0))
            ttk.Label(field, text=title + " · °", style="Panel.Muted.TLabel").pack(anchor="w", pady=(0, 8))
            ttk.Entry(field, textvariable=variable, width=6).pack(side="left", fill="x", expand=True, padx=(0, 8))
            ttk.Button(field, text="Задать",
                       command=lambda code=cmd, var=variable: self.send_live_angle(code, var.get())).pack(side="right")

    def _build_settings(self, parent):
        panel, body = self._panel(parent, "Калибровка датчиков", "Перед калибровкой установите планер в нужное положение")
        panel.pack(fill="x", pady=(0, 12))
        for index, (title, cmd) in enumerate((("Ноль высоты", 50), ("Гироскоп", 48),
                                            ("Акселерометр", 51), ("Магнитометр", 49))):
            body.columnconfigure(index % 2, weight=1, uniform="calibration")
            ttk.Button(body, text=title, command=lambda code=cmd: self.send_sys_cmd(code)).grid(
                row=index // 2, column=index % 2, sticky="ew", padx=(0, 8), pady=(0, 8))
        panel, body = self._panel(parent, "Коэффициенты PID", "Значение сохраняется на SD-карту борта")
        panel.pack(fill="x", pady=(0, 12))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        self.pid_param_var = tk.StringVar(value="0x60 (Roll Kp)")
        self.pid_val_var = tk.StringVar(value="1.00")
        ttk.Label(body, text="Параметр", style="Panel.Muted.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 8))
        ttk.Label(body, text="Значение", style="Panel.Muted.TLabel").grid(row=0, column=1, sticky="w", pady=(0, 8))
        combo = ttk.Combobox(body, textvariable=self.pid_param_var, state="readonly", width=18)
        combo["values"] = [f"0x{96 + i:02X} ({axis} {term})"
                           for i, (axis, term) in enumerate((a, t) for a in ("Roll", "Pitch", "Yaw") for t in ("Kp", "Ki", "Kd"))]
        combo.grid(row=1, column=0, sticky="ew", padx=(0, 12))
        ttk.Entry(body, textvariable=self.pid_val_var, width=10).grid(row=1, column=1, sticky="ew", padx=(0, 12))
        ttk.Button(body, text="Отправить и сохранить", command=self.send_pid, style="Accent.TButton").grid(row=1, column=2, sticky="ew")
        panel, body = self._panel(parent, "Повторная инициализация", "Перезапуск выбранного модуля")
        panel.pack(fill="x", pady=(0, 4))
        for index, (title, cmd) in enumerate((("BMI088", 32), ("LIS3MDL", 33), ("BMP388", 34), ("SD-карта", 35), ("I2C Expander", 36))):
            body.columnconfigure(index % 3, weight=1, uniform="reinit")
            ttk.Button(body, text=title, command=lambda code=cmd: self.send_sys_cmd(code)).grid(
                row=index // 3, column=index % 3, sticky="ew", padx=(0, 8), pady=(0, 8))

    def _build_log(self, parent):
        panel = tk.Frame(parent, bg=COLORS["panel"], highlightthickness=1, highlightbackground=COLORS["line"])
        panel.grid(row=3, column=0, sticky="ew", pady=(16, 0))
        toolbar = ttk.Frame(panel, style="Panel.TFrame", padding=(14, 8))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="Журнал событий", style="Panel.Title.TLabel").pack(side="left")
        ttk.Label(toolbar, text="Ошибки · команды · сообщения борта", style="Panel.Muted.TLabel").pack(side="left", padx=16)
        ttk.Button(toolbar, text="Очистить", command=self.clear_log, style="Quiet.TButton").pack(side="right")
        log_body = ttk.Frame(panel, style="Panel.TFrame", padding=(14, 0, 8, 10))
        log_body.pack(fill="both", expand=True)
        self.telemetry_text = tk.Text(log_body, wrap="word", height=4, width=1,
                                      bg=COLORS["panel"], fg=COLORS["muted"],
                                      font=(self.mono_family, 9), relief="flat", borderwidth=0,
                                      highlightthickness=0, padx=0, pady=4,
                                      selectbackground=COLORS["raised"], insertbackground=COLORS["accent"],
                                      state="disabled")
        self.telemetry_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_body, orient="vertical", command=self.telemetry_text.yview)
        scroll.pack(side="right", fill="y", padx=(8, 0))
        self.telemetry_text.configure(yscrollcommand=scroll.set)
        for tag, color in (("error", "red"), ("command", "blue"), ("session", "accent"), ("time", "muted")):
            self.telemetry_text.tag_configure(tag, foreground=COLORS[color])

    def clear_log(self):
        """Clear only the display; the on-disk session remains intact."""
        self.telemetry_text.configure(state="normal")
        self.telemetry_text.delete("1.0", "end")
        self.telemetry_text.configure(state="disabled")
