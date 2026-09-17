"""Shared Tk/ttk appearance and small display-only widgets. No radio I/O."""
from collections import deque
import math
import tkinter as tk
from tkinter import font as tkfont, ttk


COLORS = {
    "bg": "#0B111C", "sidebar": "#101A29", "panel": "#142033",
    "raised": "#1D2D43", "line": "#2A3B52", "text": "#EEF4FC",
    "muted": "#9AAEC7", "accent": "#56E0C1", "blue": "#82B1FF",
    "amber": "#F3C17A", "red": "#FF9BAA", "ink": "#09251F",
}


def configure_theme(root):
    """Use a portable ttk theme so Windows, macOS and Linux share the palette."""
    available = set(tkfont.families(root))
    family = next((f for f in ("Segoe UI", "SF Pro Text", "DejaVu Sans")
                   if f in available), "Helvetica")
    mono = next((f for f in ("Cascadia Code", "Consolas", "DejaVu Sans Mono")
                 if f in available), "Courier")
    tkfont.nametofont("TkDefaultFont").configure(family=family, size=10)
    tkfont.nametofont("TkTextFont").configure(family=family, size=10)
    root.configure(bg=COLORS["bg"])
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", font=(family, 10), background=COLORS["bg"],
                    foreground=COLORS["text"], bordercolor=COLORS["line"],
                    lightcolor=COLORS["line"], darkcolor=COLORS["line"])
    for prefix, background in (("", "bg"), ("Panel.", "panel"),
                               ("Side.", "sidebar")):
        style.configure(prefix + "TFrame", background=COLORS[background])
        style.configure(prefix + "TLabel", background=COLORS[background])
        style.configure(prefix + "Muted.TLabel", background=COLORS[background],
                        foreground=COLORS["muted"], font=(family, 9))
        style.configure(prefix + "Title.TLabel", background=COLORS[background],
                        font=(family, 11, "bold"))
    style.configure("Hero.TLabel", font=(family, 22, "bold"))
    style.configure("Brand.TLabel", background=COLORS["sidebar"],
                    font=(family, 20, "bold"))
    style.configure("Metric.TLabel", background=COLORS["panel"],
                    font=(family, 27, "bold"))
    style.configure("Value.TLabel", background=COLORS["panel"],
                    foreground=COLORS["accent"], font=(mono, 16, "bold"))
    style.configure("Mono.TLabel", background=COLORS["panel"],
                    font=(mono, 10))
    style.configure("State.TLabel", background=COLORS["sidebar"],
                    foreground=COLORS["accent"], font=(family, 18, "bold"))
    style.configure("TButton", background=COLORS["raised"],
                    foreground=COLORS["text"], padding=(12, 9),
                    borderwidth=1, relief="flat", focusthickness=2,
                    focuscolor=COLORS["blue"], anchor="center", width=0)
    style.map("TButton", background=[("disabled", COLORS["panel"]),
                                     ("pressed", "#314B68"),
                                     ("active", "#293E57")],
              foreground=[("disabled", "#6E839E")],
              bordercolor=[("focus", COLORS["blue"])])
    style.configure("Accent.TButton", background=COLORS["accent"],
                    foreground=COLORS["ink"], font=(family, 10, "bold"))
    style.map("Accent.TButton", background=[("pressed", "#39BFA3"),
                                            ("active", "#88EED5")],
              foreground=[("disabled", "#526D70")])
    style.configure("Stop.TButton", foreground=COLORS["red"],
                    background="#352736")
    style.map("Stop.TButton", background=[("pressed", "#633448"),
                                          ("active", "#4B2F41")])
    style.configure("Quiet.TButton", padding=(10, 5), foreground=COLORS["muted"])
    style.configure("TEntry", fieldbackground=COLORS["bg"],
                    foreground=COLORS["text"], insertcolor=COLORS["accent"],
                    padding=(10, 8), borderwidth=1)
    style.map("TEntry", bordercolor=[("focus", COLORS["accent"])])
    style.configure("TCombobox", fieldbackground=COLORS["bg"],
                    background=COLORS["raised"], foreground=COLORS["text"],
                    arrowcolor=COLORS["muted"], padding=(10, 8))
    style.map("TCombobox", fieldbackground=[("readonly", COLORS["bg"])],
              foreground=[("readonly", COLORS["text"]), ("disabled", COLORS["muted"])],
              selectbackground=[("readonly", COLORS["raised"])],
              selectforeground=[("readonly", COLORS["text"])],
              bordercolor=[("focus", COLORS["accent"])])
    root.option_add("*TCombobox*Listbox.background", COLORS["panel"])
    root.option_add("*TCombobox*Listbox.foreground", COLORS["text"])
    root.option_add("*TCombobox*Listbox.selectBackground", COLORS["raised"])
    root.option_add("*TCombobox*Listbox.selectForeground", COLORS["accent"])
    style.configure("Horizontal.TScale", background=COLORS["panel"],
                    troughcolor=COLORS["bg"], bordercolor=COLORS["line"])
    style.configure("TSeparator", background=COLORS["line"])
    style.configure("TNotebook", background=COLORS["bg"], borderwidth=0,
                    bordercolor=COLORS["bg"], lightcolor=COLORS["bg"], darkcolor=COLORS["bg"],
                    tabmargins=(0, 0, 0, 12))
    style.configure("TNotebook.Tab", padding=(20, 10), borderwidth=0,
                    background=COLORS["bg"], foreground=COLORS["muted"],
                    lightcolor=COLORS["bg"], darkcolor=COLORS["bg"], bordercolor=COLORS["bg"])
    style.map("TNotebook.Tab", background=[("selected", COLORS["panel"]),
                                           ("active", COLORS["raised"])],
              foreground=[("selected", COLORS["accent"]), ("active", COLORS["text"])],
              padding=[("selected", (20, 10)), ("!selected", (20, 10))],
              lightcolor=[("selected", COLORS["panel"])],
              darkcolor=[("selected", COLORS["panel"])],
              bordercolor=[("selected", COLORS["panel"])])
    style.configure("Vertical.TScrollbar", background=COLORS["line"],
                    troughcolor=COLORS["bg"], arrowcolor=COLORS["muted"],
                    borderwidth=0, arrowsize=11)
    return family, mono


class ScrollablePage(ttk.Frame):
    """A notebook page that stays usable on a short laptop display."""
    def __init__(self, parent):
        super().__init__(parent)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, bg=COLORS["bg"], highlightthickness=0,
                                borderwidth=0, width=1, height=1)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns", padx=(8, 0))
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.body = ttk.Frame(self.canvas)
        self.window_id = self.canvas.create_window(0, 0, window=self.body, anchor="nw")
        self.body.bind("<Configure>", lambda _: self.canvas.configure(
            scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(
            self.window_id, width=e.width))

    def bind_scrolling(self):
        # Local bindings only: scrolling one page never hijacks another widget.
        def bind_tree(widget):
            if not isinstance(widget, (ttk.Combobox, ttk.Scale, tk.Text)):
                widget.bind("<MouseWheel>", self._on_wheel, add="+")
                widget.bind("<Button-4>", self._on_wheel, add="+")
                widget.bind("<Button-5>", self._on_wheel, add="+")
            for child in widget.winfo_children():
                bind_tree(child)
        bind_tree(self.canvas)

    def _on_wheel(self, event):
        if self.body.winfo_height() <= self.canvas.winfo_height():
            return
        if getattr(event, "num", None) in (4, 5):
            step = -1 if event.num == 4 else 1
        else:
            delta = event.delta
            step = -int(delta / 120) if abs(delta) >= 120 else (-1 if delta > 0 else 1)
        self.canvas.yview_scroll(step, "units")
        return "break"


class AltitudeChart(tk.Canvas):
    """A bounded history of actual received samples, with no synthetic data."""
    def __init__(self, parent, font_family):
        super().__init__(parent, bg=COLORS["panel"], height=104,
                         highlightthickness=0, borderwidth=0)
        self.samples = deque(maxlen=120)
        self.font_family = font_family
        self.bind("<Configure>", lambda _: self.redraw())

    def add_sample(self, value):
        if math.isfinite(value):
            self.samples.append(value)
            self.redraw()

    def clear(self):
        self.samples.clear()
        self.redraw()

    def redraw(self):
        self.delete("all")
        width, height = max(self.winfo_width(), 100), max(self.winfo_height(), 100)
        left, right, top, bottom = 60, width - 16, 16, height - 26
        low, high = (min(self.samples), max(self.samples)) if self.samples else (0, 100)
        margin = max((high - low) * 0.15, 1)
        low, high = low - margin, high + margin
        for i in range(4):
            y = top + (bottom - top) * i / 3
            self.create_line(left, y, right, y, fill=COLORS["line"], dash=(3, 5))
            if self.samples:
                self.create_text(left - 10, y, text=f"{high - (high-low)*i/3:.1f}",
                                 anchor="e", fill=COLORS["muted"],
                                 font=(self.font_family, 8))
        if not self.samples:
            self.create_text(width / 2, height / 2, text="График появится после приёма телеметрии",
                             fill=COLORS["muted"], font=(self.font_family, 10))
            return
        points = []
        for index, sample in enumerate(self.samples):
            x = left + (right - left) * index / max(len(self.samples) - 1, 1)
            y = bottom - (sample - low) / (high - low) * (bottom - top)
            points.extend((x, y))
        if len(self.samples) > 1:
            self.create_polygon(left, bottom, *points, right, bottom,
                                fill="#193C43", outline="")
            self.create_line(*points, fill=COLORS["accent"], width=2)
        x, y = points[-2:]
        self.create_oval(x-4, y-4, x+4, y+4, fill=COLORS["accent"], outline=COLORS["panel"], width=2)
        self.create_text(left, height - 8, anchor="w", text="Ранее",
                         fill=COLORS["muted"], font=(self.font_family, 8))
        self.create_text(right, height - 8, anchor="e", text="Последний пакет",
                         fill=COLORS["muted"], font=(self.font_family, 8))
