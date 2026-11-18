"""GCS ROV: dashboard terintegrasi monitoring sensor dan kontrol thruster.

MODE
    SIMULASI : sensor dibangkitkan otomatis dengan random walk setiap 500 ms.
               Penguji dapat memicu skenario tidak ideal lewat tombol uji.
    REAL     : telemetri dibaca dari file JSON hasil logger ROV, dan perintah
               ditulis ke perintah_rov.txt. Tanpa pyserial/socket, jembatan
               berbasis file dipakai sebagai pengganti port serial.

ALUR KODE (untuk pengembangan lanjut)
    1. Konstanta       : warna, font, batas keselamatan, daftar sensor,
                         peta arah per thruster, dan parameter auto ascent.
    2. categorize()    : mengubah nilai sensor menjadi NORMAL/WARNING/BAHAYA.
    3. App.__init__    : menyiapkan state awal lalu memanggil _build_*().
    4. Pembangunan UI  : _build_header, _build_main (sensor + kontrol),
                         _build_log, _build_status_bar.
    5. Loop utama      : _tick() dipanggil tiap TICK_MS lewat after():
         a. sumber data  : _simulate_step() / _step_auto_ascent() / _read_real()
         b. LINK         : _update_link() memakai selisih waktu datetime
         c. keselamatan  : _update_safety() mengunci kontrol saat BAHAYA dan
                           memicu auto ascent bila baterai < 15 % di kedalaman
         d. kategori     : _log_level_changes() mencatat perubahan status
         e. tampilan     : _refresh_display()
       Tidak ada sleep() maupun while True.
    6. Skenario uji    : dropdown + tombol TERAPKAN di header memanggil
                         scenario_normal / scenario_low_battery /
                         scenario_hot, dan tombol link memutus data.
                         Hanya aktif di mode SIMULASI.
    7. Pemilihan       : _select_thruster() mengganti opsi arah sesuai
                         thruster (Vertikal memakai Naik/Turun).
    8. Perintah        : send_command() -> _check_interlock() -> motor[]
                         -> log (dan _write_cmd() di mode REAL). Tombol
                         "Isi: ..." menyiapkan teks perintah di kolom log.
    9. Darurat         : kill_switch() menghentikan semua thruster.
   10. Penutupan       : jendela tidak bisa ditutup dengan X; hanya lewat
                         tombol TUTUP APLIKASI (close_app()).

SKENARIO OTOMATIS SAAT BAHAYA
    - Suhu > 45 °C atau baterai < 15 % -> kontrol dikunci, tombol BUKA KUNCI
      aktif untuk override manual.
    - Baterai < 15 % saat ROV masih di kedalaman -> auto ascent menghitung
      sisa baterai dan jarak ke permukaan, lalu menaikkan ROV otomatis
      sambil mengunci seluruh kontrol.
    - Saat kondisi kembali normal, kunci dibuka otomatis.

CARA MENAMBAH SENSOR
    Tambahkan entri di SENSOR_SPECS, aturan kategori di categorize(), dan
    logika simulasi di _simulate_step(). Kartu tampilan dibuat otomatis.

CARA MENAMBAH THRUSTER
    Tambahkan nama di THRUSTERS dan pasangan arahnya di THRUSTER_DIRECTIONS.
    Tombol pilih dan kartu status akan mengikuti otomatis.

TAMPILAN OUTDOOR
    Palet kontras tinggi (teks hampir hitam, kartu putih, batas tebal) dan
    font besar agar terbaca di bawah sinar matahari. Window 1240x840.

ASUMSI
    Batas WARNING suhu (40 °C), baterai (30 %), dan kedalaman (8 m, batas
    maks 10 m) tidak ada di studi kasus dan ditetapkan sendiri. Konsumsi
    baterai saat naik 0.4 % per meter dan laju naik maksimal 0.8 m/tick
    juga asumsi untuk simulasi.
"""

import json
import random
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from datetime import datetime

# ---------------------------------------------------------------------------
# 1. KONSTANTA
# ---------------------------------------------------------------------------
# Palet kontras tinggi untuk pemakaian outdoor
BG = "#e8e2da"
CARD = "#ffffff"
BORDER = "#8a7468"
BROWN = "#3d2418"
BROWN_ACTIVE = "#1f120c"
CREAM = "#fff6ec"
TEXT = "#1f1510"
MUTED = "#5f4d43"
GREEN = "#1e7a3e"
AMBER = "#a35f00"
RED = "#b3261e"
RED_ACTIVE = "#8c1c16"
LOG_BG = "#1b110c"
LOG_RED = "#ff8a7a"
LOG_GREEN = "#7bffa8"
LOG_AMBER = "#ffd24a"

# Warna latar kartu sesuai kategori sensor
CARD_TINTS = {"NORMAL": CARD, "WARNING": "#fff3d1", "BAHAYA": "#ffdfda"}
MOTOR_ACTIVE_BG = "#e4f6ea"

# Font besar agar terbaca di luar ruangan
FONT = "Segoe UI"
F_MINI = (FONT, 9, "bold")
F_SMALL = (FONT, 11)
F_BOLD = (FONT, 11, "bold")
F_TITLE = (FONT, 16, "bold")
F_MOTOR = (FONT, 17, "bold")
F_VALUE = (FONT, 22, "bold")
F_KILL = (FONT, 16, "bold")

# Batas keselamatan (studi kasus) dan batas peringatan (asumsi)
TEMP_WARN = 40.0
TEMP_MAX = 45.0
BATT_WARN = 30.0
BATT_MIN = 15.0
DEPTH_WARN = 8.0
DEPTH_MAX = 10.0

# Waktu dan link
SPEED_MAX = 100
SPEED_STEP = 5
LINK_TIMEOUT = 2.0
TICK_MS = 500
CMD_FILE = "perintah_rov.txt"

# Model simulasi sederhana
SIM_TEMP_BASE = 28.0
SIM_COOL_RATE = 0.002
SIM_HEAT_PER_SPEED = 0.0002
SIM_BATT_IDLE_DRAIN = 0.01
SIM_BATT_LOAD_DRAIN = 0.0003

# Auto ascent: konsumsi baterai per meter dan batas laju naik
BATT_PER_METER = 0.4
ASCENT_RATE_MIN = 0.15
ASCENT_RATE_MAX = 0.8
ASCENT_MOTOR_SPEED = 60

THRUSTERS = [
    "Kiri Depan",
    "Kanan Depan",
    "Kiri Belakang",
    "Kanan Belakang",
    "Vertikal",
]
# Thruster vertikal bergerak naik/turun, bukan maju/mundur
THRUSTER_DIRECTIONS = {
    "Kiri Depan": ("Maju", "Mundur"),
    "Kanan Depan": ("Maju", "Mundur"),
    "Kiri Belakang": ("Maju", "Mundur"),
    "Kanan Belakang": ("Maju", "Mundur"),
    "Vertikal": ("Naik", "Turun"),
}
DEFAULT_DIRECTIONS = ("Maju", "Mundur")

# Sensor: kunci -> (label, satuan, nilai awal)
SENSOR_SPECS = {
    "suhu": ("Suhu Air", "°C", 28.0),
    "kedalaman": ("Kedalaman", "m", 3.0),
    "baterai": ("Sisa Baterai", "%", 95.0),
}
LEVEL_COLORS = {"NORMAL": GREEN, "WARNING": AMBER, "BAHAYA": RED}
LEVEL_TAGS = {"NORMAL": "green", "WARNING": "amber", "BAHAYA": "red"}
LEVEL_RANK = {"NORMAL": 0, "WARNING": 1, "BAHAYA": 2}


def categorize(key, value):
    """Mengembalikan kategori NORMAL, WARNING, atau BAHAYA untuk sensor."""
    if key == "suhu":
        if value > TEMP_MAX:
            return "BAHAYA"
        if value >= TEMP_WARN:
            return "WARNING"
        return "NORMAL"
    if key == "baterai":
        if value < BATT_MIN:
            return "BAHAYA"
        if value < BATT_WARN:
            return "WARNING"
        return "NORMAL"
    if value > DEPTH_MAX:
        return "BAHAYA"
    if value > DEPTH_WARN:
        return "WARNING"
    return "NORMAL"


def clamp(value, low, high):
    """Membatasi nilai ke rentang [low, high]."""
    return max(low, min(high, value))


class App(tk.Tk):
    """Jendela utama GCS ROV. Lihat docstring modul untuk alur kode."""

    # -----------------------------------------------------------------
    # 3. INISIALISASI
    # -----------------------------------------------------------------
    def __init__(self):
        super().__init__()
        self.title("GCS ROV - Dashboard Terintegrasi")
        self.geometry("1240x840")
        self.minsize(1240, 800)
        self.configure(bg=BG)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.protocol("WM_DELETE_WINDOW", self._on_close_request)

        self.started_at = datetime.now()
        self.tick_count = 0
        self.mode = "SIMULASI"
        self.real_path = None
        self.last_raw = None
        self.link_cut = False
        self.link_ok = True
        self.last_data_time = datetime.now()
        self.sensor = {key: spec[2] for key, spec in SENSOR_SPECS.items()}
        self.levels = {key: categorize(key, self.sensor[key])
                       for key in SENSOR_SPECS}
        self.telemetry = {"heading": 180.0, "pitch": 0.0, "roll": 0.0}
        self.motor = {name: {"speed": 0, "dir": "-"} for name in THRUSTERS}

        # Keselamatan: kunci kontrol, override manual, dan auto ascent
        self.control_locked = False
        self.lock_reason = ""
        self.override_hazard = False
        self.auto_ascent = False
        self.ascent_rate = 0.5

        self.thruster_var = tk.StringVar(value="")
        self.speed_var = tk.IntVar(value=0)
        self.direction_var = tk.StringVar(value=DEFAULT_DIRECTIONS[0])
        self.scenario_var = tk.StringVar(value="Kondisi Normal")
        self.scenario_actions = {
            "Kondisi Normal": self.scenario_normal,
            "Baterai < 15 %": self.scenario_low_battery,
            "Suhu 46 °C": self.scenario_hot,
        }

        self.cards = {}
        self.motor_cards = {}
        self.thruster_toggles = []
        self.dir_toggles = []
        self.scenario_combo = None
        self.btn_apply_scenario = None
        self.btn_link = None
        self.btn_unlock = None
        self._job = None

        self._setup_style()
        self._build_header()
        self._build_main()
        self._build_kill_switch()
        self._build_log()
        self._build_status_bar()
        self._apply_mode_widgets()
        self.log("Sistem siap dalam mode SIMULASI. Sensor berubah otomatis.",
                 "info")
        self._job = self.after(TICK_MS, self._tick)

    # -----------------------------------------------------------------
    # 4. PEMBANGUNAN UI
    # -----------------------------------------------------------------
    def _setup_style(self):
        """Mengatur tema kontras tinggi untuk widget ttk."""
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=TEXT, font=F_SMALL)
        style.configure("TLabelframe", background=CARD, bordercolor=BORDER,
                        borderwidth=2)
        style.configure("TLabelframe.Label", background=CARD,
                        foreground=BROWN, font=F_BOLD)
        style.configure("TFrame", background=CARD)
        style.configure("TLabel", background=CARD, foreground=TEXT,
                        font=F_SMALL)
        style.configure("Brown.TButton", background=BROWN, foreground=CREAM,
                        font=F_BOLD, padding=8, borderwidth=0)
        style.map("Brown.TButton", background=[("active", BROWN_ACTIVE),
                                               ("disabled", BORDER)],
                  foreground=[("disabled", MUTED)])
        style.configure("Vertical.TScrollbar", background=BORDER,
                        troughcolor=CARD, width=16)

    def _build_header(self):
        """Judul, pemilih mode, status LINK, status sistem, dan jam."""
        bar = tk.Frame(self, bg=BROWN)
        bar.grid(row=0, column=0, sticky="ew", padx=6, pady=6)
        bar.grid_columnconfigure(0, weight=1)

        tk.Label(bar, text="GCS ROV - DASHBOARD TERINTEGRASI", bg=BROWN,
                 fg=CREAM, font=F_TITLE).grid(row=0, column=0, sticky="w",
                                              padx=10, pady=8)

        mode_box = tk.Frame(bar, bg=BROWN)
        mode_box.grid(row=0, column=1, padx=6)
        self.btn_sim = tk.Button(mode_box, text="SIMULASI", font=F_BOLD,
                                 relief="flat", padx=14, pady=6,
                                 cursor="hand2", command=self.set_mode_sim)
        self.btn_sim.grid(row=0, column=0, padx=(0, 4))
        self.btn_real = tk.Button(mode_box, text="REAL", font=F_BOLD,
                                  relief="flat", padx=14, pady=6,
                                  cursor="hand2", command=self.set_mode_real)
        self.btn_real.grid(row=0, column=1)

        self.lbl_link = tk.Label(bar, bg=BROWN, fg=CREAM, font=F_BOLD,
                                 padx=10)
        self.lbl_link.grid(row=0, column=2, padx=4)
        self.lbl_system = tk.Label(bar, bg=GREEN, fg="white", font=F_BOLD,
                                   padx=10, pady=4)
        self.lbl_system.grid(row=0, column=3, padx=4)
        self.lbl_clock = tk.Label(bar, bg=BROWN, fg=CREAM, font=F_BOLD,
                                  padx=10)
        self.lbl_clock.grid(row=0, column=4, padx=6)

        # Baris kedua header: pemilih skenario uji dan tombol link
        scen_row = tk.Frame(bar, bg=BROWN)
        scen_row.grid(row=1, column=0, columnspan=5, sticky="ew", padx=10,
                      pady=(0, 8))
        tk.Label(scen_row, text="SKENARIO UJI:", bg=BROWN, fg=CREAM,
                 font=F_BOLD).grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.scenario_combo = ttk.Combobox(
            scen_row, textvariable=self.scenario_var,
            values=list(self.scenario_actions), state="readonly", width=22,
            font=F_SMALL)
        self.scenario_combo.grid(row=0, column=1, sticky="w")
        self.btn_apply_scenario = tk.Button(
            scen_row, text="TERAPKAN", command=self.apply_scenario,
            bg=CREAM, fg=BROWN, activebackground=CARD,
            activeforeground=BROWN, font=F_BOLD, relief="flat", padx=12,
            pady=4, cursor="hand2")
        self.btn_apply_scenario.grid(row=0, column=2, sticky="w", padx=6)
        self.btn_link = tk.Button(
            scen_row, text="Putuskan Link", command=self.toggle_link,
            bg=BROWN_ACTIVE, fg=CREAM, activebackground=RED,
            activeforeground="white", font=F_BOLD, relief="flat", padx=12,
            pady=4, cursor="hand2")
        self.btn_link.grid(row=0, column=3, sticky="w", padx=6)
        tk.Label(scen_row, text="(hanya mode SIMULASI)", bg=BROWN,
                 fg=CREAM, font=F_SMALL).grid(row=0, column=4, sticky="w",
                                              padx=6)

    def _build_main(self):
        """Kiri: monitoring sensor. Kanan: kontrol thruster."""
        main = tk.Frame(self, bg=BG)
        main.grid(row=1, column=0, sticky="nsew", padx=6)
        main.grid_rowconfigure(0, weight=1)
        main.grid_columnconfigure(0, weight=1, uniform="kolom")
        main.grid_columnconfigure(1, weight=1, uniform="kolom")
        self._build_sensor_panel(main)
        self._build_control_panel(main)

    def _build_sensor_panel(self, parent):
        """Kartu sensor berkategori, telemetri, dan skenario uji."""
        frame = ttk.LabelFrame(parent, text="MONITORING SENSOR", padding=6)
        frame.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        for col in range(len(SENSOR_SPECS)):
            frame.grid_columnconfigure(col, weight=1, uniform="kartu")

        for col, (key, spec) in enumerate(SENSOR_SPECS.items()):
            card = tk.Frame(frame, bg=CARD, highlightthickness=3,
                            highlightbackground=BORDER)
            card.grid(row=0, column=col, sticky="nsew", padx=3, pady=3)
            title = tk.Label(card, text=spec[0].upper(), bg=CARD, fg=MUTED,
                             font=F_BOLD, anchor="w")
            title.pack(fill="x", padx=6, pady=(6, 0))
            value = tk.Label(card, text="-", bg=CARD, fg=TEXT, font=F_VALUE,
                             anchor="w")
            value.pack(fill="x", padx=6)
            badge = tk.Label(card, text="-", bg=CARD, fg=MUTED, font=F_BOLD,
                             anchor="w")
            badge.pack(fill="x", padx=6, pady=(0, 6))
            self.cards[key] = {"frame": card, "title": title, "value": value,
                               "badge": badge}

        self.lbl_telemetry = tk.Label(frame, bg=CARD, fg=TEXT, font=F_BOLD,
                                      anchor="w")
        self.lbl_telemetry.grid(row=1, column=0, columnspan=3, sticky="ew",
                                padx=3, pady=(6, 0))

        self.lbl_source = tk.Label(frame, bg=CARD, fg=MUTED, font=F_SMALL,
                                   anchor="w", justify="left")
        self.lbl_source.grid(row=2, column=0, columnspan=3, sticky="ew",
                             padx=3, pady=(8, 0))

    def _build_control_panel(self, parent):
        """Tiga blok kontrol, lalu kartu status tiap thruster."""
        frame = ttk.LabelFrame(parent, text="KONTROL THRUSTER", padding=6)
        frame.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(2, weight=1)

        self._build_thruster_picker(frame)
        self._build_speed_slider(frame)
        self._build_drive_panel(frame)
        self._build_motor_status(frame)

    def _build_thruster_picker(self, parent):
        """Tombol pilih thruster: satu klik, langsung terlihat terpilih."""
        box = ttk.LabelFrame(parent, text="1. PILIH THRUSTER", padding=6)
        box.grid(row=0, column=0, sticky="ns", padx=6)
        for idx, name in enumerate(THRUSTERS):
            row, col = divmod(idx, 2)
            span = 2 if idx == len(THRUSTERS) - 1 else 1
            btn = tk.Button(box, text=name, font=F_BOLD, width=13,
                            relief="groove", bd=2, bg=CARD, fg=TEXT,
                            activebackground=CREAM, activeforeground=TEXT,
                            cursor="hand2",
                            command=lambda n=name: self._select_thruster(n))
            btn.grid(row=row, column=col, columnspan=span, sticky="ew",
                     padx=3, pady=3)
            self.thruster_toggles.append((btn, self.thruster_var, name))

    def _build_speed_slider(self, parent):
        """Slider kecepatan vertikal 0-100 % dengan tombol langkah."""
        box = ttk.LabelFrame(parent, text="2. KECEPATAN", padding=6)
        box.grid(row=0, column=1, sticky="ns", padx=6)
        tk.Scale(box, from_=SPEED_MAX, to=0, orient="vertical",
                 variable=self.speed_var, showvalue=False, length=170,
                 width=26, bg=CARD, fg=TEXT, troughcolor=BORDER,
                 activebackground=BROWN, highlightthickness=0,
                 sliderrelief="raised", bd=2,
                 command=lambda _v: self._refresh_display()).grid(
            row=0, column=0, columnspan=3, pady=(0, 6))
        for col, (text, delta) in enumerate(
                (("-5", -SPEED_STEP), ("0", -SPEED_MAX),
                 ("+5", SPEED_STEP))):
            ttk.Button(box, text=text, style="Brown.TButton", width=4,
                       command=lambda d=delta: self._step_speed(d)).grid(
                row=1, column=col, padx=2)
        self.lbl_target = tk.Label(box, bg=CARD, fg=BROWN, font=F_BOLD)
        self.lbl_target.grid(row=2, column=0, columnspan=3, pady=(6, 0))

    def _build_drive_panel(self, parent):
        """Arah putaran, tombol kirim, override, dan kecepatan aktual."""
        box = ttk.LabelFrame(parent, text="3. ARAH & KIRIM", padding=6)
        box.grid(row=0, column=2, sticky="nsew", padx=6)
        box.grid_columnconfigure(0, weight=1)

        ttk.Label(box, text="ARAH PUTARAN").grid(row=0, column=0,
                                                 sticky="w", pady=(0, 4))
        self.dir_box = ttk.Frame(box)
        self.dir_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        self._rebuild_directions(DEFAULT_DIRECTIONS)

        self.btn_send = ttk.Button(box, text="KIRIM PERINTAH",
                                   style="Brown.TButton",
                                   command=self.send_command)
        self.btn_send.grid(row=2, column=0, sticky="ew", pady=(0, 4))

        self.btn_unlock = ttk.Button(box, text="TIDAK ADA KENDALA",
                                     style="Brown.TButton",
                                     command=self.unlock_controls)
        self.btn_unlock.grid(row=3, column=0, sticky="ew", pady=(0, 8))

        tk.Label(box, text="KECEPATAN AKTUAL", bg=CARD, fg=MUTED,
                 font=F_BOLD, anchor="w").grid(row=4, column=0, sticky="ew")
        self.lbl_speed = tk.Label(box, bg=CARD, fg=BROWN, font=F_VALUE,
                                  anchor="w")
        self.lbl_speed.grid(row=5, column=0, sticky="ew")

    def _build_motor_status(self, parent):
        """Kartu status tiap thruster: nama, kecepatan, dan arah."""
        box = ttk.LabelFrame(parent, text="STATUS THRUSTER (AKTUAL)",
                             padding=6)
        box.grid(row=1, column=0, columnspan=3, sticky="ew", padx=6,
                 pady=(6, 0))
        for col in range(len(THRUSTERS)):
            box.grid_columnconfigure(col, weight=1, uniform="motor")

        for col, name in enumerate(THRUSTERS):
            card = tk.Frame(box, bg=CARD, highlightthickness=2,
                            highlightbackground=BORDER)
            card.grid(row=0, column=col, sticky="nsew", padx=3, pady=3)
            title = tk.Label(card, text=name.upper(), bg=CARD, fg=MUTED,
                             font=F_MINI, anchor="w")
            title.pack(fill="x", padx=5, pady=(5, 0))
            value = tk.Label(card, text="0 %", bg=CARD, fg=TEXT,
                             font=F_MOTOR, anchor="w")
            value.pack(fill="x", padx=5)
            direction = tk.Label(card, text="-", bg=CARD, fg=MUTED,
                                 font=F_SMALL, anchor="w")
            direction.pack(fill="x", padx=5, pady=(0, 5))
            self.motor_cards[name] = (card, title, value, direction)

    def _rebuild_directions(self, options):
        """Membuat ulang tombol arah sesuai thruster yang dipilih."""
        for child in self.dir_box.winfo_children():
            child.destroy()
        self.dir_toggles = []
        for col, name in enumerate(options):
            btn = tk.Button(self.dir_box, text=name.upper(), font=F_BOLD,
                            relief="groove", bd=2, bg=CARD, fg=TEXT,
                            activebackground=CREAM, activeforeground=TEXT,
                            cursor="hand2",
                            command=lambda n=name: self._select_direction(n))
            btn.grid(row=0, column=col, sticky="ew", padx=(0, 6))
            self.dir_box.grid_columnconfigure(col, weight=1)
            self.dir_toggles.append((btn, self.direction_var, name))
        self._refresh_toggles(self.dir_toggles)

    def _build_kill_switch(self):
        """Tombol darurat selebar layar agar mudah ditemukan."""
        tk.Button(self, text="KILL SWITCH / EMERGENCY STOP",
                  command=self.kill_switch, bg=RED, fg="white",
                  activebackground=RED_ACTIVE, activeforeground="white",
                  font=F_KILL, relief="flat", bd=0, pady=10,
                  cursor="hand2").grid(row=2, column=0, sticky="ew", padx=6,
                                       pady=(0, 6))

    def _build_log(self):
        """Log aktivitas, tombol isi perintah, dan kolom kirim perintah."""
        frame = ttk.LabelFrame(self, text="LOG AKTIVITAS & KIRIM PERINTAH",
                               padding=6)
        frame.grid(row=3, column=0, sticky="ew", padx=6, pady=6)
        frame.grid_columnconfigure(0, weight=1)

        self.log_text = tk.Text(frame, bg=LOG_BG, fg=CREAM, font=F_SMALL,
                                wrap="word", state="disabled", relief="flat",
                                padx=6, pady=6, height=6,
                                insertbackground=CREAM)
        scroll = ttk.Scrollbar(frame, orient="vertical",
                               command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.grid(row=0, column=0, columnspan=3, sticky="nsew")
        scroll.grid(row=0, column=3, sticky="ns")

        self.log_text.tag_configure("red", foreground=LOG_RED, font=F_BOLD)
        self.log_text.tag_configure("amber", foreground=LOG_AMBER)
        self.log_text.tag_configure("green", foreground=LOG_GREEN)
        self.log_text.tag_configure("info", foreground=CREAM)

        self.cmd_entry = ttk.Entry(frame, font=F_SMALL)
        self.cmd_entry.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self.cmd_entry.bind("<Return>", lambda _e: self.send_raw())
        ttk.Button(frame, text="Kirim", style="Brown.TButton",
                   command=self.send_raw).grid(row=1, column=1, padx=6,
                                               pady=(6, 0))
        ttk.Button(frame, text="Bersihkan Log", style="Brown.TButton",
                   command=self.clear_log).grid(row=1, column=2,
                                                pady=(6, 0))

        quick = ttk.Frame(frame)
        quick.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        for col in range(4):
            quick.grid_columnconfigure(col, weight=1, uniform="quick")
        quick_specs = [
            ("Isi: Perintah Motor", self.fill_motor_command),
            ("Isi: STATUS", lambda: self._set_command("STATUS")),
            ("Isi: PING", lambda: self._set_command("PING")),
            ("Isi: STOP", lambda: self._set_command("STOP")),
        ]
        for col, (text, cmd) in enumerate(quick_specs):
            ttk.Button(quick, text=text, style="Brown.TButton",
                       command=cmd).grid(row=0, column=col, sticky="ew",
                                         padx=3)

    def _build_status_bar(self):
        """Pesan status, info waktu operasi, dan tombol penutupan aplikasi."""
        bar = tk.Frame(self, bg=BROWN)
        bar.grid(row=4, column=0, sticky="ew")
        bar.grid_columnconfigure(0, weight=1)
        self.lbl_status = tk.Label(bar, bg=BROWN, fg=CREAM, font=F_SMALL,
                                   anchor="w", padx=6, pady=4)
        self.lbl_status.grid(row=0, column=0, sticky="ew")
        self.lbl_info = tk.Label(bar, bg=BROWN, fg=CREAM, font=F_SMALL,
                                 padx=6)
        self.lbl_info.grid(row=0, column=1, sticky="e")
        tk.Button(bar, text="TUTUP APLIKASI", command=self.close_app,
                  bg=BROWN_ACTIVE, fg=CREAM, activebackground=RED,
                  activeforeground="white", font=F_BOLD, relief="flat",
                  padx=12, cursor="hand2").grid(row=0, column=2, padx=6,
                                                pady=2)

    # -----------------------------------------------------------------
    # Utilitas pesan, log, dan tombol toggle
    # -----------------------------------------------------------------
    def set_status(self, text):
        """Menampilkan pesan pada baris status."""
        self.lbl_status.configure(text=text)

    def log(self, message, level="info"):
        """Menambahkan baris log ber-timestamp."""
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n", level)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        """Mengosongkan area log."""
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log("Log dibersihkan.", "info")

    @staticmethod
    def _refresh_toggles(toggles):
        """Mewarnai tombol: terpilih cokelat teks krem, sisanya putih."""
        for btn, var, value in toggles:
            selected = var.get() == value
            btn.configure(bg=BROWN if selected else CARD,
                          fg=CREAM if selected else TEXT,
                          activebackground=BROWN if selected else CREAM,
                          activeforeground=CREAM if selected else TEXT,
                          relief="sunken" if selected else "groove")

    # -----------------------------------------------------------------
    # Mode simulasi / real
    # -----------------------------------------------------------------
    def set_mode_sim(self):
        """Beralih ke mode simulasi: sensor dibangkitkan otomatis."""
        if self.mode == "SIMULASI":
            return
        self.mode = "SIMULASI"
        self.real_path = None
        self.link_cut = False
        self.last_data_time = datetime.now()
        self._apply_mode_widgets()
        self.log("Mode diubah ke SIMULASI. Sensor dibangkitkan otomatis.",
                 "info")
        self.set_status("Mode SIMULASI aktif")

    def set_mode_real(self):
        """Beralih ke mode real: telemetri dibaca dari file JSON."""
        path = filedialog.askopenfilename(
            title="Pilih file telemetri ROV (JSON)",
            filetypes=[("JSON", "*.json"), ("Semua file", "*.*")])
        if not path:
            self.set_status("Pindah ke mode REAL dibatalkan")
            return
        self.mode = "REAL"
        self.real_path = path
        self.last_raw = None
        self.last_data_time = datetime.now()
        self._apply_mode_widgets()
        self.log(f"Mode diubah ke REAL. Telemetri: {path}", "green")
        self.log(f"Perintah ditulis ke {CMD_FILE}.", "info")
        self.set_status("Mode REAL aktif")

    def _apply_mode_widgets(self):
        """Menyesuaikan tombol dan label sumber data dengan mode aktif."""
        is_sim = self.mode == "SIMULASI"
        state = "normal" if is_sim else "disabled"
        self.scenario_combo.configure(
            state="readonly" if is_sim else "disabled")
        for btn in (self.btn_apply_scenario, self.btn_link):
            btn.configure(state=state)

        for name, btn in (("SIMULASI", self.btn_sim),
                          ("REAL", self.btn_real)):
            selected = name == self.mode
            btn.configure(bg=CREAM if selected else BROWN_ACTIVE,
                          fg=BROWN if selected else CREAM,
                          activebackground=CREAM if selected else BROWN_ACTIVE,
                          activeforeground=BROWN if selected else CREAM)

        if is_sim:
            source = "Sumber data: simulasi otomatis (random walk 500 ms)"
        else:
            name = self.real_path.replace("\\", "/").split("/")[-1]
            source = f"Sumber data: file {name}"
        self.lbl_source.configure(text=source)
        self._refresh_display()

    # -----------------------------------------------------------------
    # 5. LOOP UTAMA
    # -----------------------------------------------------------------
    def _tick(self):
        """Dipanggil tiap TICK_MS: data, LINK, keselamatan, kategori, UI."""
        self.tick_count += 1
        if self.mode == "REAL":
            self._read_real()
        else:
            self._simulate_step()
        self._update_link()
        self._update_safety()
        self._log_level_changes()
        self._refresh_display()
        self._job = self.after(TICK_MS, self._tick)

    def _simulate_step(self):
        """Mengubah nilai sensor sedikit demi sedikit secara acak."""
        if self.link_cut:
            return
        if self.auto_ascent:
            self._step_auto_ascent()
            return
        total = sum(m["speed"] for m in self.motor.values())
        s = self.sensor

        # Suhu: memanas sesuai beban motor, mendingin ke suhu dasar
        heat = total * SIM_HEAT_PER_SPEED
        cool = (s["suhu"] - SIM_TEMP_BASE) * SIM_COOL_RATE
        s["suhu"] = clamp(s["suhu"] + random.uniform(-0.1, 0.1) + heat
                          - cool, -5.0, 100.0)

        # Baterai: turun pelan, lebih cepat saat motor menyala
        drain = SIM_BATT_IDLE_DRAIN + total * SIM_BATT_LOAD_DRAIN
        s["baterai"] = clamp(s["baterai"] - drain, 0.0, 100.0)

        # Kedalaman: berayun kecil di sekitar posisi kolam uji
        s["kedalaman"] = clamp(s["kedalaman"] + random.uniform(-0.1, 0.1),
                               0.0, 9.5)

        self._simulate_orientation()
        self.last_data_time = datetime.now()

    def _simulate_orientation(self):
        """Mengubah heading, pitch, dan roll secara halus."""
        t = self.telemetry
        t["heading"] = (t["heading"] + random.uniform(-1.0, 1.0)) % 360
        t["pitch"] = clamp(t["pitch"] + random.uniform(-0.3, 0.3), -20, 20)
        t["roll"] = clamp(t["roll"] + random.uniform(-0.3, 0.3), -20, 20)

    def _step_auto_ascent(self):
        """Menaikkan ROV otomatis sampai permukaan sambil memakai baterai."""
        s = self.sensor
        s["kedalaman"] = max(0.0, s["kedalaman"] - self.ascent_rate)
        s["baterai"] = clamp(s["baterai"] - self.ascent_rate * BATT_PER_METER,
                             0.0, 100.0)
        self.motor["Vertikal"] = {"speed": ASCENT_MOTOR_SPEED, "dir": "Naik"}
        self.telemetry["pitch"] = -5.0
        self._simulate_orientation()
        self.last_data_time = datetime.now()

        if s["baterai"] <= 0:
            self.auto_ascent = False
            self.motor["Vertikal"] = {"speed": 0, "dir": "Naik"}
            self.log("BATERAI HABIS sebelum mencapai permukaan. "
                     f"ROV berhenti di {s['kedalaman']:.1f} m.", "red")
            return

        if s["kedalaman"] <= 0.05:
            s["kedalaman"] = 0.0
            self.auto_ascent = False
            self.motor["Vertikal"] = {"speed": 0, "dir": "Naik"}
            self.telemetry["pitch"] = 0.0
            self.log("ROV mencapai permukaan (0.0 m). Kontrol tetap terkunci "
                     "sampai kondisi aman.", "green")
            self.set_status("Auto ascent selesai - ROV di permukaan")

    def _read_real(self):
        """Membaca file telemetri JSON; data baru memperbarui waktu terakhir."""
        try:
            with open(self.real_path, encoding="utf-8") as handle:
                raw = handle.read()
        except OSError:
            return
        if raw == self.last_raw:
            return
        self.last_raw = raw
        try:
            data = json.loads(raw)
            values = {}
            for key, spec in SENSOR_SPECS.items():
                values[key] = self._parse(str(data[key]), spec[0],
                                          *self._range_of(key))
        except (ValueError, KeyError, TypeError) as err:
            self.log(f"Data real tidak valid: {err}", "amber")
            return
        self.sensor.update(values)
        self.last_data_time = datetime.now()

    @staticmethod
    def _range_of(key):
        """Rentang nilai yang diterima dari file telemetri."""
        ranges = {"suhu": (-5.0, 100.0), "kedalaman": (0.0, 11000.0),
                  "baterai": (0.0, 100.0)}
        return ranges[key]

    def _data_age(self):
        """Selisih detik sejak data terakhir diterima."""
        return (datetime.now() - self.last_data_time).total_seconds()

    def _update_link(self):
        """Menentukan LINK OK atau LOST dari selisih waktu data terakhir."""
        ok = self._data_age() <= LINK_TIMEOUT
        if ok != self.link_ok:
            if ok:
                self.log("LINK OK kembali. Kontrol dibuka.", "green")
            else:
                self.log("LINK LOST: data tidak diperbarui > 2 detik. "
                         "Kontrol dikunci.", "red")
            self.link_ok = ok

    def _log_level_changes(self):
        """Mencatat ke log setiap kali kategori sensor berubah."""
        for key, spec in SENSOR_SPECS.items():
            level = categorize(key, self.sensor[key])
            if level != self.levels[key]:
                self.log(f"{spec[0]}: {self.levels[key]} -> {level} "
                         f"({self.sensor[key]:.1f} {spec[1]})",
                         LEVEL_TAGS[level])
                self.levels[key] = level

    # -----------------------------------------------------------------
    # 5c. KESELAMATAN: KUNCI KONTROL DAN AUTO ASCENT
    # -----------------------------------------------------------------
    def _update_safety(self):
        """Mengunci kontrol saat BAHAYA dan memicu auto ascent baterai kritis."""
        hazard = self._danger_reasons()
        if hazard:
            if not self.control_locked and not self.override_hazard:
                self._lock_controls(" ".join(hazard))
            low_batt = self.sensor["baterai"] < BATT_MIN
            if low_batt and self.sensor["kedalaman"] > 0.2 \
                    and not self.auto_ascent:
                self._start_auto_ascent()
        elif self.control_locked or self.override_hazard:
            self.control_locked = False
            self.override_hazard = False
            self.lock_reason = ""
            self.log("Kondisi aman kembali. Kontrol dibuka otomatis.",
                     "green")
            self.set_status("Kontrol dibuka: kondisi sensor normal")

    def _lock_controls(self, reason):
        """Mengunci seluruh kontrol dan mencatat alasannya."""
        self.control_locked = True
        self.lock_reason = reason
        self.log(f"KONTROL DIKUNCI: {reason}", "red")
        self.set_status("Kontrol dikunci oleh safety interlock")

    def _start_auto_ascent(self):
        """Menghitung sisa baterai vs jarak ke permukaan lalu naik otomatis."""
        depth = self.sensor["kedalaman"]
        battery = self.sensor["baterai"]
        needed = depth * BATT_PER_METER
        self.ascent_rate = clamp(battery / 100 * ASCENT_RATE_MAX,
                                 ASCENT_RATE_MIN, ASCENT_RATE_MAX)
        ticks = int(depth / self.ascent_rate + 0.999)
        self.auto_ascent = True
        self.control_locked = True
        self.override_hazard = False
        self.lock_reason = "Auto ascent aktif (baterai kritis)."
        cukup = "CUKUP" if battery >= needed else "TIDAK CUKUP"
        self.log(f"OTOMATIS NAIK | baterai {battery:.1f} %, kedalaman "
                 f"{depth:.1f} m. Butuh {needed:.1f} % untuk naik, sisa "
                 f"{battery - needed:.1f} % ({cukup}). Laju "
                 f"{self.ascent_rate:.2f} m/tick, ETA {ticks} tick.", "red")
        self.set_status("Auto ascent aktif - kontrol dikunci")

    def unlock_controls(self):
        """Override manual: membuka kunci saat masih ada bahaya."""
        if not self._danger_reasons() or self.auto_ascent:
            return
        self.control_locked = False
        self.override_hazard = True
        self.log("OVERRIDE: kontrol dibuka manual oleh operator meskipun "
                 "kondisi sensor berbahaya.", "amber")
        self.set_status("Kontrol dibuka manual (override)")
        self._refresh_display()

    # -----------------------------------------------------------------
    # 6. SKENARIO UJI (hanya SIMULASI)
    # -----------------------------------------------------------------
    def _sim_only(self):
        """Mengembalikan False dan memberi tahu jika mode bukan SIMULASI."""
        if self.mode == "SIMULASI":
            return True
        messagebox.showinfo("Mode REAL",
                            "Skenario uji hanya tersedia di mode SIMULASI.")
        return False

    def apply_scenario(self):
        """Menjalankan skenario yang dipilih pada dropdown header."""
        if not self._sim_only():
            return
        label = self.scenario_var.get()
        action = self.scenario_actions.get(label)
        if action:
            action()

    def scenario_normal(self):
        """Mengembalikan semua kondisi ke normal dan membuka kunci."""
        if not self._sim_only():
            return
        self.sensor["suhu"] = SENSOR_SPECS["suhu"][2]
        self.sensor["baterai"] = 90.0
        self.sensor["kedalaman"] = 3.0
        self.auto_ascent = False
        self.control_locked = False
        self.override_hazard = False
        self.lock_reason = ""
        for name in THRUSTERS:
            self.motor[name] = {"speed": 0, "dir": "-"}
        self.speed_var.set(0)
        self.log("SKENARIO: kondisi normal (suhu 28 °C, baterai 90 %, "
                 "kedalaman 3 m). Kontrol dibuka.", "info")
        self.set_status("Skenario kondisi normal diterapkan")

    def scenario_low_battery(self):
        """Baterai 12 % di kedalaman 6 m: memicu auto ascent."""
        if not self._sim_only():
            return
        self.sensor["baterai"] = 12.0
        self.sensor["kedalaman"] = 6.0
        self.log("SKENARIO: baterai 12 % di kedalaman 6 m. Auto ascent akan "
                 "dihitung dan dijalankan.", "amber")
        self.set_status("Skenario baterai rendah diterapkan")

    def scenario_hot(self):
        """Memaksa suhu air di atas 45 °C."""
        if not self._sim_only():
            return
        self.sensor["suhu"] = 46.0
        self.log("SKENARIO: suhu air 46 °C (di atas 45 °C).", "amber")
        self.set_status("Skenario suhu tinggi diterapkan")

    def toggle_link(self):
        """Memutus atau menyambung kembali data simulasi."""
        if not self._sim_only():
            return
        self.link_cut = not self.link_cut
        if self.link_cut:
            self.btn_link.configure(text="Sambung Link")
            self.log("SKENARIO: simulasi diputus. Data berhenti.", "amber")
            self.set_status("Link diputus oleh penguji")
        else:
            self.last_data_time = datetime.now()
            self.btn_link.configure(text="Putuskan Link")
            self.log("SKENARIO: simulasi disambung kembali.", "green")
            self.set_status("Link disambung kembali")

    # -----------------------------------------------------------------
    # 7. PEMILIHAN THRUSTER, ARAH, DAN KECEPATAN
    # -----------------------------------------------------------------
    def _select_thruster(self, name):
        """Memilih thruster lalu menyesuaikan opsi arah dan target."""
        self.thruster_var.set(name)
        self._on_thruster_change()

    def _select_direction(self, name):
        """Memilih arah putaran lalu memperbarui warna tombol."""
        self.direction_var.set(name)
        self._refresh_toggles(self.dir_toggles)
        self.set_status(f"Arah {name} dipilih")

    def _on_thruster_change(self):
        """Mengatur ulang target dan arah sesuai thruster yang dipilih."""
        name = self.thruster_var.get()
        options = THRUSTER_DIRECTIONS[name]
        self.speed_var.set(0)
        self.direction_var.set(options[0])
        self._rebuild_directions(options)
        self._refresh_toggles(self.thruster_toggles)
        self.log(f"Thruster dipilih: {name}. Arah {options[0]}/{options[1]}, "
                 f"target 0 %.", "info")
        self.set_status(f"Thruster {name} dipilih")
        self._refresh_display()

    def _step_speed(self, delta):
        """Mengubah target kecepatan dengan langkah tetap, dibatasi 0-100."""
        value = clamp(self.speed_var.get() + delta, 0, SPEED_MAX)
        self.speed_var.set(value)
        self._refresh_display()

    # -----------------------------------------------------------------
    # 5e. TAMPILAN
    # -----------------------------------------------------------------
    def _overall_level(self):
        """Kategori terburuk dari seluruh sensor, atau BAHAYA jika LINK LOST."""
        if not self.link_ok:
            return "BAHAYA"
        worst = "NORMAL"
        for key in SENSOR_SPECS:
            level = categorize(key, self.sensor[key])
            if LEVEL_RANK[level] > LEVEL_RANK[worst]:
                worst = level
        return worst

    def _refresh_display(self):
        """Memperbarui seluruh indikator dari keadaan terkini."""
        self._refresh_sensor_cards()
        self._refresh_header_status()

        now = datetime.now()
        self.lbl_clock.configure(text=now.strftime("%d/%m/%Y  %H:%M:%S"))
        uptime = str(now - self.started_at).split(".")[0]
        age = self._data_age()
        self.lbl_info.configure(
            text=f"Mode {self.mode}  |  Aktif {uptime}  |  "
                 f"Data {age:.1f} s lalu  |  Tick {self.tick_count}")

        self.lbl_target.configure(text=f"Target: {self.speed_var.get()} %")
        self._refresh_motor_view()
        self._refresh_toggles(self.thruster_toggles)
        self._refresh_toggles(self.dir_toggles)
        self._refresh_lock_button()
        state = "normal" if self.link_ok and not self.control_locked \
            else "disabled"
        self.btn_send.configure(state=state)

    def _refresh_sensor_cards(self):
        """Mewarnai kartu sensor sesuai kategori: normal, warning, bahaya."""
        for key, spec in SENSOR_SPECS.items():
            card = self.cards[key]
            level = categorize(key, self.sensor[key])
            tint = CARD_TINTS[level]
            card["frame"].configure(bg=tint,
                                    highlightbackground=LEVEL_COLORS[level])
            card["title"].configure(bg=tint)
            card["value"].configure(bg=tint,
                                    text=f"{self.sensor[key]:.1f} {spec[1]}",
                                    fg=TEXT)
            card["badge"].configure(bg=tint, text=level,
                                    fg=LEVEL_COLORS[level])

        t = self.telemetry
        self.lbl_telemetry.configure(
            text=f"Heading {t['heading']:.0f}°    Pitch {t['pitch']:+.1f}°"
                 f"    Roll {t['roll']:+.1f}°")

    def _refresh_header_status(self):
        """Memperbarui badge sistem dan indikator LINK."""
        overall = self._overall_level()
        self.lbl_system.configure(text=f"SISTEM: {overall}",
                                  bg=LEVEL_COLORS[overall])
        if self.link_ok:
            self.lbl_link.configure(text="LINK OK", fg=LOG_GREEN)
        else:
            self.lbl_link.configure(text="LINK LOST", fg=LOG_RED)

    def _refresh_lock_button(self):
        """Mengatur tombol override sesuai kondisi bahaya dan auto ascent."""
        hazard = bool(self._danger_reasons())
        if self.auto_ascent:
            self.btn_unlock.configure(text="AUTO NAIK AKTIF",
                                      state="disabled")
        elif hazard and self.control_locked:
            self.btn_unlock.configure(text="BUKA KUNCI (OVERRIDE)",
                                      state="normal")
        elif hazard and self.override_hazard:
            self.btn_unlock.configure(text="OVERRIDE AKTIF",
                                      state="disabled")
        else:
            self.btn_unlock.configure(text="TIDAK ADA KENDALA",
                                      state="disabled")

    def _refresh_motor_view(self):
        """Menampilkan kecepatan tiap thruster pada kartu status."""
        for name in THRUSTERS:
            motor = self.motor[name]
            card, title, value, direction = self.motor_cards[name]
            active = motor["speed"] > 0
            tint = MOTOR_ACTIVE_BG if active else CARD
            card.configure(bg=tint, highlightbackground=GREEN if active
                           else BORDER)
            title.configure(bg=tint)
            value.configure(bg=tint, text=f"{motor['speed']} %",
                            fg=GREEN if active else TEXT)
            direction.configure(bg=tint, text=motor["dir"],
                                fg=GREEN if active else MUTED)

        name = self.thruster_var.get()
        if name:
            self.lbl_speed.configure(text=f"{self.motor[name]['speed']} %")
        else:
            self.lbl_speed.configure(text="-")

    # -----------------------------------------------------------------
    # 8. PERINTAH DAN SAFETY INTERLOCK
    # -----------------------------------------------------------------
    @staticmethod
    def _parse(text, name, low, high):
        """Mengubah teks menjadi angka dan memeriksa rentangnya."""
        text = text.strip().replace(",", ".")
        if not text:
            raise ValueError(f"{name} tidak boleh kosong.")
        try:
            value = float(text)
        except ValueError:
            raise ValueError(f"{name} harus berupa angka.") from None
        if not low <= value <= high:
            raise ValueError(f"{name} harus antara {low} dan {high}.")
        return value

    def _snapshot(self):
        """Ringkasan pembacaan sensor untuk log."""
        return (f"suhu={self.sensor['suhu']:.1f} °C, "
                f"baterai={self.sensor['baterai']:.1f} %")

    def _danger_reasons(self):
        """Alasan sensor berada pada kategori BAHAYA."""
        reasons = []
        if categorize("suhu", self.sensor["suhu"]) == "BAHAYA":
            reasons.append(f"Suhu {self.sensor['suhu']:.1f} °C di atas "
                           f"{TEMP_MAX:.0f} °C.")
        if categorize("baterai", self.sensor["baterai"]) == "BAHAYA":
            reasons.append(f"Sisa baterai {self.sensor['baterai']:.1f} % "
                           f"di bawah {BATT_MIN:.0f} %.")
        return reasons

    def _block(self, message):
        """Memblokir perintah, menampilkan peringatan, dan mencatat ke log."""
        messagebox.showwarning("Perintah Diblokir", message)
        self.log(f"DIBLOKIR: {message}", "amber")
        self.set_status("Perintah diblokir oleh safety interlock")

    def _write_cmd(self, text):
        """Menambahkan perintah ke file perintah (mode REAL)."""
        stamp = datetime.now().isoformat(timespec="seconds")
        try:
            with open(CMD_FILE, "a", encoding="utf-8") as handle:
                handle.write(f"{stamp} {text}\n")
        except OSError as err:
            self.log(f"Gagal menulis {CMD_FILE}: {err}", "red")

    def _check_interlock(self, name, current, target):
        """Mengembalikan pesan blokir, atau None jika perintah aman."""
        if not self.link_ok:
            return "Data telemetri terputus (LINK LOST). Kontrol dikunci."
        if self.auto_ascent:
            return ("Auto ascent aktif. Kontrol dikunci sampai ROV mencapai "
                    "permukaan.")
        if self.control_locked:
            return (f"Kontrol terkunci: {self.lock_reason} Tekan BUKA KUNCI "
                    f"untuk override manual.")
        if not name:
            return "Pilih thruster terlebih dahulu sebelum mengirim perintah."
        if current == 0 and target == SPEED_MAX:
            return ("Kecepatan langsung dari 0 % ke 100 % diblokir untuk "
                    "mencegah lonjakan arus. Naikkan secara bertahap.")
        if current == SPEED_MAX and target == 0:
            return ("Penurunan langsung dari 100 % ke 0 % diblokir. "
                    "Turunkan secara bertahap atau gunakan KILL SWITCH.")
        if target > 0:
            reasons = self._danger_reasons()
            if reasons:
                return " ".join(reasons) + " Motor tidak boleh menyala."
        return None

    def send_command(self):
        """Menjalankan safety interlock lalu mengirim perintah ke thruster."""
        name = self.thruster_var.get()
        target = self.speed_var.get()
        direction = self.direction_var.get()
        current = self.motor[name]["speed"] if name else 0

        problem = self._check_interlock(name, current, target)
        if problem:
            self._block(problem)
            return

        self.motor[name] = {"speed": target, "dir": direction}
        status = "AKTIF" if target > 0 else "BERHENTI"
        thruster = f"Thruster {name}"
        self.log(f"PERINTAH OK | thruster={thruster}, status={status}, "
                 f"kecepatan={target} %, arah={direction}, "
                 f"{self._snapshot()}", "green")
        if self.mode == "REAL":
            self._write_cmd(f"MOTOR {thruster} {target} {direction}")
        self._refresh_display()
        self.set_status(f"Perintah dikirim ke {thruster}")

    def _set_command(self, text):
        """Mengisi kolom perintah tanpa mengirimkannya."""
        self.cmd_entry.delete(0, "end")
        self.cmd_entry.insert(0, text)
        self.set_status(f"Perintah disiapkan: {text}")

    def fill_motor_command(self):
        """Menyusun perintah motor dari pilihan thruster, kecepatan, arah."""
        name = self.thruster_var.get()
        if not name:
            messagebox.showwarning("Belum Ada Thruster",
                                   "Pilih thruster terlebih dahulu.")
            return
        text = (f"MOTOR {name} {self.speed_var.get()} "
                f"{self.direction_var.get()}")
        self._set_command(text)

    # -----------------------------------------------------------------
    # 9. DARURAT
    # -----------------------------------------------------------------
    def kill_switch(self):
        """Menghentikan paksa semua thruster dan mencatat peringatan merah."""
        for name in THRUSTERS:
            self.motor[name] = {"speed": 0, "dir": "-"}
        self.auto_ascent = False
        self.speed_var.set(0)
        self.log("!!! SISTEM DIHENTIKAN PAKSA oleh KILL SWITCH | "
                 "semua thruster = 0 %, status=DIHENTIKAN, "
                 f"{self._snapshot()}", "red")
        if self.mode == "REAL":
            self._write_cmd("STOP")
        self._refresh_display()
        self.set_status("KILL SWITCH aktif - sistem dihentikan paksa")

    def send_raw(self):
        """Mengirim perintah teks bebas dari kolom log dan mencatatnya."""
        text = self.cmd_entry.get().strip()
        if not text:
            messagebox.showwarning("Perintah Kosong",
                                   "Ketik perintah terlebih dahulu.")
            return
        if not self.link_ok:
            self._block("Data telemetri terputus (LINK LOST). "
                        "Perintah tidak dikirim.")
            return
        if self.control_locked and text.upper() != "STOP":
            self._block("Kontrol terkunci. Hanya perintah STOP yang "
                        "diizinkan selama kondisi bahaya.")
            return
        self.log(f"TX: {text}", "info")
        if self.mode == "REAL":
            self._write_cmd(text)
            self.log("RX: menunggu balasan dari ROV", "info")
        else:
            self.log("RX: ACK (simulasi)", "green")
        self.cmd_entry.delete(0, "end")
        self.set_status(f"Perintah terkirim: {text}")

    # -----------------------------------------------------------------
    # 10. PENUTUPAN
    # -----------------------------------------------------------------
    def _on_close_request(self):
        """Tombol X di jendela diabaikan; penutupan hanya lewat tombol."""
        self.log("Penutupan lewat tombol X diabaikan. Gunakan TUTUP APLIKASI.",
                 "amber")
        self.set_status("Gunakan tombol TUTUP APLIKASI untuk keluar")

    def close_app(self):
        """Menutup aplikasi setelah konfirmasi dari operator."""
        if not messagebox.askyesno("Tutup Aplikasi",
                                   "Tutup GCS ROV sekarang?"):
            return
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
