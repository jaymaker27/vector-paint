#!/usr/bin/env python3
# VPP bench GUI (Tkinter) — full-screen, debounced inputs, smooth jog.
# Uses RPi.GPIO bit-banging; STEP idles HIGH (active-LOW pulse).
import sys, time, threading
from tkinter import *
from tkinter import ttk, messagebox
import RPi.GPIO as GPIO

# -------- GPIO setup --------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pins (BCM)
STEP_X, DIR_X = 23, 24
STEP_Y, DIR_Y = 20, 21
TRIGGER_PIN   = 18           # fire output (SAFE_MODE blocks)
ESTOP_PIN     = 25           # E-STOP: NO→GND (active-LOW)
LIM_X_MIN     = 17           # X limit: NC→GND (opens=1)
LIM_Y_MIN     = 22           # Y limit: NC→GND (opens=1)

SAFE_MODE      = True
MICROSTEP      = 16
STEPS_PER_REV  = 200 * MICROSTEP
DIR_INV_X      = False
DIR_INV_Y      = False

# Outputs: STEP idle HIGH (active-LOW), DIR LOW, trigger LOW
for p in (STEP_X, STEP_Y):
    GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)
for p in (DIR_X, DIR_Y, TRIGGER_PIN):
    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

# Inputs with pull-ups
for p in (ESTOP_PIN, LIM_X_MIN, LIM_Y_MIN):
    GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# -------- shared state (set by poller, read by motion threads) --------
_stop = threading.Event()
_motion_lock = threading.Lock()
estop_flag = False
xlim_flag  = False
ylim_flag  = False

# -------- helpers --------
def _step_pulse(step_pin: int, halfT: float):
    GPIO.output(step_pin, GPIO.LOW);  time.sleep(halfT)
    GPIO.output(step_pin, GPIO.HIGH); time.sleep(halfT)

def _dir_write(dir_pin: int, forward: bool, invert=False):
    GPIO.output(dir_pin, GPIO.LOW if (forward ^ invert) else GPIO.HIGH)

def read_debounced(pin: int, samples=12, interval_s=0.002) -> int:
    ones = 0
    for _ in range(samples):
        ones += GPIO.input(pin)
        time.sleep(interval_s)
    return 1 if ones > samples//2 else 0

def fire_once(pulse_s=0.1):
    if SAFE_MODE:
        print("[SAFE_MODE] FIRE suppressed")
        return
    GPIO.output(TRIGGER_PIN, GPIO.HIGH); time.sleep(pulse_s); GPIO.output(TRIGGER_PIN, GPIO.LOW)

def park_step_lines():
    for p in (STEP_X, STEP_Y):
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def cleanup():
    try: _stop.set()
    except: pass
    park_step_lines()
    GPIO.output(TRIGGER_PIN, GPIO.LOW)
    GPIO.cleanup()

# -------- motion (now checks fast flags; no sleeps inside safety checks) --------
DIR_SETTLE_S = 0.002

def jog_axis(step_pin, dir_pin, forward, freq_hz, invert_dir, limit_flag_name):
    if freq_hz < 1: freq_hz = 1
    halfT = 0.5/float(freq_hz)
    with _motion_lock:
        _stop.clear()
        _dir_write(dir_pin, forward, invert=invert_dir)
        time.sleep(DIR_SETTLE_S)
        while not _stop.is_set():
            if estop_flag: break
            if (limit_flag_name == "x" and xlim_flag) or (limit_flag_name == "y" and ylim_flag):
                break
            _step_pulse(step_pin, halfT)

def move_degrees(step_pin, dir_pin, forward, degrees, freq_hz, invert_dir, limit_flag_name):
    steps = int(abs(degrees) * STEPS_PER_REV / 360.0)
    if steps <= 0: return
    if freq_hz < 1: freq_hz = 1
    halfT = 0.5/float(freq_hz)
    with _motion_lock:
        _stop.clear()
        _dir_write(dir_pin, forward, invert=invert_dir)
        time.sleep(DIR_SETTLE_S)
        for _ in range(steps):
            if _stop.is_set() or estop_flag: break
            if (limit_flag_name == "x" and xlim_flag) or (limit_flag_name == "y" and ylim_flag):
                break
            _step_pulse(step_pin, halfT)

# -------- UI --------
root = Tk()
root.title("VPP • Bench UI")
root.attributes('-fullscreen', True)
root.configure(bg="#0b0f12")
root.bind("<F11>", lambda e: root.attributes('-fullscreen', not root.attributes('-fullscreen')))
def _esc(_): on_close()
root.bind("<Escape>", _esc)

s = ttk.Style()
s.theme_use("clam")
s.configure("TButton", padding=8, font=("Segoe UI", 13))
s.configure("HL.TButton", padding=12, font=("Segoe UI", 15, "bold"))
s.configure("TLabel", background="#0b0f12", foreground="#e6e6e6", font=("Segoe UI", 12))
s.configure("LED.Good.TLabel", background="#1b2a19", foreground="#87f987")
s.configure("LED.Bad.TLabel",  background="#2a1919", foreground="#ff7b7b")

frame_top = ttk.Frame(root); frame_top.pack(pady=10)
lbl_estop = ttk.Label(frame_top, text="E-STOP: —", width=20)
lbl_limx  = ttk.Label(frame_top, text="X LIMIT: —", width=16)
lbl_limy  = ttk.Label(frame_top, text="Y LIMIT: —", width=16)
lbl_estop.grid(row=0, column=0, padx=10); lbl_limx.grid(row=0, column=1, padx=10); lbl_limy.grid(row=0, column=2, padx=10)

frame_cfg = ttk.Frame(root); frame_cfg.pack(pady=6)
ttk.Label(frame_cfg, text="Speed (Hz):").grid(row=0, column=0, sticky=E, padx=6)
spd = IntVar(value=800)  # start a bit gentler; bump up once smooth
spd_scale = ttk.Scale(frame_cfg, from_=50, to=1500, orient=HORIZONTAL, length=350,
                      command=lambda v: spd.set(int(float(v))))
spd_scale.set(spd.get()); spd_scale.grid(row=0, column=1, padx=8)
ttk.Label(frame_cfg, textvariable=spd, width=6).grid(row=0, column=2, sticky=W)

ttk.Label(frame_cfg, text="Move (deg):").grid(row=1, column=0, sticky=E, padx=6)
deg_str = StringVar(value="45")
ttk.Entry(frame_cfg, textvariable=deg_str, width=10).grid(row=1, column=1, sticky=W, padx=8)

frame_jog = ttk.Frame(root); frame_jog.pack(pady=16)

def _stop_jog(_=None): _stop.set()

def _start_jog_x(forward):
    if estop_flag: messagebox.showwarning("E-STOP", "E-STOP is active."); return
    threading.Thread(target=jog_axis,
                     args=(STEP_X, DIR_X, forward, spd.get(), DIR_INV_X, "x"),
                     daemon=True).start()

def _start_jog_y(forward):
    if estop_flag: messagebox.showwarning("E-STOP", "E-STOP is active."); return
    threading.Thread(target=jog_axis,
                     args=(STEP_Y, DIR_Y, forward, spd.get(), DIR_INV_Y, "y"),
                     daemon=True).start()

btn_xm = ttk.Button(frame_jog, text="X −", style="HL.TButton")
btn_xp = ttk.Button(frame_jog, text="X +", style="HL.TButton")
btn_ym = ttk.Button(frame_jog, text="Y −", style="HL.TButton")
btn_yp = ttk.Button(frame_jog, text="Y +", style="HL.TButton")
btn_xm.grid(row=0, column=0, padx=14); btn_xp.grid(row=0, column=1, padx=14)
btn_ym.grid(row=1, column=0, padx=14, pady=12); btn_yp.grid(row=1, column=1, padx=14, pady=12)

# press-and-hold behavior
btn_xm.bind("<ButtonPress-1>", lambda e: _start_jog_x(False)); btn_xm.bind("<ButtonRelease-1>", _stop_jog)
btn_xp.bind("<ButtonPress-1>", lambda e: _start_jog_x(True));  btn_xp.bind("<ButtonRelease-1>", _stop_jog)
btn_ym.bind("<ButtonPress-1>", lambda e: _start_jog_y(False)); btn_ym.bind("<ButtonRelease-1>", _stop_jog)
btn_yp.bind("<ButtonPress-1>", lambda e: _start_jog_y(True));  btn_yp.bind("<ButtonRelease-1>", _stop_jog)

frame_move = ttk.Frame(root); frame_move.pack(pady=8)

def _move_x(sign):
    if estop_flag: messagebox.showwarning("E-STOP", "E-STOP is active."); return
    try: deg = float(deg_str.get()) * sign
    except: messagebox.showerror("Value", "Enter a number for degrees"); return
    threading.Thread(target=move_degrees,
                     args=(STEP_X, DIR_X, deg>=0, abs(deg), spd.get(), DIR_INV_X, "x"),
                     daemon=True).start()

def _move_y(sign):
    if estop_flag: messagebox.showwarning("E-STOP", "E-STOP is active."); return
    try: deg = float(deg_str.get()) * sign
    except: messagebox.showerror("Value", "Enter a number for degrees"); return
    threading.Thread(target=move_degrees,
                     args=(STEP_Y, DIR_Y, deg>=0, abs(deg), spd.get(), DIR_INV_Y, "y"),
                     daemon=True).start()

ttk.Button(frame_move, text="Move X −deg", command=lambda: _move_x(-1)).grid(row=0, column=0, padx=8)
ttk.Button(frame_move, text="Move X +deg", command=lambda: _move_x(+1)).grid(row=0, column=1, padx=8)
ttk.Button(frame_move, text="Move Y −deg", command=lambda: _move_y(-1)).grid(row=1, column=0, padx=8, pady=4)
ttk.Button(frame_move, text="Move Y +deg", command=lambda: _move_y(+1)).grid(row=1, column=1, padx=8, pady=4)

frame_fire = ttk.Frame(root); frame_fire.pack(pady=12)
def _do_fire():
    if SAFE_MODE:
        messagebox.showinfo("FIRE", "SAFE_MODE is ON — not pulsing trigger.")
    fire_once(0.1)
ttk.Button(frame_fire, text="FIRE", command=_do_fire).pack()

# -------- status poller (debounced) --------
def poll_inputs():
    global estop_flag, xlim_flag, ylim_flag
    estop_flag = (read_debounced(ESTOP_PIN, 12, 0.002) == 0)  # active-LOW
    xlim_flag  = (read_debounced(LIM_X_MIN,  8, 0.002) == 1)  # NC open = 1
    ylim_flag  = (read_debounced(LIM_Y_MIN,  8, 0.002) == 1)

    lbl_estop.configure(text=f"E-STOP: {'PRESSED' if estop_flag else 'OK'}",
                        style="LED.Bad.TLabel" if estop_flag else "LED.Good.TLabel")
    lbl_limx.configure(text=f"X LIMIT: {'TRIPPED' if xlim_flag else 'OK'}",
                       style="LED.Bad.TLabel" if xlim_flag else "LED.Good.TLabel")
    lbl_limy.configure(text=f"Y LIMIT: {'TRIPPED' if ylim_flag else 'OK'}",
                       style="LED.Bad.TLabel" if ylim_flag else "LED.Good.TLabel")

    if estop_flag or xlim_flag or ylim_flag:
        _stop.set()

    root.after(100, poll_inputs)  # ~10 Hz

def on_close():
    try:
        if messagebox.askokcancel("Quit", "Quit and park outputs?"):
            cleanup(); root.destroy()
    except Exception:
        cleanup(); sys.exit(0)

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(150, poll_inputs)

if __name__ == "__main__":
    try:
        root.mainloop()
    finally:
        cleanup()
