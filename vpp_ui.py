#!/usr/bin/env python3
# vpp_ui.py — simple bench GUI for the VPP turret (Pi + DM556D drivers)
# - Fullscreen on 7" Pi display (F11 toggles windowed)
# - Hold-to-jog X/Y with speed slider (Hz)
# - Move by degrees (+/−)
# - FIRE button (honors SAFE_MODE)
# - Live, debounced indicators for E-STOP (active-LOW) and X/Y limit (NC)
# - STEP lines are parked HIGH on exit so nothing "free-runs"

import sys, time, threading
from tkinter import *
from tkinter import ttk, messagebox

# ---------------- GPIO ----------------
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# === Pin map (BCM) — matches your wiring ===
STEP_X, DIR_X = 23, 24      # Motor X (Pan)
STEP_Y, DIR_Y = 20, 21      # Motor Y (Tilt)
TRIGGER_PIN    = 18         # Fire output (relay/MOSFET)
ESTOP_PIN      = 25         # E-STOP NO → GND (active-LOW)
LIM_X_MIN      = 17         # X limit (NC → GND, opens when hit) 
LIM_Y_MIN      = 22         # Y limit (NC → GND, opens when hit)

# === Motion config (bench) ===
SAFE_MODE      = True       # leave True until fully safe to fire
MICROSTEP      = 16         # DM556D SW5-8 → 3200 pulses/rev
STEPS_PER_REV  = 200 * MICROSTEP
DIR_INV_X      = False      # flip if needed
DIR_INV_Y      = False

# Outputs idle: STEP idles HIGH (active-LOW pulse), DIR LOW, TRIGGER LOW
for p in (STEP_X, STEP_Y):
    GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)
for p in (DIR_X, DIR_Y, TRIGGER_PIN):
    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

# Inputs with pull-ups (switches to GND)
for p in (ESTOP_PIN, LIM_X_MIN, LIM_Y_MIN):
    GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- Helpers ----------------
_stop = threading.Event()
_motion_lock = threading.Lock()

def _step_pulse(step_pin: int, half_period_s: float):
    GPIO.output(step_pin, GPIO.LOW)    # active-LOW pulse (common-anode style)
    time.sleep(half_period_s)
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(half_period_s)

def _dir_write(dir_pin: int, forward: bool, invert: bool=False):
    # Forward = LOW unless inverted (keeps electrical polarity explicit)
    val = GPIO.LOW if (forward ^ invert) else GPIO.HIGH
    GPIO.output(dir_pin, val)

def read_debounced(pin: int, samples=12, interval_s=0.002) -> int:
    """Majority-vote debounce (~24 ms default). Returns 0 or 1."""
    ones = 0
    for _ in range(samples):
        ones += GPIO.input(pin)
        time.sleep(interval_s)
    return 1 if ones > samples//2 else 0

def estop_active() -> bool:
    # E-STOP wired NO→GND, pulled-up; pressed = LOW
    v = read_debounced(ESTOP_PIN, samples=12, interval_s=0.002)
    return (v == 0)

def limit_tripped(pin: int) -> bool:
    # NC → GND; open = 1 (tripped)
    return read_debounced(pin, samples=8, interval_s=0.002) == 1

def fire_once(pulse_s=0.1):
    if SAFE_MODE:
        print("[SAFE_MODE] FIRE suppressed")
        return
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(pulse_s)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)

def park_step_lines():
    # Park STEP as inputs with pull-ups to prevent free-run if any process exits
    for p in (STEP_X, STEP_Y):
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def cleanup():
    try:
        _stop.set()
    except:
        pass
    park_step_lines()
    GPIO.output(TRIGGER_PIN, GPIO.LOW)
    GPIO.cleanup()

# ---------------- Motion primitives ----------------
def jog_axis(step_pin: int, dir_pin: int, forward: bool, freq_hz: int, invert_dir: bool, limit_pin: int):
    """Press-and-hold jog; stops on E-STOP, limit, or stop event."""
    if freq_hz < 1: freq_hz = 1
    halfT = 0.5 / float(freq_hz)
    with _motion_lock:
        _stop.clear()
        _dir_write(dir_pin, forward, invert=invert_dir)
        time.sleep(0.002)  # small settle after DIR change
        while not _stop.is_set():
            if estop_active(): break
            if limit_tripped(limit_pin): break
            _step_pulse(step_pin, halfT)

def move_degrees(step_pin: int, dir_pin: int, forward: bool, degrees: float, freq_hz: int, invert_dir: bool, limit_pin: int):
    steps = int(abs(degrees) * STEPS_PER_REV / 360.0)
    if steps <= 0: return
    if freq_hz < 1: freq_hz = 1
    halfT = 0.5 / float(freq_hz)
    with _motion_lock:
        _stop.clear()
        _dir_write(dir_pin, forward, invert=invert_dir)
        time.sleep(0.002)
        for _ in range(steps):
            if _stop.is_set() or estop_active() or limit_tripped(limit_pin):
                break
            _step_pulse(step_pin, halfT)

# ---------------- UI ----------------
root = Tk()
root.title("VPP • Bench UI")
# Fullscreen on 7" Pi display; F11 toggles windowed, ESC prompts exit
root.attributes('-fullscreen', True)
root.bind("<F11>", lambda e: root.attributes('-fullscreen', not root.attributes('-fullscreen')))
def _esc(e): on_close()
root.bind("<Escape>", _esc)

root.configure(bg="#0b0f12")
style = ttk.Style()
style.theme_use("clam")
style.configure("TButton", padding=8, font=("Segoe UI", 13))
style.configure("HL.TButton", padding=12, font=("Segoe UI", 15, "bold"))
style.configure("TLabel", background="#0b0f12", foreground="#e6e6e6", font=("Segoe UI", 12))
style.configure("LED.Good.TLabel", background="#1b2a19", foreground="#87f987")
style.configure("LED.Bad.TLabel",  background="#2a1919", foreground="#ff7b7b")

# Indicators row
frame_top = ttk.Frame(root)
frame_top.pack(pady=10)
lbl_estop = ttk.Label(frame_top, text="E-STOP: —", width=20)
lbl_limx  = ttk.Label(frame_top, text="X LIMIT: —", width=16)
lbl_limy  = ttk.Label(frame_top, text="Y LIMIT: —", width=16)
lbl_estop.grid(row=0, column=0, padx=10)
lbl_limx.grid(row=0, column=1, padx=10)
lbl_limy.grid(row=0, column=2, padx=10)

# Speed + degrees
frame_cfg = ttk.Frame(root); frame_cfg.pack(pady=6)
ttk.Label(frame_cfg, text="Speed (Hz):").grid(row=0, column=0, sticky=E, padx=6)
spd = IntVar(value=1000)  # you found 1000 Hz smooth
spd_scale = ttk.Scale(frame_cfg, from_=50, to=1500, orient=HORIZONTAL, length=350,
                      command=lambda v: spd.set(int(float(v))))
spd_scale.set(spd.get()); spd_scale.grid(row=0, column=1, padx=8)
ttk.Label(frame_cfg, textvariable=spd, width=6).grid(row=0, column=2, sticky=W)

ttk.Label(frame_cfg, text="Move (deg):").grid(row=1, column=0, sticky=E, padx=6)
deg_str = StringVar(value="45")
ttk.Entry(frame_cfg, textvariable=deg_str, width=10).grid(row=1, column=1, sticky=W, padx=8)

# Jog buttons
frame_jog = ttk.Frame(root); frame_jog.pack(pady=16)

def _start_jog_x(forward: bool):
    if estop_active(): messagebox.showwarning("E-STOP", "E-STOP is active."); return
    threading.Thread(target=jog_axis,
                     args=(STEP_X, DIR_X, forward, spd.get(), DIR_INV_X, LIM_X_MIN),
                     daemon=True).start()

def _start_jog_y(forward: bool):
    if estop_active(): messagebox.showwarning("E-STOP", "E-STOP is active."); return
    threading.Thread(target=jog_axis,
                     args=(STEP_Y, DIR_Y, forward, spd.get(), DIR_INV_Y, LIM_Y_MIN),
                     daemon=True).start()

def _stop_jog(_=None): _stop.set()

btn_xm = ttk.Button(frame_jog, text="X −", style="HL.TButton")
btn_xp = ttk.Button(frame_jog, text="X +", style="HL.TButton")
btn_ym = ttk.Button(frame_jog, text="Y −", style="HL.TButton")
btn_yp = ttk.Button(frame_jog, text="Y +", style="HL.TButton")

btn_xm.grid(row=0, column=0, padx=14)
btn_xp.grid(row=0, column=1, padx=14)
btn_ym.grid(row=1, column=0, padx=14, pady=12)
btn_yp.grid(row=1, column=1, padx=14, pady=12)

# press-and-hold behavior
btn_xm.bind("<ButtonPress-1>", lambda e: _start_jog_x(False))
btn_xm.bind("<ButtonRelease-1>", _stop_jog)
btn_xp.bind("<ButtonPress-1>", lambda e: _start_jog_x(True))
btn_xp.bind("<ButtonRelease-1>", _stop_jog)
btn_ym.bind("<ButtonPress-1>", lambda e: _start_jog_y(False))
btn_ym.bind("<ButtonRelease-1>", _stop_jog)
btn_yp.bind("<ButtonPress-1>", lambda e: _start_jog_y(True))
btn_yp.bind("<ButtonRelease-1>", _stop_jog)

# Move by degrees
frame_move = ttk.Frame(root); frame_move.pack(pady=8)

def _move_x(sign: int):
    if estop_active(): messagebox.showwarning("E-STOP", "E-STOP is active."); return
    try:
        deg = float(deg_str.get()) * sign
    except:
        messagebox.showerror("Value", "Enter a number for degrees"); return
    forward = (deg >= 0)
    threading.Thread(target=move_degrees,
                     args=(STEP_X, DIR_X, forward, abs(deg), spd.get(), DIR_INV_X, LIM_X_MIN),
                     daemon=True).start()

def _move_y(sign: int):
    if estop_active(): messagebox.showwarning("E-STOP", "E-STOP is active."); return
    try:
        deg = float(deg_str.get()) * sign
    except:
        messagebox.showerror("Value", "Enter a number for degrees"); return
    forward = (deg >= 0)
    threading.Thread(target=move_degrees,
                     args=(STEP_Y, DIR_Y, forward, abs(deg), spd.get(), DIR_INV_Y, LIM_Y_MIN),
                     daemon=True).start()

ttk.Button(frame_move, text="Move X −deg", command=lambda: _move_x(-1)).grid(row=0, column=0, padx=8)
ttk.Button(frame_move, text="Move X +deg", command=lambda: _move_x(+1)).grid(row=0, column=1, padx=8)
ttk.Button(frame_move, text="Move Y −deg", command=lambda: _move_y(-1)).grid(row=1, column=0, padx=8, pady=4)
ttk.Button(frame_move, text="Move Y +deg", command=lambda: _move_y(+1)).grid(row=1, column=1, padx=8, pady=4)

# FIRE controls
frame_fire = ttk.Frame(root); frame_fire.pack(pady=12)
def _do_fire():
    if SAFE_MODE:
        messagebox.showinfo("FIRE", "SAFE_MODE is ON — not pulsing trigger.")
    fire_once(0.1)
ttk.Button(frame_fire, text="FIRE", command=_do_fire).pack()

# Status poller (debounced) + safety interlock
def poll_inputs():
    e_low = estop_active()
    x_lim = limit_tripped(LIM_X_MIN)
    y_lim = limit_tripped(LIM_Y_MIN)

    lbl_estop.configure(text=f"E-STOP: {'PRESSED' if e_low else 'OK'}",
                        style="LED.Bad.TLabel" if e_low else "LED.Good.TLabel")
    lbl_limx.configure(text=f"X LIMIT: {'TRIPPED' if x_lim else 'OK'}",
                       style="LED.Bad.TLabel" if x_lim else "LED.Good.TLabel")
    lbl_limy.configure(text=f"Y LIMIT: {'TRIPPED' if y_lim else 'OK'}",
                       style="LED.Bad.TLabel" if y_lim else "LED.Good.TLabel")

    # If anything unsafe, stop jogging immediately
    if e_low or x_lim or y_lim:
        _stop.set()

    root.after(100, poll_inputs)  # ~10 Hz

def on_close():
    try:
        if messagebox.askokcancel("Quit", "Quit and park outputs?"):
            cleanup()
            root.destroy()
    except Exception:
        cleanup()
        sys.exit(0)

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(150, poll_inputs)

if __name__ == "__main__":
    try:
        root.mainloop()
    finally:
        cleanup()
