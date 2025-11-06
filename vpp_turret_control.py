#!/usr/bin/env python3
"""
vpp_turret_control.py

Clean, hardware-accurate controller for the 2-axis paint turret.
- pigpio if available (better timing), else RPi.GPIO fallback
- Active-LOW step pulses (PUL+ = +5 V, we toggle PUL−)
- One NC limit per axis (home/MIN). Optional MAX reserved but not required.
- E-STOP: hardware cuts 24 V; software latches on GPIO25 (NO->GND, pull-up)

WIRING (BCM numbering):
  X axis (Motor 1 / Pan):  STEP=23, DIR=24
  Y axis (Motor 2 / Tilt): STEP=20, DIR=21
  Limits (NC->GND):        LIM_X_MIN=17, LIM_Y_MIN=22  (reads LOW normally, HIGH when tripped/broken)
  E-STOP (NO->GND):        ESTOP=25 (pull-up; LOW when pressed)
  FIRE relay/indicator:    FIRE=18 (active-HIGH)

Driver DIP (both DM556D): SW1..SW8 = OFF,OFF,ON,OFF,  OFF,OFF,ON,ON
 => ~2.7 A RMS, 50% hold, microstep ×16 (3200 steps/rev)
Mechanics: 3:1 gear ratio -> steps/deg = 200*16*3/360 = 26.6666667 (80/3)

Author’s notes:
- Keep Pi on its own 5 V PSU. Logic 5 V (for PUL+/DIR+ and ’125 VCC) can be a small buck.
- Star-ground Pi GND, ’125 GND, logic 5 V negative together. 24 V negative may be isolated; if bonded, do it once at the star.
"""

import time, sys, csv, threading
from math import copysign
from pathlib import Path

# ----------------- TRY PIGPIO -----------------
PIGPIO = False
try:
    import pigpio
    pi = pigpio.pi()
    if pi is not None and pi.connected:
        PIGPIO = True
except Exception:
    PIGPIO = False

if not PIGPIO:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

# ----------------- CONFIG -----------------
# Pins (BCM)
STEP_X, DIR_X = 23, 24
STEP_Y, DIR_Y = 20, 21

LIM_X_MIN = 17  # NC to GND, pull-up -> LOW normal, HIGH when tripped/broken
LIM_Y_MIN = 22

# Optional MAX pins (not wired for now; set to None)
LIM_X_MAX = None
LIM_Y_MAX = None

ESTOP = 25      # NO->GND, pull-up -> HIGH normal, LOW when pressed
FIRE  = 18      # active-HIGH relay/SSR

# Motion constants
FULL_STEPS = 200
MICROSTEP  = 16
GEAR_RATIO = 1.0
STEPS_PER_DEG = (FULL_STEPS * MICROSTEP * GEAR_RATIO) / 360.0   # 80/3 = 26.666...

# Pulse timing (conservative to start)
STEP_ON_US     = 12         # LOW time for active-LOW pulse
STEP_PERIOD_US = 1000       # total period ~1 kHz while testing
DIR_SETUP_US   = 20
DIR_HOLD_US    = 20

# Soft limits (deg), relative to the homed (zero) position
X_MIN_DEG, X_MAX_DEG = 0.0, 120.0
Y_MIN_DEG, Y_MAX_DEG = 0.0, 120.0

# Modes
SAFE_MODE = True  # if False, FIRE will pulse; keep True until fully verified
VERBOSE   = True

# Optional enable logic (not wired now)
USE_ENABLE_PINS = False
EN_X_PIN = None
EN_Y_PIN = None
ENABLE_ACTIVE_LEVEL = 0  # DM556 often active-LOW ENA−; ignored since not used

# ----------------- GPIO ABSTRACTION -----------------
def _setup_out(pin, idle_high=False):
    if pin is None: return
    if PIGPIO:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 1 if idle_high else 0)
    else:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH if idle_high else GPIO.LOW)

def _setup_in_pullup(pin):
    if pin is None: return
    if PIGPIO:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
    else:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def _w(pin, val):
    if pin is None: return
    if PIGPIO: pi.write(pin, 1 if val else 0)
    else:      GPIO.output(pin, GPIO.HIGH if val else GPIO.LOW)

def _r(pin):
    if pin is None: return 1
    if PIGPIO: return pi.read(pin)
    else:      return GPIO.input(pin)

# ----------------- INIT IO -----------------
# Outputs: STEP idle HIGH (active-LOW pulses), DIR idle LOW, FIRE off
_setup_out(STEP_X, idle_high=True)
_setup_out(DIR_X,  idle_high=False)
_setup_out(STEP_Y, idle_high=True)
_setup_out(DIR_Y,  idle_high=False)
_setup_out(FIRE,   idle_high=False)

# Inputs
_setup_in_pullup(ESTOP)
_setup_in_pullup(LIM_X_MIN)
_setup_in_pullup(LIM_Y_MIN)
if LIM_X_MAX is not None: _setup_in_pullup(LIM_X_MAX)
if LIM_Y_MAX is not None: _setup_in_pullup(LIM_Y_MAX)

# Optional EN pins (not used, but stubs kept)
if USE_ENABLE_PINS and EN_X_PIN is not None: _setup_out(EN_X_PIN, idle_high=(1-ENABLE_ACTIVE_LEVEL))
if USE_ENABLE_PINS and EN_Y_PIN is not None: _setup_out(EN_Y_PIN, idle_high=(1-ENABLE_ACTIVE_LEVEL))

# ----------------- STATE -----------------
estop_latched = False
cur_x_steps = 0
cur_y_steps = 0

# ----------------- HELPERS -----------------
def log(*a):
    if VERBOSE: print(*a)

def estop_pressed_now():
    # NO->GND with pull-up => pressed == LOW
    return _r(ESTOP) == 0

def limits_nc_tripped(pin):
    # NC->GND with pull-up => normal LOW, tripped/broken == HIGH
    return _r(pin) == 1

def enable_axis(enable: bool):
    """No-op unless EN pins are later wired."""
    if not USE_ENABLE_PINS: return
    level = ENABLE_ACTIVE_LEVEL if enable else 1-ENABLE_ACTIVE_LEVEL
    if EN_X_PIN is not None: _w(EN_X_PIN, level)
    if EN_Y_PIN is not None: _w(EN_Y_PIN, level)

def fire_once(seconds=0.1):
    if SAFE_MODE:
        log("[SAFE_MODE] would FIRE for", seconds, "s")
        return
    _w(FIRE, 1); time.sleep(seconds); _w(FIRE, 0)

def _sleep_us(us):
    time.sleep(us / 1_000_000.0)

def step_pulse(pin_step, on_us=STEP_ON_US, period_us=STEP_PERIOD_US):
    """Active-LOW pulse: idle HIGH, LOW for on_us, HIGH for the rest of the period."""
    _w(pin_step, 0)                 # LOW = on (optos conduct)
    _sleep_us(on_us)
    _w(pin_step, 1)
    remain = max(0, period_us - on_us)
    _sleep_us(remain)

def set_dir(pin_dir, forward=True, invert=False):
    level = 1 if (forward ^ invert) else 0
    _w(pin_dir, level)
    _sleep_us(DIR_SETUP_US)

def emit_steps(pin_step, n_steps, on_us=STEP_ON_US, period_us=STEP_PERIOD_US):
    for _ in range(n_steps):
        if estop_latched or estop_pressed_now():
            return False
        step_pulse(pin_step, on_us, period_us)
    _sleep_us(DIR_HOLD_US)
    return True

def deg_to_steps(deg):
    return int(round(deg * STEPS_PER_DEG))

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ----------------- SAFETY -----------------
def estop_latch_if_pressed():
    global estop_latched
    if estop_pressed_now():
        estop_latched = True
        _w(FIRE, 0)
        log(">>> E-STOP: latched (power to drivers should be OFF via mushroom).")
    return estop_latched

def estop_reset():
    """Call after the mushroom is released and 24 V is back. Re-home before motion."""
    global estop_latched
    if estop_pressed_now():
        log("E-STOP still pressed; cannot reset.")
        return False
    log("Resetting E-STOP... waiting for drivers to re-energize...")
    time.sleep(1.5)
    estop_latched = False
    return True

# ----------------- HOMING (one NC switch per axis) -----------------
def home_axis(name, pin_step, pin_dir, pin_lim, search_positive=True, backoff_deg=2.0, approach_period_us=2000):
    """
    Move toward the NC home switch until it opens (reads HIGH), then back off and re-approach slowly.
    Returns the absolute step position assigned to 0 at the switch edge.
    """
    global cur_x_steps, cur_y_steps
    if estop_latch_if_pressed(): return None

    # If already tripped (HIGH), move away until it reads LOW
    away_forward = not search_positive
    set_dir(pin_dir, forward=away_forward, invert=False)
    tries = 0
    while limits_nc_tripped(pin_lim):
        if estop_latch_if_pressed(): return None
        if not emit_steps(pin_step, 1, period_us=approach_period_us): return None
        tries += 1
        if tries > 20000:
            log(f"[{name}] Unable to clear switch while backing off.")
            return None

    # Now approach toward the switch until it trips (goes HIGH)
    set_dir(pin_dir, forward=search_positive, invert=False)
    tries = 0
    while not limits_nc_tripped(pin_lim):
        if estop_latch_if_pressed(): return None
        if not emit_steps(pin_step, 1, period_us=approach_period_us): return None
        tries += 1
        if tries > 50000:
            log(f"[{name}] Did not find home limit; check wiring and direction.")
            return None

    # Back off slightly, then slowly re-approach for a clean edge
    steps_back = deg_to_steps(backoff_deg)
    set_dir(pin_dir, forward=away_forward, invert=False)
    if not emit_steps(pin_step, steps_back, period_us=approach_period_us): return None
    set_dir(pin_dir, forward=search_positive, invert=False)
    if not emit_steps(pin_step, steps_back + deg_to_steps(0.5), period_us=approach_period_us): return None

    # Assign 0 steps at home
    if pin_step == STEP_X:
        cur_x_steps = 0
    else:
        cur_y_steps = 0
    log(f"[{name}] Homed. Set position = 0 steps.")
    return 0

def home_both():
    log("Homing X...")
    if home_axis("X", STEP_X, DIR_X, LIM_X_MIN, search_positive=True) is None: return False
    log("Homing Y...")
    if home_axis("Y", STEP_Y, DIR_Y, LIM_Y_MIN, search_positive=True) is None: return False
    return True

# ----------------- MOTION -----------------
def move_to_deg(x_deg, y_deg):
    """Blocking move X then Y to absolute degrees within soft limits."""
    global cur_x_steps, cur_y_steps
    if estop_latch_if_pressed(): return False
    x_deg = clamp(x_deg, X_MIN_DEG, X_MAX_DEG)
    y_deg = clamp(y_deg, Y_MIN_DEG, Y_MAX_DEG)

    # X first
    target_x = deg_to_steps(x_deg)
    dx = target_x - cur_x_steps
    if dx != 0:
        set_dir(DIR_X, forward=(dx > 0), invert=False)
        if not emit_steps(STEP_X, abs(dx)): return False
        cur_x_steps = target_x

    # then Y
    target_y = deg_to_steps(y_deg)
    dy = target_y - cur_y_steps
    if dy != 0:
        set_dir(DIR_Y, forward=(dy > 0), invert=False)
        if not emit_steps(STEP_Y, abs(dy)): return False
        cur_y_steps = target_y

    return True

def move_norm(x_norm, y_norm):
    """Map 0..1 to [MIN..MAX] deg and move."""
    xd = X_MIN_DEG + clamp(x_norm, 0.0, 1.0) * (X_MAX_DEG - X_MIN_DEG)
    yd = Y_MIN_DEG + clamp(y_norm, 0.0, 1.0) * (Y_MAX_DEG - Y_MIN_DEG)
    return move_to_deg(xd, yd)

# ----------------- CSV -----------------
CSV_IS_NORMALIZED = True  # if False, CSV expected in degrees

def load_points(csv_path):
    pts = []
    with open(csv_path, newline='') as f:
        r = csv.reader(f)
        for row in r:
            if not row or row[0].strip().startswith("#"): continue
            try:
                a = float(row[0]); b = float(row[1])
            except Exception:
                continue
            pts.append((a, b))
    return pts

def run_points(pts, dwell_s=0.2):
    for (a, b) in pts:
        if estop_latch_if_pressed(): break
        if CSV_IS_NORMALIZED:
            ok = move_norm(a, b)
        else:
            ok = move_to_deg(a, b)
        if not ok: 
            log("Move aborted."); break
        fire_once(0.10)
        time.sleep(dwell_s)
    log("Sequence complete.")

# ----------------- SELF-TESTS -----------------
def wiring_self_test_x():
    """Jog X forward, then back, to verify pulses/dir path."""
    log("Self-test X: forward...")
    set_dir(DIR_X, forward=True)
    if not emit_steps(STEP_X, deg_to_steps(45), period_us=STEP_PERIOD_US): return
    time.sleep(0.2)
    log("Self-test X: back...")
    set_dir(DIR_X, forward=False)
    emit_steps(STEP_X, deg_to_steps(45), period_us=STEP_PERIOD_US)
    log("Done.")

def wiring_self_test_y():
    log("Self-test Y: forward...")
    set_dir(DIR_Y, forward=True)
    if not emit_steps(STEP_Y, deg_to_steps(45), period_us=STEP_PERIOD_US): return
    time.sleep(0.2)
    log("Self-test Y: back...")
    set_dir(DIR_Y, forward=False)
    emit_steps(STEP_Y, deg_to_steps(45), period_us=STEP_PERIOD_US)
    log("Done.")

# ----------------- CLI -----------------
def main_menu():
    print("\nVPP Turret Control")
    print("SAFE_MODE =", SAFE_MODE)
    print("[0] Wiring self-test (X)")
    print("[9] Wiring self-test (Y)")
    print("[1] Home both axes")
    print("[2] Move to degrees (X Y)")
    print("[3] Move to normalized (X Y in 0..1)")
    print("[4] Run CSV")
    print("[5] Toggle SAFE_MODE (current: %s)" % SAFE_MODE)
    print("[6] Reset E-STOP latch")
    print("[q] Quit")

def cleanup():
    try:
        _w(FIRE, 0)
        if not PIGPIO:
            GPIO.cleanup()
        else:
            pi.stop()
    except Exception:
        pass
    print("Clean exit.")

if __name__ == "__main__":
    try:
        while True:
            main_menu()
            c = input("Choose: ").strip().lower()
            if c == '0':
                wiring_self_test_x()
            elif c == '9':
                wiring_self_test_y()
            elif c == '1':
                ok = home_both()
                print("Home:", ok)
            elif c == '2':
                try:
                    x = float(input("X deg: "))
                    y = float(input("Y deg: "))
                except Exception:
                    print("bad numbers"); continue
                print("Move:", move_to_deg(x, y))
            elif c == '3':
                try:
                    x = float(input("X norm (0..1): "))
                    y = float(input("Y norm (0..1): "))
                except Exception:
                    print("bad numbers"); continue
                print("Move:", move_norm(x, y))
            elif c == '4':
                p = input("CSV path: ").strip()
                if not Path(p).exists():
                    print("File not found."); continue
                pts = load_points(p)
                print("Loaded", len(pts), "points.")
                run_points(pts, dwell_s=0.1)
            elif c == '5':
                SAFE = input("SAFE_MODE True/False: ").strip().lower()
                globals()['SAFE_MODE'] = (SAFE in ('1','true','t','yes','y'))
                print("SAFE_MODE now:", SAFE_MODE)
            elif c == '6':
                if estop_reset():
                    print("Latch cleared. Re-home before motion.")
            elif c == 'q':
                break
            else:
                print("Unknown choice.")
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
