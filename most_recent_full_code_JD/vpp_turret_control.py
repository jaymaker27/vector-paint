#!/usr/bin/env python3
"""
vpp_turret_control.py

Backend control for the Vector Projectile Painting (VPP) turret.

Hardware mapping (BCM numbering)
--------------------------------
GPIO 23 — Motor 1 PUL (step)  → buffer → Driver #1 PUL+ (360 Ω series)
GPIO 24 — Motor 1 DIR         → buffer → Driver #1 DIR+ (360 Ω series)

GPIO 20 — Motor 2 PUL (step)  → buffer → Driver #2 PUL+ (360 Ω series)
GPIO 21 — Motor 2 DIR         → buffer → Driver #2 DIR+ (360 Ω series)

GPIO 17 — Limit/Home X (NC to GND, internal pull-up enabled)
GPIO 27 — Limit/Home Y (NC to GND, internal pull-up enabled)

GPIO 18 — Marker/Relay/Indicator output (PWM-capable)
GPIO 25 — E-STOP status input (NO aux contact; closes to GND when pressed, pull-up enabled)

Notes:
- Driver negative inputs (PUL−, DIR−) return to logic ground.
- Each PUL pair includes a 10 kΩ shunt resistor.
- All signal lines incorporate 360 Ω series resistors.
- ENA pins are unconnected (drivers always enabled).

This module is designed to be **safe**:
- All public functions are wrapped so exceptions don't crash the UI.
- E-STOP and limits gate motion (safe_mode).
- GPIO-busy conditions are handled gracefully.

Coordinate system
-----------------
We maintain an internal step-based position:

    _POS_X_STEPS, _POS_Y_STEPS  → current steps from home
    _FWD_X_STEPS, _FWD_Y_STEPS  → "forward reference" steps

Typical workflow:
1. Press Calibrate → home_all()
   - Runs each axis into its NC home switch, backs off, sets (0, 0).

2. Jog via UI until the marker / laser is dead center in the camera.
3. Press "Set Forward" in UI → set_current_as_forward()
   - Stores current step position as the forward reference.

Later:
- goto_forward() returns to that reference pose.
- Painting / tracking can use these steps as a base for mapping image coords.
"""

import time
import threading
from typing import Dict, Any

try:
    import RPi.GPIO as GPIO
except Exception as e:  # pragma: no cover (on non-Pi dev)
    GPIO = None
    print("[turret] WARNING: RPi.GPIO not available:", e)

# ---------------- GPIO PINS (BCM) ----------------

STEP_X_PIN = 23
DIR_X_PIN  = 24

STEP_Y_PIN = 20
DIR_Y_PIN  = 21

LIM_X_PIN  = 17  # NC to GND, pulls up when TRIPPED
LIM_Y_PIN  = 27  # NC to GND, pulls up when TRIPPED

FIRE_PIN   = 18  # Marker/relay
ESTOP_PIN  = 25  # NO to GND when pressed

# ---------------- MOTION CONSTANTS ----------------

BASE_STEP_DELAY    = 0.0008   # seconds between steps at speed_scale=1.0
HOMING_STEP_DELAY  = 0.0010   # slightly slower for homing
MAX_HOMING_STEPS   = 20000    # safety bound

# Jogging: base steps for a "unit" nudge.
# You can tune these after seeing how far one jog moves in reality.
DEFAULT_JOG_STEPS  = 400       # was 50; make jogs much more visible
JOG_STEPS_PER_DEG  = 200       # ~200 steps per degree as a starting guess

FIRE_PULSE_SEC     = 0.150    # marker/relay pulse duration

# Motion profile scaling (can be tuned from UI)
_MOTION_PROFILE = {
    "x_speed_scale": 1.0,
    "y_speed_scale": 1.0,
}

# ---------------- INTERNAL STATE ----------------

_GPIO_READY   = False
_STATE_LOCK   = threading.Lock()

# Current position in steps from home (0,0)
_POS_X_STEPS  = 0
_POS_Y_STEPS  = 0

# "Forward" reference in steps (camera-center / forward align)
_FWD_X_STEPS  = 0
_FWD_Y_STEPS  = 0

# Flags
_TRACKING_ENABLED = False
_AUTOFIRE_ENABLED = False
_SENTRY_ENABLED   = False

# ---------------- SAFE WRAPPER ----------------

def _safe(func):
    """Decorator: catch all exceptions and log them instead of crashing UI."""
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:  # pragma: no cover
            import traceback
            print(f"[turret] ERROR in {func.__name__}: {e}")
            traceback.print_exc()
            return None
    return wrapper

# ---------------- GPIO SETUP / TEARDOWN ----------------

def _init_gpio():
    global _GPIO_READY

    if GPIO is None:
        print("[turret] GPIO init skipped: RPi.GPIO not available")
        _GPIO_READY = False
        return

    if _GPIO_READY:
        return

    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Outputs
        GPIO.setup(STEP_X_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(DIR_X_PIN,  GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(STEP_Y_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(DIR_Y_PIN,  GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(FIRE_PIN, GPIO.OUT, initial=GPIO.LOW)

        # Inputs with pull-ups
        GPIO.setup(LIM_X_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LIM_Y_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ESTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        _GPIO_READY = True
        print("[turret] GPIO initialized")
    except Exception as e:
        # If GPIO is "busy", it usually means another process (like the running UI)
        # already owns the lines. That's OK in the UI process; in a second process,
        # we just warn.
        print(f"[turret] GPIO init failed: {e}")
        _GPIO_READY = False

def _ensure_gpio() -> bool:
    if not _GPIO_READY:
        _init_gpio()
    return _GPIO_READY

@_safe
def shutdown():
    """Release GPIO resources. UI calls this on exit."""
    global _GPIO_READY
    if GPIO is not None and _GPIO_READY:
        GPIO.cleanup()
        _GPIO_READY = False
        print("[turret] GPIO cleaned up")

# ---------------- INPUT HELPERS ----------------

def _estop_pressed() -> bool:
    """E-STOP is NO to GND, pull-up enabled.

    - Idle (not pressed) : open, reads 1
    - Pressed            : closed to GND, reads 0

    We debounce by sampling a few times and taking a majority vote so that
    electrical noise during stepping doesn't trigger false presses.
    """
    if not _ensure_gpio():
        return False

    try:
        lows = 0
        highs = 0
        # Sample a few times very quickly (no sleep) to reject glitches.
        for _ in range(5):
            val = GPIO.input(ESTOP_PIN)
            if val == GPIO.LOW:
                lows += 1
            else:
                highs += 1
        # Consider E-STOP pressed only if it's LOW in the majority of samples.
        return lows >= 3
    except Exception:
        return False

def _limit_tripped(pin: int) -> bool:
    """NC limit switches to GND with pull-ups.

    Electrically:
      - Idle (not pressed, circuit closed to GND) -> reads 0
      - TRIPPED (pressed, circuit open, pulled up) -> reads 1

    We want:
      - idle  -> OK
      - pressed -> TRIPPED
    """
    if not _ensure_gpio():
        return False
    try:
        return GPIO.input(pin) == GPIO.HIGH
    except Exception:
        return False

def _limits_ok() -> Dict[str, bool]:
    """Return limit OK flags for X and Y (True means OK, False means TRIPPED)."""
    x_ok = not _limit_tripped(LIM_X_PIN)
    y_ok = not _limit_tripped(LIM_Y_PIN)
    return {"x_limit_ok": x_ok, "y_limit_ok": y_ok}
@_safe
def debug_gpio_snapshot():
    """
    Print raw GPIO readings for ESTOP and limit switches.

    Use this from a REPL with switches in different states to verify wiring
    and polarity. Only works when GPIO is actually available (no 'busy').
    """
    if not _ensure_gpio():
        print("[turret] debug_gpio_snapshot: GPIO not ready")
        return

    try:
        estop_raw = GPIO.input(ESTOP_PIN)
        x_raw = GPIO.input(LIM_X_PIN)
        y_raw = GPIO.input(LIM_Y_PIN)
    except Exception as e:
        print("[turret] debug_gpio_snapshot error:", e)
        return

    print("[turret] GPIO snapshot:")
    print(f"  ESTOP raw: {estop_raw}  (0 = grounded, 1 = pulled up)")
    print(f"  LIM_X raw: {x_raw}      (0 = grounded, 1 = pulled up)")
    print(f"  LIM_Y raw: {y_raw}      (0 = grounded, 1 = pulled up)")

# ---------------- STATUS ----------------

@_safe
def get_status() -> Dict[str, Any]:
    """
    Return a status dictionary for the UI.

    Keys:
      - estop       : bool (True if pressed)
      - x_limit_ok  : bool
      - y_limit_ok  : bool
      - safe_mode   : bool (True when estop pressed or any limit tripped)
      - pos_steps   : dict {'x': int, 'y': int}
      - forward_ref : dict {'x': int, 'y': int}
      - tracking    : bool
      - autofire    : bool
      - sentry      : bool
    """
    if not _ensure_gpio():
        # Still provide meaningful info if GPIO isn't ready
        with _STATE_LOCK:
            pos = {"x": _POS_X_STEPS, "y": _POS_Y_STEPS}
            fwd = {"x": _FWD_X_STEPS, "y": _FWD_Y_STEPS}
            tracking = _TRACKING_ENABLED
            autofire = _AUTOFIRE_ENABLED
            sentry = _SENTRY_ENABLED
        return {
            "estop": None,
            "x_limit_ok": None,
            "y_limit_ok": None,
            "safe_mode": True,
            "pos_steps": pos,
            "forward_ref": fwd,
            "tracking": tracking,
            "autofire": autofire,
            "sentry": sentry,
        }

    estop = _estop_pressed()
    limits = _limits_ok()
    safe_mode = bool(estop or not limits["x_limit_ok"] or not limits["y_limit_ok"])

    with _STATE_LOCK:
        pos = {"x": _POS_X_STEPS, "y": _POS_Y_STEPS}
        fwd = {"x": _FWD_X_STEPS, "y": _FWD_Y_STEPS}
        tracking = _TRACKING_ENABLED
        autofire = _AUTOFIRE_ENABLED
        sentry = _SENTRY_ENABLED

    return {
        "estop": estop,
        "x_limit_ok": limits["x_limit_ok"],
        "y_limit_ok": limits["y_limit_ok"],
        "safe_mode": safe_mode,
        "pos_steps": pos,
        "forward_ref": fwd,
        "tracking": tracking,
        "autofire": autofire,
        "sentry": sentry,
    }

# ---------------- POSITION HELPERS ----------------

def _set_position(x=None, y=None):
    global _POS_X_STEPS, _POS_Y_STEPS
    with _STATE_LOCK:
        if x is not None:
            _POS_X_STEPS = int(x)
        if y is not None:
            _POS_Y_STEPS = int(y)

@_safe
def get_position_steps() -> Dict[str, int]:
    """Return current step-based position from home."""
    with _STATE_LOCK:
        return {"x": _POS_X_STEPS, "y": _POS_Y_STEPS}

@_safe
def set_current_as_forward():
    """
    Mark the current step position as the 'forward / camera-center' reference.

    Recommended procedure:
      1. Home (Calibrate / Home All).
      2. Jog until the marker/laser is centered in the camera view.
      3. Call this once.
    """
    global _FWD_X_STEPS, _FWD_Y_STEPS
    with _STATE_LOCK:
        _FWD_X_STEPS = _POS_X_STEPS
        _FWD_Y_STEPS = _POS_Y_STEPS
        fx, fy = _FWD_X_STEPS, _FWD_Y_STEPS
    print(f"[turret] Forward reference set at X={fx}, Y={fy}")

@_safe
def goto_forward():
    """Move back to the stored forward reference pose."""
    if _estop_pressed():
        print("[turret] goto_forward aborted: E-STOP pressed")
        return

    with _STATE_LOCK:
        dx = _FWD_X_STEPS - _POS_X_STEPS
        dy = _FWD_Y_STEPS - _POS_Y_STEPS
        x_scale = _MOTION_PROFILE["x_speed_scale"]
        y_scale = _MOTION_PROFILE["y_speed_scale"]

    if dx != 0:
        _move_axis_with_pos("X", STEP_X_PIN, DIR_X_PIN, dx, BASE_STEP_DELAY * x_scale)
    if dy != 0:
        _move_axis_with_pos("Y", STEP_Y_PIN, DIR_Y_PIN, dy, BASE_STEP_DELAY * y_scale)

# ---------------- LOW-LEVEL MOTION ----------------

def _pulse_step(step_pin: int, delay_s: float):
    """Single step pulse on a given step pin."""
    if not _ensure_gpio():
        return
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(delay_s / 2.0)
    GPIO.output(step_pin, GPIO.LOW)
    time.sleep(delay_s / 2.0)

def _move_axis_with_pos(axis: str, step_pin: int, dir_pin: int, steps: int, delay_s: float):
    """Move an axis by 'steps' and update internal step position."""
    global _POS_X_STEPS, _POS_Y_STEPS

    if not _ensure_gpio():
        return
    if steps == 0:
        return

    # E-STOP check before motion
    if _estop_pressed():
        print(f"[turret] move {axis}: aborted (E-STOP pressed)")
        return

    # Decide direction line level
    direction = GPIO.HIGH if steps > 0 else GPIO.LOW
    GPIO.output(dir_pin, direction)

    total = abs(steps)
    for i in range(total):
        # During motion, bail out if E-STOP pressed
        if _estop_pressed():
            print(f"[turret] move {axis}: interrupted by E-STOP at step {i}/{total}")
            break

        _pulse_step(step_pin, delay_s)

    # Update position (we assume all steps completed; if we wanted,
    # we could use 'i' from above for partial moves)
    with _STATE_LOCK:
        if axis == "X":
            _POS_X_STEPS += steps
        elif axis == "Y":
            _POS_Y_STEPS += steps

# ---------------- HOMING ----------------
@_safe
def _home_single_axis(
    axis: str,
    step_pin: int,
    dir_pin: int,
    limit_pin: int,
    step_delay: float = HOMING_STEP_DELAY,
    max_steps: int = MAX_HOMING_STEPS,
):
    """
    Home a single axis toward its NC limit switch, then back off and zero.

    Strategy:
      1) If the switch is currently TRIPPED, move away until it clears.
      2) Move toward the switch until it trips.
      3) Back off a fixed amount and define that as 0.
    """
    if not _ensure_gpio():
        print(f"[turret] Cannot home {axis}: GPIO not ready")
        return

    print(f"[turret] Homing axis {axis}...")

    # Safety: if E-STOP is pressed, don't move at all.
    if _estop_pressed():
        print(f"[turret] ABORT homing {axis}: E-STOP pressed at start")
        return

    # -----------------------------------------------------------------
    # Phase 1: if we're already on the switch, back off until it clears.
    # -----------------------------------------------------------------
    if _limit_tripped(limit_pin):
        print(f"[turret] Axis {axis} is already on home switch; backing off to clear")
        GPIO.output(dir_pin, GPIO.HIGH)  # define HIGH as "away from home"

        steps = 0
        while _limit_tripped(limit_pin):
            if _estop_pressed():
                print(f"[turret] ABORT homing {axis}: E-STOP pressed while clearing")
                return
            if steps >= max_steps:
                print(f"[turret] ABORT homing {axis}: max_steps exceeded while clearing")
                return

            _pulse_step(step_pin, step_delay)
            steps += 1

        print(f"[turret] Axis {axis} cleared home switch after {steps} steps")

    # -----------------------------------------------------------------
    # Phase 2: move toward the switch until it trips.
    # -----------------------------------------------------------------
    GPIO.output(dir_pin, GPIO.LOW)  # define LOW as "toward home"
    steps = 0

    while not _limit_tripped(limit_pin):
        if _estop_pressed():
            print(f"[turret] ABORT homing {axis}: E-STOP pressed while moving toward home")
            return
        if steps >= max_steps:
            print(f"[turret] ABORT homing {axis}: max_steps exceeded while seeking home")
            return

        _pulse_step(step_pin, step_delay)
        steps += 1

    print(f"[turret] Axis {axis} hit home after {steps} steps")

    # -----------------------------------------------------------------
    # Phase 3: back off until the switch clears, then define that as 0.
    # -----------------------------------------------------------------
    print(f"[turret] Axis {axis} backing off from switch")
    GPIO.output(dir_pin, GPIO.HIGH)  # HIGH = away from home

    backoff_steps = 0
    # First, ensure we move at least a small amount.
    MIN_BACKOFF = 80

    # Back off until limit is no longer tripped, or until we hit a safety cap.
    while (_limit_tripped(limit_pin) or backoff_steps < MIN_BACKOFF) and backoff_steps < (max_steps // 2):
        if _estop_pressed():
            print(f"[turret] ABORT homing {axis}: E-STOP pressed while backing off")
            return
        _pulse_step(step_pin, step_delay)
        backoff_steps += 1

    if _limit_tripped(limit_pin):
        print(f"[turret] WARNING: Axis {axis} still reports TRIPPED after "
              f"{backoff_steps} backoff steps")
    else:
        print(f"[turret] Axis {axis} cleared switch after {backoff_steps} backoff steps")

    global _POS_X_STEPS, _POS_Y_STEPS
    with _STATE_LOCK:
        if axis == "X":
            _POS_X_STEPS = 0
        elif axis == "Y":
            _POS_Y_STEPS = 0

    print(f"[turret] Axis {axis} homed and zeroed (off switch)")


@_safe
def home_all():
    """Home both axes and zero their step positions."""
    print("[turret] Home all axes requested")

    if _estop_pressed():
        print("[turret] home_all aborted: E-STOP pressed")
        return

    # Home X first, then Y
    _home_single_axis("X", STEP_X_PIN, DIR_X_PIN, LIM_X_PIN)
    _home_single_axis("Y", STEP_Y_PIN, DIR_Y_PIN, LIM_Y_PIN)

    print("[turret] Home all complete")


@_safe
def calibrate_all():
    """
    Wrapper used by UI 'Calibrate' button.

    Currently just homes all axes, but we can extend this later
    (e.g., forward reference setup, additional checks).
    """
    print("[turret] Calibrate requested")
    home_all()

# ---------------- JOGGING ----------------

@_safe
def jog_xy(x_steps: int, y_steps: int, speed_scale: float = 1.0):
    """
    Jog the turret by a certain number of steps on X and Y.

    Positive steps = DIR_PIN HIGH
    Negative steps = DIR_PIN LOW

    speed_scale multiplies BASE_STEP_DELAY (higher = slower).
    """
    if not _ensure_gpio():
        print("[turret] jog_xy aborted: GPIO not ready")
        return

    if _estop_pressed():
        print("[turret] jog_xy aborted: E-STOP pressed")
        return

    limits = _limits_ok()
    if not limits["x_limit_ok"] or not limits["y_limit_ok"]:
        print("[turret] jog_xy aborted: limit switch tripped", limits)
        return

    # Effective delays
    delay_x = BASE_STEP_DELAY * max(0.1, min(speed_scale * _MOTION_PROFILE["x_speed_scale"], 10.0))
    delay_y = BASE_STEP_DELAY * max(0.1, min(speed_scale * _MOTION_PROFILE["y_speed_scale"], 10.0))

    print(f"[turret] jog_xy: X={x_steps}, Y={y_steps}, "
          f"delay_x={delay_x:.6f}, delay_y={delay_y:.6f}")

    if x_steps != 0:
        _move_axis_with_pos("X", STEP_X_PIN, DIR_X_PIN, x_steps, delay_x)
    if y_steps != 0:
        _move_axis_with_pos("Y", STEP_Y_PIN, DIR_Y_PIN, y_steps, delay_y)
@_safe
def jog(axis: str, direction: int, step_deg: float, speed: float):
    """
    UI hook for manual jog buttons (Settings dialog).

    We map degrees -> steps using JOG_STEPS_PER_DEG, and also enforce a
    minimum of DEFAULT_JOG_STEPS so you can see movement even for tiny deg.
    """
    axis = (axis or "").upper()
    direction = 1 if direction >= 0 else -1

    # How many steps for this jog?
    try:
        step_deg = float(step_deg)
    except Exception:
        step_deg = 2.0  # fallback

    steps_from_deg = int(abs(step_deg) * JOG_STEPS_PER_DEG)
    steps = max(DEFAULT_JOG_STEPS, steps_from_deg)
    steps *= direction

    if axis == "X":
        x_steps = steps
        y_steps = 0
    elif axis == "Y":
        x_steps = 0
        y_steps = steps
    else:
        print(f"[turret] jog: unknown axis {axis}")
        return

    # Convert the UI speed into a speed_scale for jog_xy
    try:
        speed = float(speed)
    except Exception:
        speed = 1000.0

    base_speed = 1200.0  # our reference speed
    speed_scale = base_speed / max(1.0, speed)

    print(f"[turret] jog: axis={axis}, dir={direction}, "
          f"steps=({x_steps},{y_steps}), step_deg={step_deg}, "
          f"speed={speed}, scale={speed_scale:.3f}")

    jog_xy(x_steps, y_steps, speed_scale=speed_scale)

# ---------------- MOTION PROFILE ----------------

@_safe
def set_motion_profile(x_speed_scale: float = None, y_speed_scale: float = None):
    """
    Adjust the motion profile from the Settings menu.

    Example:
      set_motion_profile(x_speed_scale=0.5, y_speed_scale=1.5)
    """
    if x_speed_scale is not None:
        _MOTION_PROFILE["x_speed_scale"] = max(0.1, min(float(x_speed_scale), 10.0))
    if y_speed_scale is not None:
        _MOTION_PROFILE["y_speed_scale"] = max(0.1, min(float(y_speed_scale), 10.0))

    print("[turret] Motion profile updated:", _MOTION_PROFILE)
@_safe
def set_motor_speeds(x_speed: float, y_speed: float):
    """
    UI hook: map user-friendly speed values into our motion profile.

    The UI treats x_speed, y_speed as an abstract "speed" number.
    We convert that into a scale factor for BASE_STEP_DELAY.

    Higher x_speed/y_speed -> faster motion (smaller delay).
    """
    try:
        x_speed = float(x_speed)
        y_speed = float(y_speed)
    except Exception:
        print("[turret] set_motor_speeds: invalid values, keeping previous profile")
        return

    # Avoid zero / negative and clamp to something reasonable
    x_speed = max(1.0, x_speed)
    y_speed = max(1.0, y_speed)

    # Use 1200 as our "baseline" speed (matches UI defaults)
    base_speed = 1200.0
    x_scale = base_speed / x_speed   # bigger speed -> smaller scale -> faster
    y_scale = base_speed / y_speed

    set_motion_profile(x_speed_scale=x_scale, y_speed_scale=y_scale)
    print("[turret] set_motor_speeds:",
          {"x_speed": x_speed, "y_speed": y_speed,
           "x_scale": x_scale, "y_scale": y_scale})
# ---------------- FIRE CONTROL ----------------

@_safe
def manual_fire(pulse_sec: float = FIRE_PULSE_SEC):
    """Pulse the marker/relay on FIRE_PIN for a short duration."""
    if not _ensure_gpio():
        print("[turret] manual_fire aborted: GPIO not ready")
        return

    if _estop_pressed():
        print("[turret] manual_fire aborted: E-STOP pressed")
        return

    print(f"[turret] manual_fire: pulse {pulse_sec:.3f} s")
    GPIO.output(FIRE_PIN, GPIO.HIGH)
    time.sleep(max(0.01, pulse_sec))
    GPIO.output(FIRE_PIN, GPIO.LOW)

@_safe
def test_fire():
    """Shortcut for UI 'Test Fire' button."""
    manual_fire()

# ---------------- SESSION / PAINT PLACEHOLDERS ----------------

@_safe
def start_paint_session():
    """Called when UI enters a paint session."""
    print("[turret] start_paint_session")

@_safe
def end_paint_session():
    """Called when UI leaves a paint session."""
    print("[turret] end_paint_session")

@_safe
def start_paint_from_image(job: dict):
    """
    Called by UI after user selects an image and segmentation job is created.

    'job' is expected to be a dict describing passes / splats.
    We **don't** assume a specific schema here; the UI can log/inspect it.
    """
    print("[turret] start_paint_from_image: received job:")
    print(repr(job)[:1000], "...")
    # In a later iteration, this is where we'll precompute step paths.
@_safe
def run_paint_job(job: dict):
    """
    Execute a paint job defined by the UI.

    CURRENTLY A SAFE PLACEHOLDER:
      - Logs passes and point counts.
      - Does NOT move motors or fire.
    """
    if not job:
        print("[turret] run_paint_job: empty job")
        return

    passes = job.get("passes") or []
    print(f"[turret] run_paint_job: mode={job.get('mode')}, passes={len(passes)}")

    for idx, p in enumerate(passes):
        pts = p.get("points") or []
        print(f"  pass {idx}: label={p.get('label')!r}, "
              f"points={len(pts)}, color={p.get('color')}")

    print("[turret] run_paint_job: placeholder implementation (no motion yet)")

@_safe
def run_paint_pass(job: dict, pass_index: int = 0):
    """
    Execute a single paint pass from a prepared job.

    NOTE: This is a placeholder. To make it fully physical we need:
      - Mapping from image coords → turret step coords.
      - Scaling derived from calibration (forward reference + known target size).
    For now, this just logs and can be extended later.
    """
    print(f"[turret] run_paint_pass: pass_index={pass_index}")
    print(repr(job)[:500], "...")
    # TODO: implement real move sequences once mapping constants are known.

# ---------------- TRACKING / SENTRY FLAGS ----------------

@_safe
def set_tracking_enabled(enabled: bool):
    global _TRACKING_ENABLED
    _TRACKING_ENABLED = bool(enabled)
    print("[turret] Tracking enabled:", _TRACKING_ENABLED)

@_safe
def set_autofire_enabled(enabled: bool):
    global _AUTOFIRE_ENABLED
    _AUTOFIRE_ENABLED = bool(enabled)
    print("[turret] Autofire enabled:", _AUTOFIRE_ENABLED)

@_safe
def set_sentry_mode(enabled: bool):
    """
    Enable/disable "Predator Sentry Mode".

    This backend just flips a flag; the UI / vision loop is responsible
    for:
      - sweeping the turret when sentry is True
      - detecting targets in the camera frame
      - calling jog_xy() and manual_fire() as appropriate
    """
    global _SENTRY_ENABLED
    _SENTRY_ENABLED = bool(enabled)
    print("[turret] Sentry mode:", _SENTRY_ENABLED)
@_safe
def sentry_scan_step(direction: int):
    """
    Small horizontal sweep step for Predator sentry mode.

    direction: +1 (right) or -1 (left).
    """
    direction = 1 if direction >= 0 else -1
    # Small, gentle jog so we don't slam into limits quickly
    steps = direction * max(5, DEFAULT_JOG_STEPS // 4)
    print(f"[turret] sentry_scan_step: direction={direction}, steps={steps}")
    jog_xy(steps, 0, speed_scale=1.5)  # slightly slower than base


@_safe
def sentry_fire_at(x_norm: float, y_norm: float):
    """
    Called by UI predator mode when it has a target at (x_norm, y_norm) in [0..1].

    We **only** fire if:
      - tracking is enabled AND
      - autofire is enabled AND
      - E-STOP is not pressed and GPIO is ready.
    """
    global _TRACKING_ENABLED, _AUTOFIRE_ENABLED

    print(f"[turret] sentry_fire_at: target at ({x_norm:.3f}, {y_norm:.3f})")

    # Gate 1: tracking toggle (UI "Enable tracking")
    if not _TRACKING_ENABLED:
        print("[turret] sentry_fire_at: tracking disabled → not firing")
        return

    # Gate 2: autofire toggle (UI "Auto fire on target")
    if not _AUTOFIRE_ENABLED:
        print("[turret] sentry_fire_at: autofire disabled → not firing")
        return

    # Gate 3: E-STOP and GPIO sanity
    if not _ensure_gpio():
        print("[turret] sentry_fire_at: GPIO not ready → not firing")
        return
    if _estop_pressed():
        print("[turret] sentry_fire_at: E-STOP pressed → not firing")
        return

    print("[turret] sentry_fire_at: tracking+autofire enabled → firing")
    manual_fire()


# ---------------- MODULE IMPORT SIDE EFFECT ----------------

print("[turret] vpp_turret_control module loaded (GPIO will init on first use)")
