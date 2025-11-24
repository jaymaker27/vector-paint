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

Safety philosophy
-----------------
- All public functions are decorated with @_safe so exceptions won't crash the UI.
- E-STOP and limits gate motion.
- We expose get_status() so the UI can show E-STOP, limits, and safe_mode.
- Paint jobs can be cancelled with request_abort() from the UI ("Cancel Paint" button).

Coordinate system
-----------------
We maintain internal step-based positions:

    _POS_X_STEPS, _POS_Y_STEPS  → current steps from home
    _FWD_X_STEPS, _FWD_Y_STEPS  → "forward reference" steps

Typical workflow:
1. Press Calibrate → home_all()
   - Runs each axis into its NC home switch, backs off, sets (0, 0).

2. Jog via UI until the marker / laser is dead center in the camera.
3. Press "Set Current Aim as Center" in the UI → set_current_as_forward()
   - Stores current step position as the forward reference.

Later:
- goto_forward() returns to that reference pose.
- Painting / tracking use these as the base pose, with small offsets.
"""

import time
import threading
from typing import Dict, Any, Tuple

try:
    import RPi.GPIO as GPIO
except Exception as e:  # pragma: no cover (on non-Pi dev)
    GPIO = None
    print("[turret] WARNING: RPi.GPIO not available:", e)
from pathlib import Path
import json

# ------------------------------------------------------------
# ------------------------------------------------------------
# Soft travel limits (no upper limit switch → software limits)
# ------------------------------------------------------------

SOFT_LIMITS_FILE = Path.home() / ".vpp_soft_limits.json"
TRACK_INVERT_FILE = Path.home() / ".vpp_tracking_inversion.json"

# Default: we know min is home (0), max is unknown until calibrated.
# These are in MOTOR STEPS, relative to the homed position.
SOFT_LIMITS = {
    "x_min": 0,
    "x_max": None,  # filled in by travel calibration
    "y_min": 0,
    "y_max": None,
}

# Debug view of our logical position in motor steps, relative to HOME.
# We keep this in sync with the internal _POS_* counters that the rest
# of the backend uses.
CUR_X_STEPS = 0
CUR_Y_STEPS = 0


def _load_soft_limits():
    """Load soft limits from disk (if present)."""
    global SOFT_LIMITS
    try:
        data = json.loads(SOFT_LIMITS_FILE.read_text())
        if isinstance(data, dict):
            SOFT_LIMITS.update(data)
        print(f"[turret] Soft limits loaded: {SOFT_LIMITS}")
    except FileNotFoundError:
        print("[turret] No soft limits file yet")
    except Exception as e:
        print(f"[turret] Failed to load soft limits: {e}")


def _save_soft_limits():
    """Persist soft limits to disk."""
    try:
        SOFT_LIMITS_FILE.write_text(json.dumps(SOFT_LIMITS))
        print(f"[turret] Soft limits saved: {SOFT_LIMITS}")
    except Exception as e:
        print(f"[turret] Failed to save soft limits: {e}")
def _load_tracking_inversion():
    """Load tracking inversion preferences from disk (if present)."""
    global TRACK_INVERT_X, TRACK_INVERT_Y
    try:
        data = json.loads(TRACK_INVERT_FILE.read_text())
        if isinstance(data, dict):
            ix = data.get("invert_x")
            iy = data.get("invert_y")
            if isinstance(ix, bool):
                TRACK_INVERT_X = -1 if ix else 1
            if isinstance(iy, bool):
                TRACK_INVERT_Y = -1 if iy else 1
        print(
            "[turret] Tracking inversion loaded:",
            {"TRACK_INVERT_X": TRACK_INVERT_X, "TRACK_INVERT_Y": TRACK_INVERT_Y},
        )
    except FileNotFoundError:
        # No preferences yet; leave defaults
        pass
    except Exception as e:
        print(f"[turret] Failed to load tracking inversion: {e}")


def _save_tracking_inversion(invert_x: bool, invert_y: bool):
    """Persist tracking inversion preferences to disk."""
    payload = {"invert_x": bool(invert_x), "invert_y": bool(invert_y)}
    try:
        TRACK_INVERT_FILE.write_text(json.dumps(payload))
        print("[turret] Tracking inversion saved:", payload)
    except Exception as e:
        print(f"[turret] Failed to save tracking inversion: {e}")


def _set_current_position_steps(x_steps=None, y_steps=None):
    """
    Update our logical idea of where the turret is (in steps).

    This keeps CUR_* and the internal _POS_* counters in sync so that
    soft limits and UI debug both see the same position.
    """
    global CUR_X_STEPS, CUR_Y_STEPS, _POS_X_STEPS, _POS_Y_STEPS
    if x_steps is not None:
        CUR_X_STEPS = int(x_steps)
    if y_steps is not None:
        CUR_Y_STEPS = int(y_steps)

    # Mirror into the main position counters under the state lock.
    # _STATE_LOCK and _POS_* are defined later in the file; by the
    # time this is actually CALLED (e.g. from home_all), they exist.
    try:
        with _STATE_LOCK:
            if x_steps is not None:
                _POS_X_STEPS = CUR_X_STEPS
            if y_steps is not None:
                _POS_Y_STEPS = CUR_Y_STEPS
    except NameError:
        # If someone calls this very early during import before those
        # globals exist, just skip syncing; they'll be set later anyway.
        pass

    print(f"[turret] Position now X={CUR_X_STEPS}, Y={CUR_Y_STEPS}")


# Load soft limits once at import so jog_xy can enforce them.
_load_soft_limits()
# Load tracking inversion preferences (if any) so Predator mode starts with them
_load_tracking_inversion()

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
DEFAULT_JOG_STEPS  = 800       # larger so jogs are visible
JOG_STEPS_PER_DEG  = 200       # rough mapping, tune later

FIRE_PULSE_SEC     = 0.150    # marker/relay pulse duration

# Motion profile scaling (can be tuned from UI)
_MOTION_PROFILE = {
    "x_speed_scale": 1.0,  # multiplies BASE_STEP_DELAY
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

# Paint-job abort flag (set by UI "Cancel Paint" button)
_ABORT_JOB = False


# ---------------- SAFE WRAPPER ----------------

def _safe(func):
    """Decorator: catch all exceptions and log them instead of crashing the UI."""
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
    """Initialize GPIO; safe to call multiple times."""
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
        # If GPIO is "busy", it usually means another process owns the lines.
        print(f"[turret] GPIO init failed: {e}")
        _GPIO_READY = False


def _ensure_gpio() -> bool:
    """Lazy-init GPIO and report readiness."""
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
    """Read debounced E-STOP.

    ESTOP is NO to GND, pull-up enabled:
      - Idle (not pressed): open, reads 1
      - Pressed            : closed to GND, reads 0
    """
    if not _ensure_gpio():
        return False

    try:
        lows = 0
        highs = 0
        for _ in range(5):
            val = GPIO.input(ESTOP_PIN)
            if val == GPIO.LOW:
                lows += 1
            else:
                highs += 1
        return lows >= 3
    except Exception:
        return False


def _limit_tripped(pin: int) -> bool:
    """NC limit switches to GND with pull-ups.

    Electrically:
      - Idle (not pressed, circuit closed to GND) -> reads 0
      - TRIPPED (pressed, circuit open, pulled up) -> reads 1

    Here we return True when TRIPPED.
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

    Use from a REPL to verify wiring and polarity.
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
      - estop       : bool or None
      - x_limit_ok  : bool or None
      - y_limit_ok  : bool or None
      - safe_mode   : bool
      - pos_steps   : dict {'x': int, 'y': int}
      - forward_ref : dict {'x': int, 'y': int}
      - tracking    : bool
      - autofire    : bool
      - sentry      : bool
    """
    with _STATE_LOCK:
        pos = {"x": _POS_X_STEPS, "y": _POS_Y_STEPS}
        fwd = {"x": _FWD_X_STEPS, "y": _FWD_Y_STEPS}
        tracking = _TRACKING_ENABLED
        autofire = _AUTOFIRE_ENABLED
        sentry = _SENTRY_ENABLED

    if not _ensure_gpio():
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

def _set_position(x: int = None, y: int = None):
    """Internal helper to set current step position."""
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
      3. Call this once from the UI.
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
    """
    Move an axis by 'steps' and update internal step position.

    steps > 0 → DIR_PIN HIGH
    steps < 0 → DIR_PIN LOW
    """
    global _POS_X_STEPS, _POS_Y_STEPS

    if not _ensure_gpio():
        return
    if steps == 0:
        return

    if _estop_pressed():
        print(f"[turret] move {axis}: aborted (E-STOP pressed at start)")
        return

    direction_level = GPIO.HIGH if steps > 0 else GPIO.LOW
    GPIO.output(dir_pin, direction_level)

    total = abs(steps)
    steps_done = 0

    for i in range(total):
        if _estop_pressed():
            print(f"[turret] move {axis}: interrupted by E-STOP at step {i}/{total}")
            break
        _pulse_step(step_pin, delay_s)
        steps_done += 1

    # Update position based on actual steps done
    delta = steps_done if steps > 0 else -steps_done
    with _STATE_LOCK:
        if axis == "X":
            _POS_X_STEPS += delta
        elif axis == "Y":
            _POS_Y_STEPS += delta


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

    if _estop_pressed():
        print(f"[turret] ABORT homing {axis}: E-STOP pressed at start")
        return

    # Phase 1: if we're already on the switch, back off until it clears.
    if _limit_tripped(limit_pin):
        print(f"[turret] Axis {axis} is already on home switch; backing off to clear")
        GPIO.output(dir_pin, GPIO.HIGH)  # define HIGH as "away from home"

        steps = 0
        while _limit_tripped(limit_pin):
            if _estop_pressed():
                print(f"[turret] ABORT homing {axis}: E-STOP while clearing")
                return
            if steps >= max_steps:
                print(f"[turret] ABORT homing {axis}: max_steps exceeded while clearing")
                return
            _pulse_step(step_pin, step_delay)
            steps += 1

        print(f"[turret] Axis {axis} cleared home switch after {steps} steps")

    # Phase 2: move toward the switch until it trips.
    GPIO.output(dir_pin, GPIO.LOW)  # LOW as "toward home"
    steps = 0

    while not _limit_tripped(limit_pin):
        if _estop_pressed():
            print(f"[turret] ABORT homing {axis}: E-STOP while seeking home")
            return
        if steps >= max_steps:
            print(f"[turret] ABORT homing {axis}: max_steps exceeded while seeking home")
            return
        _pulse_step(step_pin, step_delay)
        steps += 1

    print(f"[turret] Axis {axis} hit home after {steps} steps")

    # Phase 3: back off until the switch clears, then define that as 0.
    print(f"[turret] Axis {axis} backing off from switch")
    GPIO.output(dir_pin, GPIO.HIGH)  # HIGH = away from home

    backoff_steps = 0
    MIN_BACKOFF = 80

    while (_limit_tripped(limit_pin) or backoff_steps < MIN_BACKOFF) and backoff_steps < (max_steps // 2):
        if _estop_pressed():
            print(f"[turret] ABORT homing {axis}: E-STOP while backing off")
            return
        _pulse_step(step_pin, step_delay)
        backoff_steps += 1

    if _limit_tripped(limit_pin):
        print(f"[turret] WARNING: Axis {axis} still TRIPPED after {backoff_steps} backoff steps")
    else:
        print(f"[turret] Axis {axis} cleared switch after {backoff_steps} backoff steps")

    if axis == "X":
        _set_position(x=0)
    elif axis == "Y":
        _set_position(y=0)

    print(f"[turret] Axis {axis} homed and zeroed (off switch)")


@_safe
def home_all():
    """Home both axes and zero their step positions."""
    print("[turret] Home all axes requested")

    if _estop_pressed():
        print("[turret] home_all aborted: E-STOP pressed")
        return

    _home_single_axis("X", STEP_X_PIN, DIR_X_PIN, LIM_X_PIN)
    _home_single_axis("Y", STEP_Y_PIN, DIR_Y_PIN, LIM_Y_PIN)

    print("[turret] Home all complete")
    # After homing, define this as position 0,0
    _set_current_position_steps(0, 0)

# ---------------- JOGGING / SOFT TRAVEL LIMITS ----------------

@_safe
def calibrate_all():
    """
    Wrapper used by UI 'Calibrate' button.

    - Homes both axes (home_all).
    - Resets our logical position to 0,0 in both the CUR_* and
      internal _POS_* counters.
    - Does NOT change any existing soft-limit maxima; use the
      travel calibration dialog for that.
    """
    print("[turret] Calibrate requested")
    home_all()
    _set_current_position_steps(0, 0)


@_safe
def begin_travel_calibration():
    """
    Step 1 of 'Rotation / Travel Calibration'.

    Called when the user presses 'Home & Unlock Travel' in the UI.
    """
    global SOFT_LIMITS

    print("[turret] begin_travel_calibration: homing + clear soft maxima")
    home_all()
    _set_current_position_steps(0, 0)

    # Home is always 0,0
    SOFT_LIMITS["x_min"] = 0
    SOFT_LIMITS["y_min"] = 0
    # Clear max so the user can jog out to new bounds
    SOFT_LIMITS["x_max"] = None
    SOFT_LIMITS["y_max"] = None
    _save_soft_limits()


@_safe
def finalize_travel_calibration():
    """
    Step 2 of 'Rotation / Travel Calibration'.

    Whatever step-based position the turret is currently at becomes
    the software max for each axis.
    """
    global SOFT_LIMITS

    with _STATE_LOCK:
        cur_x = _POS_X_STEPS
        cur_y = _POS_Y_STEPS

    x_min = SOFT_LIMITS.get("x_min", 0) or 0
    y_min = SOFT_LIMITS.get("y_min", 0) or 0

    SOFT_LIMITS["x_max"] = max(x_min, cur_x)
    SOFT_LIMITS["y_max"] = max(y_min, cur_y)

    _save_soft_limits()
    print(
        "[turret] finalize_travel_calibration: "
        f"x range = [{SOFT_LIMITS['x_min']}, {SOFT_LIMITS['x_max']}], "
        f"y range = [{SOFT_LIMITS['y_min']}, {SOFT_LIMITS['y_max']}]"
    )


def _apply_soft_limits_to_delta(x_steps: int, y_steps: int) -> Tuple[int, int]:
    """
    Clamp a requested delta (x_steps, y_steps) so that the RESULTING
    absolute position stays within SOFT_LIMITS.

    Returns (clamped_x_steps, clamped_y_steps).
    """
    global SOFT_LIMITS

    with _STATE_LOCK:
        cur_x = _POS_X_STEPS
        cur_y = _POS_Y_STEPS

    x_min = SOFT_LIMITS.get("x_min")
    x_max = SOFT_LIMITS.get("x_max")
    y_min = SOFT_LIMITS.get("y_min")
    y_max = SOFT_LIMITS.get("y_max")

    target_x = cur_x + int(x_steps)
    target_y = cur_y + int(y_steps)

    # Clamp X
    if x_min is not None and target_x < x_min:
        target_x = x_min
    if x_max is not None and target_x > x_max:
        target_x = x_max

    # Clamp Y
    if y_min is not None and target_y < y_min:
        target_y = y_min
    if y_max is not None and target_y > y_max:
        target_y = y_max

    clamped_x = target_x - cur_x
    clamped_y = target_y - cur_y
    return clamped_x, clamped_y


@_safe
def jog_xy(
    x_steps: int,
    y_steps: int,
    speed_scale: float = 1.0,
    bypass_limits: bool = False,
):
    """
    Jog the turret by a certain number of STEPS on X and Y.

      Positive steps = DIR_PIN HIGH
      Negative steps = DIR_PIN LOW

    speed_scale multiplies BASE_STEP_DELAY (higher = slower).

    If bypass_limits is False (normal jogs):
      - The requested (x_steps, y_steps) is clamped so the final
        position stays within SOFT_LIMITS (if x_max/y_max are set).

    If bypass_limits is True (used for Predator tracking only):
      - We still respect E-STOP and the physical limit switches,
        but we skip the soft-limit clamping.
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

    # Respect the current motion profile (speed scaling)
    delay_x = BASE_STEP_DELAY * max(
        0.1, min(speed_scale * _MOTION_PROFILE["x_speed_scale"], 10.0)
    )
    delay_y = BASE_STEP_DELAY * max(
        0.1, min(speed_scale * _MOTION_PROFILE["y_speed_scale"], 10.0)
    )

    orig_x, orig_y = int(x_steps), int(y_steps)

    if bypass_limits:
        x_move, y_move = orig_x, orig_y
    else:
        x_move, y_move = _apply_soft_limits_to_delta(orig_x, orig_y)
        if (x_move == 0 and y_move == 0) and (orig_x != 0 or orig_y != 0):
            print(
                "[turret] jog_xy: requested move "
                f"(X={orig_x}, Y={orig_y}) would exceed soft limits → ignored"
            )
            return

    print(
        "[turret] jog_xy: "
        f"requested X={orig_x}, Y={orig_y} → "
        f"move X={x_move}, Y={y_move}, "
        f"delay_x={delay_x:.6f}, delay_y={delay_y:.6f}, "
        f"bypass_limits={bypass_limits}"
    )

    if x_move != 0:
        _move_axis_with_pos("X", STEP_X_PIN, DIR_X_PIN, x_move, delay_x)
    if y_move != 0:
        _move_axis_with_pos("Y", STEP_Y_PIN, DIR_Y_PIN, y_move, delay_y)

    # Keep CUR_* in sync for debug
    with _STATE_LOCK:
        global CUR_X_STEPS, CUR_Y_STEPS
        CUR_X_STEPS = _POS_X_STEPS
        CUR_Y_STEPS = _POS_Y_STEPS


@_safe
def jog(axis: str, direction: int, step_deg: float, speed: float):
    """
    UI hook for manual jog buttons (Settings / Calibration dialogs).

    We map degrees -> steps using JOG_STEPS_PER_DEG, enforce a minimum
    of DEFAULT_JOG_STEPS, and send to jog_xy() (with soft limits).
    """
    axis = (axis or "").upper()
    direction = 1 if direction >= 0 else -1

    try:
        step_deg = float(step_deg)
    except Exception:
        step_deg = 2.0

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

    try:
        speed = float(speed)
    except Exception:
        speed = 1000.0

    base_speed = 1200.0
    speed_scale = base_speed / max(1.0, speed)

    print(
        f"[turret] jog: axis={axis}, dir={direction}, "
        f"steps=({x_steps},{y_steps}), step_deg={step_deg}, "
        f"speed={speed}, scale={speed_scale:.3f}"
    )

    # Normal jogs always respect soft limits
    jog_xy(x_steps, y_steps, speed_scale=speed_scale, bypass_limits=False)


# ---------------- MOTION PROFILE ----------------

@_safe
def set_motion_profile(x_speed_scale: float = None, y_speed_scale: float = None):
    """Adjust the motion profile from the Settings menu."""
    if x_speed_scale is not None:
        _MOTION_PROFILE["x_speed_scale"] = max(0.1, min(float(x_speed_scale), 10.0))
    if y_speed_scale is not None:
        _MOTION_PROFILE["y_speed_scale"] = max(0.1, min(float(y_speed_scale), 10.0))

    print("[turret] Motion profile updated:", _MOTION_PROFILE)


@_safe
def set_motor_speeds(x_speed: float, y_speed: float):
    """
    UI hook: map user-friendly speed values into our motion profile.

    Higher x_speed/y_speed -> faster motion (smaller delay).
    """
    try:
        x_speed = float(x_speed)
        y_speed = float(y_speed)
    except Exception:
        print("[turret] set_motor_speeds: invalid values, keeping previous profile")
        return

    x_speed = max(1.0, x_speed)
    y_speed = max(1.0, y_speed)

    base_speed = 1200.0
    x_scale = base_speed / x_speed
    y_scale = base_speed / y_speed

    set_motion_profile(x_speed_scale=x_scale, y_speed_scale=y_scale)
    print(
        "[turret] set_motor_speeds:",
        {
            "x_speed": x_speed,
            "y_speed": y_speed,
            "x_scale": x_scale,
            "y_scale": y_scale,
        },
    )


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


# ---------------- SESSION / PAINT SUPPORT ----------------

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
    """
    print("[turret] start_paint_from_image: received job:")
    print(repr(job)[:1000], "...")


def _move_to_normalized(xn: float, yn: float):
    """
    Map normalized image coords (0..1) to turret steps
    relative to the stored forward reference.

    Behaviour:
      - (0.5, 0.5)  -> exactly the forward reference pose.
      - Other coords are symmetric offsets around that point.
    """
    # Clamp inputs to [0, 1]
    xn = max(0.0, min(1.0, float(xn)))
    yn = max(0.0, min(1.0, float(yn)))

    # Offset from image center [-0.5 .. +0.5]
    dx_norm = xn - 0.5
    dy_norm = yn - 0.5

    # Scale to steps (small safe range)
    MAX_OFFSET_STEPS_X = 800  # tweak once you see real geometry
    MAX_OFFSET_STEPS_Y = 800

    x_offset = int(dx_norm * 2.0 * MAX_OFFSET_STEPS_X)
    y_offset = int(dy_norm * 2.0 * MAX_OFFSET_STEPS_Y)

    # Read current and forward positions atomically
    with _STATE_LOCK:
        base_x = _FWD_X_STEPS
        base_y = _FWD_Y_STEPS
        cur_x = _POS_X_STEPS
        cur_y = _POS_Y_STEPS

    # Target absolute position in steps
    target_x = base_x + x_offset
    target_y = base_y + y_offset

    # Convert to relative move from current pose
    dx_steps = target_x - cur_x
    dy_steps = target_y - cur_y

    print(
        "[turret] _move_to_normalized: "
        f"xn={xn:.3f}, yn={yn:.3f} -> "
        f"base=({base_x},{base_y}), "
        f"target=({target_x},{target_y}), "
        f"delta_steps=({dx_steps},{dy_steps})"
    )

    if dx_steps != 0 or dy_steps != 0:
        jog_xy(dx_steps, dy_steps, speed_scale=1.0)


# ---------------- PAINT JOB EXECUTION + ABORT ----------------

@_safe
def request_abort():
    """
    UI hook: ask the current paint job (if any) to abort as soon as possible.

    The run_paint_job / run_paint_pass loops check this flag and exit cleanly.
    """
    global _ABORT_JOB
    _ABORT_JOB = True
    print("[turret] ABORT requested by UI")


@_safe
def run_paint_job(job: dict):
    """
    Execute a paint job defined by the UI.

    Expected job schema:
      {
        "mode": "simple" | "complex",
        "source_image": "/path/to/file",
        "passes": [
          {
            "label": "Outline",
            "points": [(x_norm, y_norm), ...],  # 0..1 normalized coords
            "color": "#rrggbb",
          },
          ...
        ]
      }
    """
    global _ABORT_JOB
    _ABORT_JOB = False  # clear any previous abort

    if not job:
        print("[turret] run_paint_job: empty job")
        return

    passes = job.get("passes") or []
    mode = job.get("mode")
    src = job.get("source_image")

    print(
        "[turret] run_paint_job: starting",
        {"mode": mode, "source_image": src, "num_passes": len(passes)},
    )

    if not passes:
        print("[turret] run_paint_job: no passes to execute")
        return

    if not _ensure_gpio():
        print("[turret] run_paint_job: aborted (GPIO not ready)")
        return

    if _estop_pressed():
        print("[turret] run_paint_job: aborted (E-STOP pressed at start)")
        return

    limits = _limits_ok()
    if not (limits.get("x_limit_ok") and limits.get("y_limit_ok")):
        print("[turret] run_paint_job: aborted (limit switch not OK)", limits)
        return

    print("[turret] run_paint_job: moving to forward reference before painting")
    goto_forward()

    start_paint_session()
    try:
        for idx, p in enumerate(passes):
            if _ABORT_JOB:
                print(f"[turret] run_paint_job: ABORTED before pass {idx}")
                return

            label = p.get("label", f"Pass {idx}")
            pts = p.get("points") or []
            color = p.get("color", "#ffffff")

            print(
                f"[turret] run_paint_job: pass {idx}: "
                f"label={label!r}, points={len(pts)}, color={color}"
            )

            if not pts:
                continue

            for i, point in enumerate(pts):
                if _ABORT_JOB:
                    print(
                        f"[turret] run_paint_job: ABORTED at pass {idx}, "
                        f"point {i}/{len(pts)}"
                    )
                    return

                if _estop_pressed():
                    print(
                        f"[turret] run_paint_job: ABORTED by E-STOP "
                        f"during pass {idx} point {i}/{len(pts)}"
                    )
                    return

                try:
                    xn, yn = point
                except Exception:
                    print(
                        f"[turret] run_paint_job: invalid point at "
                        f"pass {idx} index {i}: {point!r}"
                    )
                    continue

                _move_to_normalized(xn, yn)
                manual_fire()
                time.sleep(0.03)

        print("[turret] run_paint_job: completed all passes")

    finally:
        end_paint_session()


@_safe
def run_paint_pass(job: dict, pass_index: int = 0):
    """
    Execute a single paint pass from a prepared job.

    Mostly used for debugging; run_paint_job() is the main entry.
    """
    global _ABORT_JOB
    _ABORT_JOB = False

    if not job:
        print("[turret] run_paint_pass: empty job")
        return

    passes = job.get("passes") or []
    if not passes:
        print("[turret] run_paint_pass: no passes")
        return

    if pass_index < 0 or pass_index >= len(passes):
        print(f"[turret] run_paint_pass: invalid pass_index {pass_index}")
        return

    p = passes[pass_index]
    pts = p.get("points") or []
    label = p.get("label", f"Pass {pass_index}")
    color = p.get("color", "#ffffff")

    print(
        f"[turret] run_paint_pass: index={pass_index}, "
        f"label={label!r}, points={len(pts)}, color={color}"
    )

    if not _ensure_gpio():
        print("[turret] run_paint_pass: aborted (GPIO not ready)")
        return

    if _estop_pressed():
        print("[turret] run_paint_pass: aborted (E-STOP pressed at start)")
        return

    limits = _limits_ok()
    if not (limits.get("x_limit_ok") and limits.get("y_limit_ok")):
        print("[turret] run_paint_pass: aborted (limit switch not OK)", limits)
        return

    print("[turret] run_paint_pass: moving to forward reference before painting")
    goto_forward()

    start_paint_session()
    try:
        for i, point in enumerate(pts):
            if _ABORT_JOB:
                print(f"[turret] run_paint_pass: ABORTED at point {i}/{len(pts)}")
                return

            if _estop_pressed():
                print(
                    f"[turret] run_paint_pass: ABORTED by E-STOP "
                    f"at point {i}/{len(pts)}"
                )
                return

            try:
                xn, yn = point
            except Exception:
                print(
                    f"[turret] run_paint_pass: invalid point at index {i}: "
                    f"{point!r}"
                )
                continue

            _move_to_normalized(xn, yn)
            manual_fire()
            time.sleep(0.03)
    finally:
        end_paint_session()

# ---------------- TRACKING / SENTRY FLAGS ----------------
# Tunable parameters for Predator tracking
TRACK_DEADZONE = 0.05     # 5% of frame; reduces jitter
TRACK_GAIN_X   = 150.0    # gentler tracking
TRACK_GAIN_Y   = 150.0
TRACK_MIN_STEP = 5        # ignore small jitter

# If X or Y moves the wrong way on your hardware, set that axis to -1
TRACK_INVERT_X = 1        # set to -1 to flip left/right
TRACK_INVERT_Y = 1        # set to -1 to flip up/down

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
    UI hook: enable/disable Predator sentry mode.

    This just flips the internal _SENTRY_ENABLED flag so:
      - get_status() can report it to the UI
      - other backend logic can optionally gate behavior on it
    """
    global _SENTRY_ENABLED
    _SENTRY_ENABLED = bool(enabled)
    print("[turret] Sentry mode enabled:", _SENTRY_ENABLED)


@_safe
def set_tracking_inversion(invert_x: bool = None, invert_y: bool = None):
    """
    UI hook: flip left/right or up/down tracking direction.

    invert_x / invert_y are booleans:
      True  -> axis is inverted (multiplier = -1)
      False -> normal (multiplier = +1)
      None  -> leave axis unchanged
    """
    global TRACK_INVERT_X, TRACK_INVERT_Y

    if invert_x is not None:
        TRACK_INVERT_X = -1 if invert_x else 1
    if invert_y is not None:
        TRACK_INVERT_Y = -1 if invert_y else 1

    # Persist to disk so UI choices survive reboot
    _save_tracking_inversion(
        invert_x=(TRACK_INVERT_X < 0),
        invert_y=(TRACK_INVERT_Y < 0),
    )

    print(
        "[turret] set_tracking_inversion:",
        {
            "invert_x": invert_x,
            "invert_y": invert_y,
            "TRACK_INVERT_X": TRACK_INVERT_X,
            "TRACK_INVERT_Y": TRACK_INVERT_Y,
        },
    )

@_safe
def sentry_scan_step(direction: int):
    """
    Small horizontal sweep step for Predator sentry mode.

    direction: +1 (right) or -1 (left).
    """
    direction = 1 if direction >= 0 else -1
    steps = direction * max(5, DEFAULT_JOG_STEPS // 4)
    print(f"[turret] sentry_scan_step: direction={direction}, steps={steps}")
    jog_xy(steps, 0, speed_scale=1.5)  # slightly slower than base


@_safe
def sentry_fire_at(x_norm: float, y_norm: float):
    """
    Called by UI predator mode when it has a target at (x_norm, y_norm) in [0..1].

    Behaviour:
      - Uses the target to nudge the turret toward the center crosshair
        if tracking is enabled.
      - Only fires when AUTOFIRE_ENABLED is True.
    """
    global _TRACKING_ENABLED, _AUTOFIRE_ENABLED

    # Clamp and normalize inputs
    try:
        xn = max(0.0, min(1.0, float(x_norm)))
        yn = max(0.0, min(1.0, float(y_norm)))
    except Exception:
        print(f"[turret] sentry_fire_at: invalid coords ({x_norm!r}, {y_norm!r})")
        return

    print(f"[turret] sentry_fire_at: target at ({xn:.3f}, {yn:.3f})")

    # 1) If tracking is disabled, ignore target completely.
    if not _TRACKING_ENABLED:
        print("[turret] sentry_fire_at: tracking disabled → ignoring target")
        return

    # 2) Compute normalized error from center of the image
    cx, cy = 0.5, 0.5
    dx = xn - cx   # >0 → target is to the RIGHT of center
    dy = yn - cy   # >0 → target is BELOW center (in image coords)

    step_x = 0
    step_y = 0

    # Use global tunables defined near the tracking flags
    if abs(dx) > TRACK_DEADZONE:
        # TRACK_INVERT_* lets you flip an axis if geometry is reversed
        step_x = int(round(dx * TRACK_GAIN_X * TRACK_INVERT_X))

    if abs(dy) > TRACK_DEADZONE:
        # dy > 0 means target is below center in image; invert once for turret Y
        step_y = int(round(-dy * TRACK_GAIN_Y * TRACK_INVERT_Y))

    # Filter out tiny twitches
    if abs(step_x) < TRACK_MIN_STEP:
        step_x = 0
    if abs(step_y) < TRACK_MIN_STEP:
        step_y = 0

    # 3) Apply a small tracking jog toward the target
    if step_x != 0 or step_y != 0:
        print(f"[turret] sentry_fire_at: tracking jog X={step_x}, Y={step_y}")
        # Here we bypass SOFT_LIMITS, but still honour E-STOP and
        # Respect SOFT_LIMITS while tracking toward the target.
        jog_xy(step_x, step_y, speed_scale=1.2, bypass_limits=False)

    # 4) Firing is gated by AUTOFIRE_ENABLED
    if not _AUTOFIRE_ENABLED:
        print("[turret] sentry_fire_at: autofire disabled → not firing")
        return

    # Extra sanity before firing: respect E-STOP / GPIO
    if not _ensure_gpio():
        print("[turret] sentry_fire_at: GPIO not ready → not firing")
        return
    if _estop_pressed():
        print("[turret] sentry_fire_at: E-STOP pressed → not firing")
        return

    print("[turret] sentry_fire_at: AUTOFIRE_ENABLED & tracking → firing")
    manual_fire()


# ---------------- MODULE IMPORT SIDE EFFECT ----------------
# ---------------- SELF TEST (CLI USE ONLY) ----------------

@_safe
def run_self_test():
    """
    Simple hardware/backend sanity test.

    Run from shell with:
        python3 vpp_turret_control.py

    It will:
      - Print status
      - Snapshot raw GPIO
      - Try a short fire pulse
      - Try a tiny X jog (if E-STOP and limits allow)
    """
    print("[turret] === SELF TEST START ===")

    # 1) High-level status
    status = get_status()
    print("[turret] status:", status)

    # 2) Raw GPIO snapshot
    debug_gpio_snapshot()

    # 3) Check GPIO before doing anything motion-related
    if not _ensure_gpio():
        print("[turret] SELF TEST: GPIO not ready, skipping motion tests")
        print("[turret] === SELF TEST END ===")
        return

    # 4) If E-STOP is pressed, do not move or fire
    if _estop_pressed():
        print("[turret] SELF TEST: E-STOP appears pressed, skipping motion/fire")
        print("[turret] === SELF TEST END ===")
        return

    # 5) Try a short fire pulse (marker/relay)
    print("[turret] SELF TEST: firing test pulse")
    test_fire()

    # 6) Try a tiny jog on X (forward then back), if limits OK
    limits = _limits_ok()
    print("[turret] SELF TEST: limits:", limits)
    if limits.get("x_limit_ok") and limits.get("y_limit_ok"):
        print("[turret] SELF TEST: jogging X +1deg then -1deg")
        jog("X", 1, 1.0, 1000.0)
        jog("X", -1, 1.0, 1000.0)
    else:
        print("[turret] SELF TEST: skipping jog, a limit is not OK")

    print("[turret] === SELF TEST END ===")


if __name__ == "__main__":
    # Allow quick hardware sanity checks by running this module directly.
    run_self_test()

# ---------------- MODULE IMPORT SIDE EFFECT ----------------
print("[turret] vpp_turret_control module loaded (GPIO will init on first use)")

