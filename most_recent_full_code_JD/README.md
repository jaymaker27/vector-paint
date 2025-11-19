# Vector Projectile Painting (VPP) — Most Recent Full Code (JD)

This folder contains the current full working code for the **Vector Projectile Painting** project
as of the latest JD build.

## Contents

- `vpp_ui_main.py`  
  Tkinter-based fullscreen UI for:
  - Predator Sentry Mode (camera-based target detection + auto-fire)
  - Image-based painting modes (simple outline and multi-color segmentation)
  - Motor & Control Settings (speeds, jog, fire, home)
  - Aim / Camera Center Calibration window

- `vpp_turret_control.py`  
  Backend / hardware control for the turret:
  - Stepper control for X and Y axes
  - Homing using NC limit switches
  - E-STOP monitoring
  - Motion profile and jog functions
  - Manual fire / test fire
  - Predator sentry hooks (`set_sentry_mode`, `sentry_scan_step`, `sentry_fire_at`)
  - Aim center storage (`set_current_as_forward`, `goto_forward`)

## Hardware Overview

- Platform: Raspberry Pi (64-bit Bookworm)
- GPIO pin mapping (BCM):

  - X Axis (Motor 1):
    - `GPIO 23` — PUL (step)
    - `GPIO 24` — DIR

  - Y Axis (Motor 2):
    - `GPIO 20` — PUL (step)
    - `GPIO 21` — DIR

  - Limit / Home switches (NC → GND, internal pull-up):
    - `GPIO 17` — X limit / home
    - `GPIO 27` — Y limit / home

  - Safety:
    - `GPIO 25` — E-STOP (NO, closes to GND when pressed, pull-up enabled)

  - Fire / Marker:
    - `GPIO 18` — Fire / relay / marker output

## Software Requirements (Pi)

From a fresh Raspberry Pi OS (Bookworm):

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3-tk libatlas-base-dev \
                    python3-picamera2 python3-opencv

