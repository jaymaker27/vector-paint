#!/usr/bin/env python3
"""
Vector Projectile Painting (VPP) — Main Tkinter UI

Features
--------
- Fullscreen neon UI
- Top status bar: E-STOP, X / Y limits, SAFE_MODE (polled from turret backend)
- Left sidebar:
    - Calibrate (homes using limit switches)
    - Test Fire (single shot)
    - Start Painting (image → segmentation → preview → approve)
    - Cancel Paint (tells backend to abort current job)
    - Predator Sentry Mode (sweeping, tracking, auto-fire hooks)
    - Settings… (motor speeds + manual jog + fire + home)
    - Quit
- Start Painting:
    1. Shows live camera preview
    2. Lets user pick an image file
    3. Choose Simple vs Complex:
        * Simple: 4 outline variants + detail slider
        * Complex: multi-colour segmentation into up to 5 passes
    4. Preview simulated paint on main canvas
    5. Choose paint color for each stage
    6. Confirm → calls backend hooks safely
- Settings dialog:
    - Editable X / Y speed (units are arbitrary for backend)
    - Jog X+/X-/Y+/Y-, Manual Fire, Home All
- Predator Sentry Mode:
    - Uses main canvas to show live camera feed
    - Sweeps left/right (backend hook)
    - Very simple “target” detection via brightness contrast + motion-ish cue
    - Draws boxes over targets and calls backend auto-fire hook

Backend expectations (all OPTIONAL)
-----------------------------------
In vpp_turret_control.py you MAY define any of:

    def calibrate_all(): ...
    def test_fire(): ...
    def set_tracking_enabled(enabled: bool): ...
    def set_autofire_enabled(enabled: bool): ...
    def set_sentry_mode(enabled: bool): ...
    def sentry_scan_step(direction: int): ...
    def sentry_fire_at(x_norm: float, y_norm: float): ...
    def run_paint_job(job: dict): ...
    def start_paint_from_image(job: dict): ...
    def get_status() -> dict:   # keys: estop (bool), x_limit_ok, y_limit_ok, safe_mode
    def shutdown(): ...
    def jog(axis: str, direction: int, step_deg: float, speed: float): ...
    def set_motor_speeds(x_speed: float, y_speed: float): ...
    def home_all(): ...
    def manual_fire(): ...
    def goto_forward(): ...
    def set_current_as_forward(): ...
    def request_abort(): ...
    def start_travel_calibration(): ...
    def set_current_as_x_max(): ...
    def set_current_as_y_max(): ...
    def finish_travel_calibration(): ...

If a function is missing, the UI will just print a message and keep running.
"""

import sys
import threading
import time
import random
import math
from pathlib import Path

import tkinter as tk
from tkinter import *
from tkinter import ttk, messagebox, filedialog
import cv2

# Pillow for image work / previews
try:
    from PIL import Image, ImageTk, ImageFilter
except Exception as e:  # pillow not present
    print("WARNING: Pillow (PIL) not available, image functions will be limited:", e, file=sys.stderr)
    Image = None
    ImageTk = None
    ImageFilter = None

# Picamera2 for live camera view
try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None

# Turret backend (safe optional)
try:
    import vpp_turret_control as turret
except Exception as e:
    print("WARNING: could not import vpp_turret_control:", e, file=sys.stderr)
    turret = None


def safe_call(name, *args, **kwargs):
    """
    Call turret.<name>(*args, **kwargs) if it exists.
    Catch all exceptions and never crash the UI.
    """
    if turret is None:
        print(f"[turret] (module missing) {name} not called")
        return
    func = getattr(turret, name, None)
    if not callable(func):
        print(f"[turret] {name} not present, skipping")
        return
    try:
        return func(*args, **kwargs)
    except Exception as e:
        print(f"[turret] error in {name}: {e}", file=sys.stderr)


# ---------------------------------------------------------------------
# MAIN UI CLASS
# ---------------------------------------------------------------------

class VPPApp:
    STATUS_INTERVAL = 0.1  # seconds
    PREDATOR_INTERVAL_MS = 40

    # Predator motion detection tuning
    PREDATOR_MOTION_THRESH = 25        # pixel difference threshold (0–255)
    PREDATOR_MIN_CONTOUR_AREA = 600    # ignore tiny moving specks
    PREDATOR_LOST_FRAMES = 15          # frames with no motion before sweeping

    def __init__(self, root: Tk):
        self.root = root
        self.root.title("Vector Projectile Painting UI")

        # Fullscreen
        self.root.update_idletasks()
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.root.geometry(f"{sw}x{sh}+0+0")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#050a0f")
        self.root.bind("<Escape>", lambda e: self.on_quit())

        # Shared state
        self.canvas_w = 800
        self.canvas_h = 480

        self.tracking_var = BooleanVar(value=False)
        self.autofire_var = BooleanVar(value=False)
        self.sentry_var = BooleanVar(value=False)

        # Tracking inversion (used by Tracking / Predator Settings dialog)
        self.track_invert_x_var = BooleanVar(value=False)
        self.track_invert_y_var = BooleanVar(value=False)

        self.current_image_path = None
        self.current_job = None

        # Camera state
        self.picam = None
        self._camera_mode = "off"  # "off" | "preview" | "predator"
        self._camera_after_id = None
        self._camera_win = None    # for preview window
        self._camera_label = None

        # Predator / sentry state
        self._predator_prev_gray = None
        self._predator_scan_dir = 1  # +1 right, -1 left
        self._predator_lock_frames = 0
        self._predator_shot_cooldown = 0.0
        self._predator_no_motion_frames = 0  # how long since we last saw motion
        # Auto-fire is now a global setting (shared with Settings dialog)

        # Settings
        self.settings = {
            "x_speed": 1200.0,
            "y_speed": 1200.0,
            "jog_step_deg": 2.0,
        }

        # Build UI layout
        self._build_layout()

        # Status polling
        self._status_running = True
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()

        # Initial backdrop
        self.redraw_scene()

    # ----- UI Layout --------------------------------------------------

    def _build_layout(self):
        self.main_frame = Frame(self.root, bg="#050a0f")
        self.main_frame.pack(fill=BOTH, expand=True)

        menubar = Menu(self.root, tearoff=False)
        settings_menu = Menu(menubar, tearoff=False)
        # Motor / control settings
        settings_menu.add_command(
            label="Motor / Control Settings…",
            command=self.open_settings_dialog
        )

        # Tracking / Predator settings (axis inversion, etc.)
        settings_menu.add_command(
            label="Tracking / Predator Settings…",
            command=self.open_tracking_settings_dialog
        )

        # Calibration submenu under Settings
        calib_menu = Menu(settings_menu, tearoff=False)

        calib_menu.add_command(
            label="Aim / Camera Center…",
            command=self.open_aim_calibration
        )
        calib_menu.add_command(
            label="Rotation / Travel Calibration…",
            command=self.open_travel_calibration
        )
        settings_menu.add_cascade(label="Calibration", menu=calib_menu)

        menubar.add_cascade(label="Settings", menu=settings_menu)
        self.root.config(menu=menubar)

        # Top status bar
        self._build_status_bar()

        body = Frame(self.main_frame, bg="#050a0f")
        body.pack(fill=BOTH, expand=True)

        self._build_sidebar(body)
        self._build_center(body)

    def _build_status_bar(self):
        bar = Frame(self.main_frame, bg="#050a0f")
        bar.pack(side=TOP, fill=X, pady=(4, 0))

        self.estop_label = Label(
            bar, text="E-STOP: UNKNOWN",
            font=("Segoe UI", 12, "bold"),
            bg="#444444", fg="#eeeeee",
            padx=12, pady=4
        )
        self.estop_label.pack(side=LEFT, padx=(8, 4))

        self.xlimit_label = Label(
            bar, text="X LIMIT: ?",
            font=("Segoe UI", 12, "bold"),
            bg="#004422", fg="#ccffdd",
            padx=12, pady=4
        )
        self.xlimit_label.pack(side=LEFT, padx=4)

        self.ylimit_label = Label(
            bar, text="Y LIMIT: ?",
            font=("Segoe UI", 12, "bold"),
            bg="#004422", fg="#ccffdd",
            padx=12, pady=4
        )
        self.ylimit_label.pack(side=LEFT, padx=4)

        self.safe_label = Label(
            bar, text="SAFE_MODE: ?",
            font=("Segoe UI", 12, "bold"),
            bg="#002244", fg="#cce6ff",
            padx=12, pady=4
        )
        self.safe_label.pack(side=LEFT, padx=4)

    def _build_sidebar(self, parent):
        sidebar = Frame(parent, bg="#050a0f")
        sidebar.pack(side=LEFT, fill=Y, padx=(12, 8), pady=(8, 12))

        title = Label(
            sidebar, text="VPP CONTROL",
            font=("Segoe UI", 18, "bold"),
            fg="#00ffcc", bg="#050a0f"
        )
        title.pack(pady=(4, 16), anchor="w")

        btn_style = dict(
            font=("Segoe UI", 16, "bold"),
            fg="#002222",
            bg="#00ffcc",
            activebackground="#33ffd9",
            activeforeground="#001111",
            relief=FLAT,
            padx=20, pady=10
        )

        self.calib_btn = Button(
            sidebar, text="Calibrate",
            command=self.on_calibrate,
            **btn_style
        )
        self.calib_btn.pack(fill=X, pady=6)

        self.test_btn = Button(
            sidebar, text="Test Fire",
            command=self.on_test_fire,
            **btn_style
        )
        self.test_btn.pack(fill=X, pady=6)

        self.paint_btn = Button(
            sidebar, text="Start Painting",
            command=self.on_start_paint,
            **btn_style
        )
        self.paint_btn.pack(fill=X, pady=6)

        # Cancel Paint button -> request_abort()
        self.cancel_btn = Button(
            sidebar, text="Cancel Paint",
            command=self.on_cancel_paint,
            font=("Segoe UI", 14, "bold"),
            fg="#ffdddd",
            bg="#993333",
            activebackground="#cc4444",
            activeforeground="#ffffff",
            relief=FLAT,
            padx=20, pady=8
        )
        self.cancel_btn.pack(fill=X, pady=6)

        # Predator sentry
        self.sentry_btn = Button(
            sidebar, text="Predator Sentry: OFF",
            command=self.on_toggle_sentry,
            font=("Segoe UI", 14, "bold"),
            fg="#ffeeee",
            bg="#550000",
            activebackground="#aa0000",
            activeforeground="#ffffff",
            relief=FLAT,
            padx=16, pady=8
        )
        self.sentry_btn.pack(fill=X, pady=(10, 6))

        # Spacer
        Frame(sidebar, height=16, bg="#050a0f").pack()

        track_title = Label(
            sidebar, text="AUTO TRACK",
            font=("Segoe UI", 14, "bold"),
            fg="#00ffcc", bg="#050a0f"
        )
        track_title.pack(anchor="w", pady=(8, 4))

        self.track_chk = Checkbutton(
            sidebar,
            text="Enable tracking",
            variable=self.tracking_var,
            command=self.on_toggle_tracking,
            font=("Segoe UI", 12),
            fg="#e0ffff",
            bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#003333"
        )
        self.track_chk.pack(anchor="w", pady=2)

        self.autofire_chk = Checkbutton(
            sidebar,
            text="Auto fire on target",
            variable=self.autofire_var,
            command=self.on_toggle_autofire,
            font=("Segoe UI", 12),
            fg="#e0ffff",
            bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#330000"
        )
        self.autofire_chk.pack(anchor="w", pady=2)

        # Settings button (mirrors menu)
        Frame(sidebar, height=16, bg="#050a0f").pack()
        Button(
            sidebar,
            text="Settings…",
            command=self.open_settings_dialog,
            font=("Segoe UI", 12, "bold"),
            fg="#002222", bg="#00bbee",
            activebackground="#33ccff",
            activeforeground="#001111",
            relief=FLAT,
            padx=16, pady=6
        ).pack(fill=X, pady=4)

        Frame(sidebar, bg="#050a0f").pack(expand=True, fill=BOTH)

        quit_btn = Button(
            sidebar,
            text="Quit",
            command=self.on_quit,
            font=("Segoe UI", 12, "bold"),
            fg="#ffdddd",
            bg="#550000",
            activebackground="#aa0000",
            activeforeground="#ffffff",
            relief=FLAT,
            padx=10, pady=6
        )
        quit_btn.pack(side=BOTTOM, fill=X, pady=(10, 0))

    def _build_center(self, parent):
        center = Frame(parent, bg="#050a0f")
        center.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 8), pady=(8, 12))

        title = Label(
            center, text="VECTOR PROJECTILE PAINTING",
            font=("Segoe UI", 26, "bold"),
            fg="#00ffcc", bg="#050a0f"
        )
        title.pack(pady=(4, 0))

        subtitle = Label(
            center, text="Auto-tracking paint turret",
            font=("Segoe UI", 14),
            fg="#88ffee", bg="#050a0f"
        )
        subtitle.pack(pady=(0, 8))

        self.canvas = Canvas(
            center,
            bg="#050a0f",
            highlightthickness=0
        )
        self.canvas.pack(fill=BOTH, expand=True, padx=20, pady=(8, 10))
        self.canvas.bind("<Configure>", self.on_canvas_resize)

    # ----- Canvas / backdrop -----------------------------------------

    def on_canvas_resize(self, event):
        self.canvas_w = max(50, event.width)
        self.canvas_h = max(50, event.height)
        self.redraw_scene()

    def redraw_scene(self):
        """Static backdrop: dark vignette + turret outline."""
        self.canvas.delete("all")
        self.canvas.create_rectangle(
            0, 0, self.canvas_w, self.canvas_h,
            fill="#050a0f", outline=""
        )
        self.draw_turret()

    def draw_turret(self):
        w, h = self.canvas_w, self.canvas_h
        base_y = h - 60
        base_x = 120

        # Base
        self.canvas.create_oval(
            base_x - 50, base_y - 12,
            base_x + 50, base_y + 12,
            fill="#101820", outline="#00ffcc", width=2
        )

        # Body
        self.canvas.create_rectangle(
            base_x - 22, base_y - 70,
            base_x + 50, base_y - 40,
            fill="#111822", outline="#00ffcc", width=2
        )

        # Barrel
        self.canvas.create_rectangle(
            base_x + 50, base_y - 60,
            base_x + 150, base_y - 48,
            fill="#081018", outline="#00ffcc", width=2
        )

        # Hopper
        self.canvas.create_polygon(
            base_x - 8, base_y - 80,
            base_x + 12, base_y - 112,
            base_x + 40, base_y - 102,
            base_x + 18, base_y - 76,
            fill="#182830", outline="#00ffcc", width=2
        )

        # Muzzle glow
        self.canvas.create_oval(
            base_x + 140, base_y - 58,
            base_x + 152, base_y - 46,
            outline="#00ffcc", width=2
        )

    # -----------------------------------------------------------------
    # BUTTON HANDLERS
    # -----------------------------------------------------------------

    def on_calibrate(self):
        def worker():
            self.calib_btn.configure(state=DISABLED, text="Calibrating...")
            safe_call("calibrate_all")
            self.calib_btn.configure(state=NORMAL, text="Calibrate")

        threading.Thread(target=worker, daemon=True).start()

    def on_test_fire(self):
        def worker():
            self.test_btn.configure(state=DISABLED, text="Firing...")
            safe_call("test_fire")
            time.sleep(0.4)
            self.test_btn.configure(state=NORMAL, text="Test Fire")

        threading.Thread(target=worker, daemon=True).start()

    def on_cancel_paint(self):
        """Tell backend to abort current paint job (if any)."""
        def worker():
            self.cancel_btn.configure(state=DISABLED, text="Cancelling...")
            safe_call("request_abort")
            time.sleep(0.3)
            self.cancel_btn.configure(state=NORMAL, text="Cancel Paint")

        threading.Thread(target=worker, daemon=True).start()

    # ----- Start Painting flow ---------------------------------------

    def on_start_paint(self):
        # Live camera preview (small window) so user sees gun view
        self.open_camera_preview()

        if Image is None:
            messagebox.showerror(
                "Missing Pillow",
                "Pillow (PIL) is not installed; cannot process images."
            )
            return

        # Ask user to pick an image
        path = filedialog.askopenfilename(
            title="Choose image to paint",
            filetypes=[
                ("Image files", "*.jpg *.jpeg *.png *.bmp *.gif"),
                ("All files", "*.*")
            ]
        )
        if not path:
            return

        self.current_image_path = path

        # Ask for mode: simple vs complex
        choice = self._ask_simple_or_complex()
        if choice == "simple":
            self._simple_outline_flow(path)
        elif choice == "complex":
            self._complex_segmentation_flow(path)

    def _ask_simple_or_complex(self):
        win = Toplevel(self.root)
        win.title("Painting mode")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        choice_var = StringVar(value="simple")

        Label(
            win, text="Select painting complexity",
            font=("Segoe UI", 12, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=6
        ).pack()

        Radiobutton(
            win, text="Simple outline (one color, big blobs, posterized)",
            variable=choice_var, value="simple",
            font=("Segoe UI", 10),
            fg="#e0ffff", bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#003333",
            anchor="w", pady=2
        ).pack(fill=X, padx=10)

        Radiobutton(
            win, text="Complex multi-color (up to 5 passes, more detail)",
            variable=choice_var, value="complex",
            font=("Segoe UI", 10),
            fg="#e0ffff", bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#003333",
            anchor="w", pady=2
        ).pack(fill=X, padx=10)

        result = {"value": None}

        def choose(val):
            result["value"] = val
            win.grab_release()
            win.destroy()

        Button(
            win, text="Continue",
            command=lambda: choose(choice_var.get()),
            font=("Segoe UI", 11, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=14, pady=4
        ).pack(pady=(6, 8))

        win.wait_window()
        return result["value"]

    # ----- Simple mode: 4 variants + detail slider -------------------

    def _simple_outline_flow(self, path):
        win = Toplevel(self.root)
        win.title("Simple outline tuning")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        Label(
            win,
            text="Simple outline setup",
            font=("Segoe UI", 12, "bold"),
            fg="#00ffcc", bg="#050a0f", pady=6
        ).pack()

        variant_var = IntVar(value=2)

        vf = Frame(win, bg="#050a0f")
        vf.pack(pady=(4, 4), padx=6, fill=X)

        Label(
            vf, text="Outline style:",
            font=("Segoe UI", 10, "bold"),
            fg="#a8ffff", bg="#050a0f"
        ).pack(anchor="w")

        variants = [
            (1, "Very chunky / sparse"),
            (2, "Standard outline"),
            (3, "Bold edges / more fill"),
            (4, "High detail / lots of blobs"),
        ]
        for val, text in variants:
            Radiobutton(
                vf, text=text,
                variable=variant_var, value=val,
                font=("Segoe UI", 9),
                fg="#e0ffff", bg="#050a0f",
                activebackground="#050a0f",
                selectcolor="#003333",
                anchor="w"
            ).pack(anchor="w")

        detail_var = IntVar(value=3)

        def detail_to_block(d):
            mapping = {1: 40, 2: 32, 3: 26, 4: 20, 5: 16}
            return mapping.get(int(d), 26)

        block_label = Label(
            win,
            text="Block size: -- px",
            font=("Segoe UI", 10),
            fg="#e0ffff", bg="#050a0f"
        )
        block_label.pack(pady=(6, 2))

        def update_preview(_value=None):
            d = int(detail_var.get())
            block_size = detail_to_block(d)
            block_label.configure(text=f"Block size: {block_size} px")
            try:
                job = self._build_simple_outline_job(
                    path,
                    block_size=block_size,
                    variant=int(variant_var.get())
                )
            except Exception as e:
                print("Simple outline error:", e, file=sys.stderr)
                return
            self.current_job = job
            self._simulate_paint_job(job)

        Scale(
            win,
            from_=1, to=5,
            orient=HORIZONTAL,
            variable=detail_var,
            command=update_preview,
            length=260,
            showvalue=False,
            sliderrelief=FLAT,
            bg="#050a0f",
            troughcolor="#222222",
            highlightthickness=0,
            fg="#e0ffff"
        ).pack(pady=(0, 6))

        Label(
            win,
            text="Coarse blobs  ⟵  Detail slider  ⟶  More pixels",
            font=("Segoe UI", 9),
            fg="#a8ffff", bg="#050a0f"
        ).pack(pady=(0, 6))

        br = Frame(win, bg="#050a0f")
        br.pack(pady=(6, 10))

        def on_confirm():
            if self.current_job is None:
                update_preview()
            if self.current_job is None:
                messagebox.showerror("Error", "Could not build outline job.")
                return
            win.grab_release()
            win.destroy()
            self._configure_pass_colors(self.current_job)

        def on_cancel():
            win.grab_release()
            win.destroy()

        Button(
            br, text="Confirm",
            command=on_confirm,
            font=("Segoe UI", 11, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

        Button(
            br, text="Cancel",
            command=on_cancel,
            font=("Segoe UI", 11),
            bg="#550000", fg="#ffdddd",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

        update_preview()

    def _build_simple_outline_job(self, path, block_size, variant=2):
        img = Image.open(path).convert("RGB")
        img.thumbnail((256, 256), Image.LANCZOS)

        gray = img.convert("L").filter(ImageFilter.GaussianBlur(radius=1.5))
        w, h = gray.size
        pix = gray.load()

        samples = []
        step = max(1, min(w, h) // 32)
        for y in range(0, h, step):
            for x in range(0, w, step):
                samples.append(pix[x, y])
        if samples:
            avg = sum(samples) / len(samples)
            mn = min(samples)
            mx = max(samples)
        else:
            avg, mn, mx = 128, 0, 255

        if variant == 1:
            thresh = (avg + mx) / 2.0
            min_dark_ratio = 0.40
        elif variant == 2:
            thresh = avg
            min_dark_ratio = 0.30
        elif variant == 3:
            thresh = (avg + mn) / 2.0
            min_dark_ratio = 0.20
        else:
            thresh = (avg + mn) / 2.5
            min_dark_ratio = 0.10

        blk = int(block_size)
        coords = []

        for by in range(0, h, blk):
            for bx in range(0, w, blk):
                dark = 0
                total = 0
                for yy in range(by, min(by + blk, h)):
                    for xx in range(bx, min(bx + blk, w)):
                        total += 1
                        if pix[xx, yy] < thresh:
                            dark += 1
                if total == 0:
                    continue
                if dark / total < min_dark_ratio:
                    continue

                cx = bx + min(blk, w - bx) / 2.0
                cy = by + min(blk, h - by) / 2.0
                coords.append((cx / w, cy / h))

        job = {
            "mode": "simple",
            "source_image": path,
            "passes": []
        }
        if coords:
            job["passes"].append({
                "label": "Outline",
                "points": coords,
                "color": "#00ffcc"
            })
        return job

    # ----- Complex segmentation mode ---------------------------------

    def _complex_segmentation_flow(self, path):
        if Image is None:
            return

        img = Image.open(path).convert("RGB")
        img.thumbnail((256, 256), Image.LANCZOS)

        # Quantize to up to 5 colors
        pal_img = img.quantize(colors=5, method=Image.FASTOCTREE)
        pal = pal_img.getpalette()
        pal_rgb = [
            (pal[i], pal[i + 1], pal[i + 2])
            for i in range(0, min(len(pal), 15), 3)
        ]
        qpix = pal_img.load()
        w, h = pal_img.size

        # Build passes: one set of coordinates per palette index
        max_passes = min(5, len(pal_rgb))
        passes = []
        for idx in range(max_passes):
            coords = []
            step = 4
            for y in range(0, h, step):
                for x in range(0, w, step):
                    if qpix[x, y] != idx:
                        continue
                    coords.append((x / w, y / h))
            if not coords:
                continue
            r, g, b = pal_rgb[idx]
            color = f"#{r:02x}{g:02x}{b:02x}"
            passes.append({
                "label": f"Region {idx + 1}",
                "points": coords,
                "color": color,
                "enabled": True,
            })

        job = {
            "mode": "complex",
            "source_image": path,
            "passes": passes
        }
        self.current_job = job
        self._complex_pass_selector(job)

    def _complex_pass_selector(self, job):
        """Let user toggle which regions to paint, then go to color config."""
        passes = job.get("passes") or []
        if not passes:
            messagebox.showerror("No regions", "Could not find any regions to paint.")
            return

        self._simulate_paint_job(job)

        win = Toplevel(self.root)
        win.title("Select regions to paint")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        Label(
            win,
            text="Enable/disable regions (passes)",
            font=("Segoe UI", 12, "bold"),
            fg="#00ffcc", bg="#050a0f", pady=6
        ).pack()

        vars_list = []
        for p in passes:
            var = BooleanVar(value=p.get("enabled", True))
            vars_list.append((p, var))
            row = Frame(win, bg="#050a0f")
            row.pack(fill=X, padx=8, pady=2)

            swatch = Frame(row, bg=p.get("color", "#ffffff"), width=16, height=16)
            swatch.pack(side=LEFT, padx=(0, 6))

            chk = Checkbutton(
                row,
                text=p.get("label", "Region"),
                variable=var,
                font=("Segoe UI", 10),
                fg="#e0ffff",
                bg="#050a0f",
                activebackground="#050a0f",
                selectcolor="#003333",
                anchor="w"
            )
            chk.pack(side=LEFT, fill=X, expand=True)

        def on_confirm():
            enabled_passes = []
            for p, var in vars_list:
                if var.get():
                    p["enabled"] = True
                    enabled_passes.append(p)
                else:
                    p["enabled"] = False
            job["passes"] = enabled_passes
            win.grab_release()
            win.destroy()
            if not enabled_passes:
                messagebox.showerror("No passes selected", "Enable at least one region.")
                return
            self._configure_pass_colors(job)

        def on_cancel():
            win.grab_release()
            win.destroy()

        br = Frame(win, bg="#050a0f")
        br.pack(pady=(6, 10))

        Button(
            br, text="Confirm",
            command=on_confirm,
            font=("Segoe UI", 11, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

        Button(
            br, text="Cancel",
            command=on_cancel,
            font=("Segoe UI", 11),
            bg="#550000", fg="#ffdddd",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

    # ----- Color picker for passes -----------------------------------

    def _configure_pass_colors(self, job):
        passes = job.get("passes") or []
        if not passes:
            job["_colors_configured"] = True
            self._confirm_and_run_job(job)
            return

        win = Toplevel(self.root)
        win.title("Choose paint colors")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        Label(
            win,
            text="Select paint color for each stage",
            font=("Segoe UI", 12, "bold"),
            fg="#00ffcc", bg="#050a0f", pady=6
        ).pack()

        # Fixed palette: each entry is (name, hex_color)
        palette = [
            ("Red", "#ff3333"),
            ("Orange", "#ff8800"),
            ("Yellow", "#ffdd33"),
            ("Green", "#33cc55"),
            ("Cyan", "#33ddff"),
            ("Blue", "#3366ff"),
            ("Magenta", "#cc33ff"),
            ("Black", "#000000"),
            ("White", "#ffffff"),
        ]

        rf = Frame(win, bg="#050a0f")
        rf.pack(padx=8, pady=4, fill=BOTH, expand=True)

        color_vars = []

        for idx, p in enumerate(passes):
            row = Frame(rf, bg="#050a0f")
            row.pack(fill=X, pady=2)

            label_text = p.get("label") or f"Stage {idx + 1}"
            Label(
                row, text=label_text,
                font=("Segoe UI", 10, "bold"),
                fg="#e0ffff", bg="#050a0f",
                width=14, anchor="w"
            ).pack(side=LEFT)

            # StringVar that stores the chosen color for this pass
            var = StringVar(value=p.get("color", "#00ffcc"))
            color_vars.append(var)

            # Little preview swatch
            preview = Label(row, text="  ", bg=var.get(), width=4, relief=FLAT)
            preview.pack(side=LEFT, padx=(4, 8))

            # Each palette button calls a handler that sets this pass's color
            def make_handler(color, v=var, preview_label=preview):
                def set_color():
                    v.set(color)
                    preview_label.configure(bg=color)
                return set_color

            for _, hex_color in palette:
                Button(
                    row,
                    text="",
                    bg=hex_color,
                    width=2,
                    relief=FLAT,
                    command=make_handler(hex_color)
                ).pack(side=LEFT, padx=1)

        br = Frame(win, bg="#050a0f")
        br.pack(pady=(8, 10))

        def on_ok():
            # Copy chosen colors back into the job structure
            for p, var in zip(passes, color_vars):
                p["color"] = var.get()
            job["_colors_configured"] = True
            win.grab_release()
            win.destroy()
            # Re-run the simulation with chosen colors, then confirm
            self._simulate_paint_job(job)
            self._confirm_and_run_job(job)

        def on_cancel():
            win.grab_release()
            win.destroy()

        Button(
            br, text="OK",
            command=on_ok,
            font=("Segoe UI", 11, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

        Button(
            br, text="Cancel",
            command=on_cancel,
            font=("Segoe UI", 11),
            bg="#550000", fg="#ffdddd",
            relief=FLAT, padx=14, pady=4
        ).pack(side=LEFT, padx=6)

    # ----- Simulated paint preview -----------------------------------

    def _simulate_paint_job(self, job):
        """Draw a simulation of the paint passes on the main canvas."""
        self.redraw_scene()
        passes = job.get("passes") or []
        if not passes:
            return

        for idx, p in enumerate(passes):
            pts = p.get("points") or []
            color = p.get("color", "#ffffff")
            radius = max(3, 9 - idx * 2)
            for (xn, yn) in pts:
                x = int(xn * self.canvas_w)
                y = int(yn * self.canvas_h)
                self.canvas.create_oval(
                    x - radius, y - radius,
                    x + radius, y + radius,
                    fill=color, outline=""
                )
        self.draw_turret()

    def _confirm_and_run_job(self, job):
        if not job or not job.get("passes"):
            messagebox.showerror("Nothing to paint", "No paint passes defined.")
            return

        txt = [
            "Simulated paint job:",
            f"  Mode: {job.get('mode')}",
            f"  Source: {Path(job.get('source_image', '')).name}",
            f"  Passes: {len(job.get('passes') or [])}",
            "",
            "Proceed to real painting sequence?"
        ]
        if not messagebox.askokcancel("Ready to paint?", "\n".join(txt)):
            return

        def worker():
            safe_call("start_paint_from_image", job)
            safe_call("run_paint_job", job)

        threading.Thread(target=worker, daemon=True).start()

    # -----------------------------------------------------------------
    # -----------------------------------------------------------------
    # SETTINGS DIALOG + CALIBRATION
    # -----------------------------------------------------------------
    # -----------------------------------------------------------------
    # SETTINGS DIALOG – MOTOR / JOG / AUTO-FIRE ONLY
    # -----------------------------------------------------------------
    def open_settings_dialog(self):
        """Main Settings dialog: motor speeds, manual jog, and auto-fire."""
        win = Toplevel(self.root)
        win.title("VPP Settings")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        # ---------------- Title ----------------
        Label(
            win,
            text="Motor & Control Settings",
            font=("Segoe UI", 13, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=6
        ).pack()

        # ---------------- Speeds block ----------------
        sf = Frame(win, bg="#050a0f")
        sf.pack(fill=X, padx=10, pady=(4, 8))

        Label(sf, text="X speed:", font=("Segoe UI", 10), fg="#e0ffff", bg="#050a0f").grid(
            row=0, column=0, sticky="e", padx=4, pady=2
        )
        x_var = StringVar(value=str(self.settings["x_speed"]))
        Entry(sf, textvariable=x_var, width=8).grid(row=0, column=1, sticky="w", pady=2)

        Label(sf, text="Y speed:", font=("Segoe UI", 10), fg="#e0ffff", bg="#050a0f").grid(
            row=1, column=0, sticky="e", padx=4, pady=2
        )
        y_var = StringVar(value=str(self.settings["y_speed"]))
        Entry(sf, textvariable=y_var, width=8).grid(row=1, column=1, sticky="w", pady=2)

        Label(sf, text="Jog step (deg):", font=("Segoe UI", 10), fg="#e0ffff", bg="#050a0f").grid(
            row=2, column=0, sticky="e", padx=4, pady=2
        )
        step_var = StringVar(value=str(self.settings["jog_step_deg"]))
        Entry(sf, textvariable=step_var, width=8).grid(row=2, column=1, sticky="w", pady=2)

        def save_speeds():
            try:
                xs = float(x_var.get())
                ys = float(y_var.get())
                st = float(step_var.get())
            except ValueError:
                messagebox.showerror("Invalid values", "Speeds and step must be numbers.")
                return
            self.settings["x_speed"] = xs
            self.settings["y_speed"] = ys
            self.settings["jog_step_deg"] = st
            safe_call("set_motor_speeds", xs, ys)
            messagebox.showinfo("Saved", "Motor speeds updated.")

        Button(
            sf, text="Save speeds",
            command=save_speeds,
            font=("Segoe UI", 10, "bold"),
            bg="#00bbee", fg="#002222",
            relief=FLAT, padx=10, pady=4
        ).grid(row=3, column=0, columnspan=2, pady=(6, 4))

        # ---------------- Manual control block ----------------
        Label(
            win,
            text="Manual Jog / Fire / Home",
            font=("Segoe UI", 11, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=4
        ).pack()

        jf = Frame(win, bg="#050a0f")
        jf.pack(padx=10, pady=(0, 10))

        def jog(axis, direction):
            step_deg = self.settings["jog_step_deg"]
            speed = self.settings["x_speed"] if axis.upper() == "X" else self.settings["y_speed"]
            safe_call("jog", axis.upper(), direction, step_deg, speed)

        Button(
            jf, text="Y+", width=6,
            command=lambda: jog("Y", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=0, column=1, pady=2)

        Button(
            jf, text="X-", width=6,
            command=lambda: jog("X", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=0, padx=4, pady=2)

        Button(
            jf, text="X+", width=6,
            command=lambda: jog("X", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=2, padx=4, pady=2)

        Button(
            jf, text="Y-", width=6,
            command=lambda: jog("Y", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=2, column=1, pady=2)

        Button(
            jf, text="Manual Fire", width=10,
            command=lambda: safe_call("manual_fire"),
            font=("Segoe UI", 10, "bold"),
            bg="#ff8800", fg="#221100",
            relief=FLAT
        ).grid(row=1, column=1, pady=2)

        Button(
            jf, text="Home All", width=10,
            command=lambda: safe_call("home_all"),
            font=("Segoe UI", 10, "bold"),
            bg="#ffaa33", fg="#221100",
            relief=FLAT
        ).grid(row=3, column=1, pady=(6, 2))

        # ---------------- Global auto-fire setting ----------------
        Label(
            win,
            text="Auto-fire",
            font=("Segoe UI", 11, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=4
        ).pack()

        af_frame = Frame(win, bg="#050a0f")
        af_frame.pack(padx=10, pady=(0, 8), fill=X)

        Checkbutton(
            af_frame,
            text="Enable auto-fire when target locked",
            variable=self.autofire_var,
            command=self.on_toggle_autofire,
            font=("Segoe UI", 10),
            fg="#e0ffff",
            bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#330000",
            anchor="w"
        ).pack(anchor="w")

        # ---------------- Close button ----------------
        Button(
            win, text="Close",
            command=lambda: (win.grab_release(), win.destroy()),
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=12, pady=4
        ).pack(pady=(4, 8))

    def open_tracking_settings_dialog(self):
        """Settings dialog: flip Predator tracking directions per axis."""
        win = Toplevel(self.root)
        win.title("Tracking / Predator Settings")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        Label(
            win,
            text="Predator Tracking Direction",
            font=("Segoe UI", 13, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=6
        ).pack()

        Label(
            win,
            text=(
                "If the turret moves the wrong way when tracking a target,\n"
                "you can flip the axes here. These only affect Predator mode."
            ),
            font=("Segoe UI", 9),
            fg="#a8ffff", bg="#050a0f",
            justify="left"
        ).pack(padx=10, pady=(0, 8))

        frame = Frame(win, bg="#050a0f")
        frame.pack(padx=10, pady=(0, 8), fill=X)

        Checkbutton(
            frame,
            text="Invert X axis (left/right)",
            variable=self.track_invert_x_var,
            font=("Segoe UI", 10),
            fg="#e0ffff", bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#003333",
            anchor="w"
        ).pack(anchor="w", pady=2)

        Checkbutton(
            frame,
            text="Invert Y axis (up/down)",
            variable=self.track_invert_y_var,
            font=("Segoe UI", 10),
            fg="#e0ffff", bg="#050a0f",
            activebackground="#050a0f",
            selectcolor="#003333",
            anchor="w"
        ).pack(anchor="w", pady=2)

        def on_save():
            invert_x = bool(self.track_invert_x_var.get())
            invert_y = bool(self.track_invert_y_var.get())
            # Tell backend to update TRACK_INVERT_X / TRACK_INVERT_Y
            safe_call("set_tracking_inversion", invert_x=invert_x, invert_y=invert_y)
            messagebox.showinfo(
                "Tracking settings saved",
                "Tracking inversion updated for Predator mode."
            )
            try:
                win.grab_release()
            except Exception:
                pass
            win.destroy()

        def on_close():
            try:
                win.grab_release()
            except Exception:
                pass
            win.destroy()

        btn_frame = Frame(win, bg="#050a0f")
        btn_frame.pack(pady=(4, 8))

        Button(
            btn_frame,
            text="Save",
            command=on_save,
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=12, pady=4
        ).pack(side=LEFT, padx=6)

        Button(
            btn_frame,
            text="Close",
            command=on_close,
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=12, pady=4
        ).pack(side=LEFT, padx=6)

    def open_aim_calibration(self):
        """Dedicated window for aim / camera-center calibration."""
        win = Toplevel(self.root)
        win.title("Aim / Camera Center Calibration")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        # Title & instructions
        Label(
            win,
            text="Aim / Center Calibration",
            font=("Segoe UI", 13, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=4
        ).pack()

        Label(
            win,
            text=(
                "Step 1: Home the turret.\n"
                "Step 2: (Optional) Go to saved center.\n"
                "Step 3: Jog + Test Fire to line up impact with screen center.\n"
                "Step 4: Save current aim as the new center."
            ),
            font=("Segoe UI", 9),
            fg="#a8ffff", bg="#050a0f",
            justify="left"
        ).pack(padx=10, pady=(0, 4))

        # Row 1: camera preview + home / center buttons
        row1 = Frame(win, bg="#050a0f")
        row1.pack(padx=10, pady=(4, 4), fill=X)

        Button(
            row1,
            text="Open Camera Preview",
            command=self.open_camera_preview,
            font=("Segoe UI", 10, "bold"),
            bg="#00bbee", fg="#002222",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

        def do_home():
            threading.Thread(
                target=lambda: safe_call("home_all"),
                daemon=True
            ).start()

        def do_goto_center():
            threading.Thread(
                target=lambda: safe_call("goto_forward"),
                daemon=True
            ).start()

        Button(
            row1,
            text="1) Home turret",
            command=do_home,
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

        Button(
            row1,
            text="2) Go to saved center",
            command=do_goto_center,
            font=("Segoe UI", 10, "bold"),
            bg="#0088cc", fg="#e0ffff",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

        # Row 2: jog + fire
        Label(
            win,
            text="Step 3: Fine-tune aim and test fire:",
            font=("Segoe UI", 9, "bold"),
            fg="#e0ffff", bg="#050a0f"
        ).pack(padx=10, pady=(4, 2))

        jf = Frame(win, bg="#050a0f")
        jf.pack(padx=10, pady=(0, 4))

        def jog(axis, direction):
            step_deg = self.settings["jog_step_deg"]
            speed = self.settings["x_speed"] if axis.upper() == "X" else self.settings["y_speed"]
            safe_call("jog", axis.upper(), direction, step_deg, speed)

        Button(
            jf, text="Y+", width=5,
            command=lambda: jog("Y", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=0, column=1, pady=1)

        Button(
            jf, text="X-", width=5,
            command=lambda: jog("X", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=0, padx=3, pady=1)

        Button(
            jf, text="X+", width=5,
            command=lambda: jog("X", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=2, padx=3, pady=1)

        Button(
            jf, text="Y-", width=5,
            command=lambda: jog("Y", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=2, column=1, pady=1)

        Button(
            jf, text="Test Fire", width=9,
            command=lambda: safe_call("manual_fire"),
            font=("Segoe UI", 10, "bold"),
            bg="#ff8800", fg="#221100",
            relief=FLAT
        ).grid(row=1, column=1, padx=3, pady=1)

        # Row 3: save + close
        row3 = Frame(win, bg="#050a0f")
        row3.pack(padx=10, pady=(4, 6))

        Button(
            row3,
            text="4) Set Current Aim as Center",
            command=lambda: safe_call("set_current_as_forward"),
            font=("Segoe UI", 10, "bold"),
            bg="#ffaa33", fg="#221100",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

        Button(
            row3,
            text="Close",
            command=lambda: (win.grab_release(), win.destroy()),
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

    def open_travel_calibration(self):
        """
        Rotation / Travel Calibration dialog.

        Uses backend:
          - begin_travel_calibration()
          - finalize_travel_calibration()
        which you already have implemented.
        """
        win = Toplevel(self.root)
        win.title("Rotation / Travel Calibration")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        # Title & instructions
        Label(
            win,
            text="Rotation / Travel Calibration",
            font=("Segoe UI", 13, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=4
        ).pack()

        Label(
            win,
            text=(
                "Step 1: Home & unlock soft limits.\n"
                "Step 2: Jog to the FARTHEST SAFE angles (do NOT hit hard stops).\n"
                "Step 3: Save current position as the max travel for each axis.\n\n"
                "You can rerun this any time to change the travel bounds."
            ),
            font=("Segoe UI", 9),
            fg="#a8ffff", bg="#050a0f",
            justify="left"
        ).pack(padx=10, pady=(0, 4))

        # --- Step 1: home + clear max ---
        row1 = Frame(win, bg="#050a0f")
        row1.pack(padx=10, pady=(4, 4), fill=X)

        start_btn = Button(
            row1,
            text="1) Home & Unlock Travel",
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=8, pady=3
        )
        start_btn.pack(side=LEFT, padx=4)

        def do_start_calibration():
            def worker():
                start_btn.configure(state=DISABLED, text="Homing + unlocking…")
                safe_call("begin_travel_calibration")
                start_btn.configure(state=NORMAL, text="1) Home & Unlock Travel")
                messagebox.showinfo(
                    "Travel calibration",
                    "Homing complete and soft max cleared.\n\n"
                    "Now use the jog controls to move to the furthest SAFE angles,\n"
                    "then press '3) Save Soft Limits'."
                )
            threading.Thread(target=worker, daemon=True).start()

        start_btn.configure(command=do_start_calibration)

        # --- Step 2: jog cluster ---
        Label(
            win,
            text="Step 2: Jog to furthest SAFE angles (do not hit hard stops):",
            font=("Segoe UI", 9, "bold"),
            fg="#e0ffff", bg="#050a0f"
        ).pack(padx=10, pady=(4, 2))

        jf = Frame(win, bg="#050a0f")
        jf.pack(padx=10, pady=(0, 4))

        def jog(axis, direction):
            step_deg = self.settings["jog_step_deg"]
            speed = self.settings["x_speed"] if axis.upper() == "X" else self.settings["y_speed"]
            safe_call("jog", axis.upper(), direction, step_deg, speed)

        Button(
            jf, text="Y+", width=6,
            command=lambda: jog("Y", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=0, column=1, pady=1)

        Button(
            jf, text="X-", width=6,
            command=lambda: jog("X", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=0, padx=3, pady=1)

        Button(
            jf, text="X+", width=6,
            command=lambda: jog("X", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=2, padx=3, pady=1)

        Button(
            jf, text="Y-", width=6,
            command=lambda: jog("Y", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=2, column=1, pady=1)

        # --- Step 3: save soft limits ---
        row3 = Frame(win, bg="#050a0f")
        row3.pack(padx=10, pady=(4, 6), fill=X)

        save_btn = Button(
            row3,
            text="3) Save Soft Limits",
            font=("Segoe UI", 10, "bold"),
            bg="#ffaa33", fg="#221100",
            relief=FLAT, padx=8, pady=3
        )
        save_btn.pack(side=LEFT, padx=4)

        def do_save_limits():
            def worker():
                save_btn.configure(state=DISABLED, text="Saving limits…")
                safe_call("finalize_travel_calibration")
                save_btn.configure(state=NORMAL, text="3) Save Soft Limits")
                messagebox.showinfo(
                    "Soft limits saved",
                    "Soft travel limits saved.\n\n"
                    "All jogs / scans / paint moves will now be clamped to these\n"
                    "bounds until you rerun travel calibration."
                )
            threading.Thread(target=worker, daemon=True).start()

        save_btn.configure(command=do_save_limits)

        Button(
            row3,
            text="Close",
            command=lambda: (win.grab_release(), win.destroy()),
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=8, pady=3
        ).pack(side=LEFT, padx=4)

    # -----------------------------------------------------------------
    # CAMERA PREVIEW (small window) & PREDATOR SENTRY MODE
    # -----------------------------------------------------------------

    def _ensure_camera(self):
        if Picamera2 is None:
            print("Picamera2 not available", file=sys.stderr)
            return None
        if self.picam is None:
            try:
                self.picam = Picamera2()
                config = self.picam.create_preview_configuration(
                    main={"size": (640, 480)}
                )
                self.picam.configure(config)
                self.picam.start()
            except Exception as e:
                print("Camera init error:", e, file=sys.stderr)
                self.picam = None
                return None
        return self.picam

    # ---- small preview window (Start Painting) ----

    def open_camera_preview(self):
        if self._camera_mode == "predator":
            # Already using camera for predator; no extra preview
            return
        if self._camera_win is not None:
            try:
                self._camera_win.lift()
            except Exception:
                pass
            return
        cam = self._ensure_camera()
        if cam is None:
            messagebox.showerror(
                "Camera error",
                "Could not open turret camera.\n\n"
                "Make sure Picamera2 is installed and the camera is enabled."
            )
            return

        win = Toplevel(self.root)
        win.title("Turret Camera View")
        win.configure(bg="#050a0f")
        self._camera_win = win

        lbl = Label(win, bg="#000000")
        lbl.pack(padx=10, pady=10)
        self._camera_label = lbl

        Button(
            win, text="Close camera",
            command=self.close_camera_preview,
            font=("Segoe UI", 10, "bold"),
            bg="#550000", fg="#ffdddd",
            relief=FLAT, padx=10, pady=4
        ).pack(pady=(0, 10))

        self._camera_mode = "preview"
        self._camera_preview_loop()

    def _camera_preview_loop(self):
        if self._camera_mode != "preview" or self.picam is None or self._camera_win is None:
            return
        try:
            frame = self.picam.capture_array()
        except Exception as e:
            print("Camera capture error:", e, file=sys.stderr)
            self.close_camera_preview()
            return

        if Image is not None and ImageTk is not None:
            img = Image.fromarray(frame).resize((640, 480))
            photo = ImageTk.PhotoImage(img)
            self._camera_label.configure(image=photo)
            self._camera_label.image = photo

        self._camera_after_id = self.root.after(80, self._camera_preview_loop)

    def close_camera_preview(self):
        if self._camera_mode == "preview":
            self._camera_mode = "off"
        if self._camera_after_id is not None:
            try:
                self.root.after_cancel(self._camera_after_id)
            except Exception:
                pass
            self._camera_after_id = None
        if self._camera_win is not None:
            try:
                self._camera_win.destroy()
            except Exception:
                pass
            self._camera_win = None
            self._camera_label = None
        # Don't stop camera here; predator might use it later

    # ---- Predator Sentry Mode (main canvas) ----

    def on_toggle_sentry(self):
        enabled = not self.sentry_var.get()
        self.sentry_var.set(enabled)
        if enabled:
            self.sentry_btn.configure(
                text="Predator Sentry: ON",
                bg="#aa0000",
                fg="#ffeeee",
                activebackground="#dd0000",
                activeforeground="#ffffff",
            )
            self._start_predator_mode()
        else:
            self.sentry_btn.configure(
                text="Predator Sentry: OFF",
                bg="#550000",
                fg="#ffeeee",
                activebackground="#aa0000",
                activeforeground="#ffffff",
            )
            self._stop_predator_mode()

    def _start_predator_mode(self):
        """Open Predator Sentry Mode window and start tracking loop."""
        if getattr(self, "_predator_window", None) is not None:
            try:
                self._predator_window.lift()
                self._predator_window.focus_force()
                return
            except tk.TclError:
                self._predator_window = None

        # State flags
        self._predator_enabled = True
        self._predator_scan_dir = 1
        self._predator_last_target = None
        self._predator_prev_gray = None
        self._predator_lock_frames = 0
        self._predator_shot_cooldown = 0.0
        self._predator_no_motion_frames = 0

        # Mark camera mode so we don't open a second preview
        self._camera_mode = "predator"

        # Ensure backend in tracking/sentry mode,
        # and sync auto-fire with the global setting
        safe_call("set_tracking_enabled", True)
        safe_call("set_sentry_mode", True)
        safe_call("set_autofire_enabled", bool(self.autofire_var.get()))

        # Build Predator window
        self._predator_window = tk.Toplevel(self.root)
        self._predator_window.title("Predator Sentry Mode")
        self._predator_window.configure(bg="#050510")
        self._predator_window.geometry("720x560+80+80")

        # Make sure it appears on top of the fullscreen main window
        try:
            self._predator_window.transient(self.root)
            self._predator_window.grab_set()
        except Exception:
            pass
        self._predator_window.lift()
        self._predator_window.focus_force()

        self._predator_window.protocol(
            "WM_DELETE_WINDOW", self._on_predator_window_close
        )

        title_lbl = tk.Label(
            self._predator_window,
            text="PREDATOR SENTRY MODE",
            font=("Consolas", 20, "bold"),
            fg="#00ffcc",
            bg="#050510",
        )
        title_lbl.pack(pady=(10, 4))

        hint_lbl = tk.Label(
            self._predator_window,
            text="Bright target in FOV → scan, track, and auto-fire when locked (if AUTO-FIRE is enabled in Settings).",
            font=("Consolas", 11),
            fg="#8888ff",
            bg="#050510",
        )
        hint_lbl.pack(pady=(0, 8))

        # Canvas for camera + HUD overlay
        self._predator_overlay_canvas = tk.Canvas(
            self._predator_window,
            width=640,
            height=480,
            highlightthickness=0,
            bg="#000000",
        )
        self._predator_overlay_canvas.pack(fill="both", expand=True)

        # Start loop using the shared camera
        self._predator_timer = self.root.after(
            self.PREDATOR_INTERVAL_MS, self._predator_loop
        )

    def _update_predator_autofire_button(self):
        """Refresh the AUTO-FIRE button label/colors from UI state."""
        if not hasattr(self, "_predator_autofire_btn") or self._predator_autofire_btn is None:
            return

        if getattr(self, "_predator_autofire_ui_enabled", False):
            # ARMED
            self._predator_autofire_btn.configure(
                text="AUTO-FIRE: ARMED",
                fg="#00ff88",
                bg="#330000",
                activebackground="#550000",
                activeforeground="#00ff88",
            )
        else:
            # SAFE
            self._predator_autofire_btn.configure(
                text="AUTO-FIRE: SAFE",
                fg="#ffaa00",
                bg="#001010",
                activebackground="#002020",
                activeforeground="#ffaa00",
            )

    def _toggle_predator_autofire(self):
        """Toggle Predator window AUTO-FIRE SAFE/ARMED."""
        self._predator_autofire_ui_enabled = not getattr(
            self, "_predator_autofire_ui_enabled", False
        )
        self._update_predator_autofire_button()
        state = "ARMED" if self._predator_autofire_ui_enabled else "SAFE"
        print(f"UI Predator: AUTO-FIRE now {state}")

    # ---------------- PREDATOR CAMERA HELPERS ----------------

    def _capture_predator_frame(self):
        """
        Grab a single frame for Predator mode, reusing the same Picamera2
        instance as the normal camera preview.
        """
        cam = self._ensure_camera()
        if cam is None:
            # Avoid spamming message boxes; just log
            print("UI Predator: camera not available for capture")
            return None

        try:
            frame = cam.capture_array()
            return frame
        except Exception as e:
            print("UI Predator: capture error:", e, file=sys.stderr)
            return None

    def _process_predator_frame(self, frame):
        """
        Motion-based target detection:

        - Convert to grayscale and blur slightly
        - Compare to previous frame (frame differencing)
        - Threshold to find moving regions
        - Choose largest moving contour
        - Return normalized centroid (x_norm, y_norm) in [0..1] or None.
        """
        if frame is None:
            return None

        # Handle XBGR8888 → BGR
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            frame_bgr = frame

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # First frame: store baseline and don't report any motion yet
        if self._predator_prev_gray is None:
            self._predator_prev_gray = gray
            self._predator_last_target = None
            return None

        # Frame differencing
        frame_delta = cv2.absdiff(self._predator_prev_gray, gray)
        self._predator_prev_gray = gray

        # Threshold differences to get a motion mask
        _, motion_mask = cv2.threshold(
            frame_delta,
            self.PREDATOR_MOTION_THRESH,
            255,
            cv2.THRESH_BINARY,
        )
        motion_mask = cv2.dilate(motion_mask, None, iterations=2)

        contours, _ = cv2.findContours(
            motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            self._predator_last_target = None
            return None

        # Find largest moving blob
        best = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.PREDATOR_MIN_CONTOUR_AREA:
                continue
            if area > best_area:
                best_area = area
                best = c

        if best is None:
            self._predator_last_target = None
            return None

        x, y, w, h = cv2.boundingRect(best)
        cx = x + w / 2.0
        cy = y + h / 2.0

        frame_h, frame_w = gray.shape[:2]
        x_norm = cx / float(frame_w)
        y_norm = cy / float(frame_h)

        self._predator_last_target = (x_norm, y_norm)
        return self._predator_last_target

    def _update_predator_overlay(self, frame, target):
        """Draw camera frame + HUD (crosshair + target marker) onto canvas."""
        if (
            not hasattr(self, "_predator_overlay_canvas")
            or self._predator_overlay_canvas is None
        ):
            return
        if frame is None:
            return

        # If Pillow is not available, skip drawing overlay but keep logic running
        if Image is None or ImageTk is None:
            return

        # Convert to RGB for Tk
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        else:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        img = Image.fromarray(frame_rgb)
        self._predator_photo = ImageTk.PhotoImage(img)

        c = self._predator_overlay_canvas
        c.delete("all")
        c.create_image(0, 0, image=self._predator_photo, anchor="nw")

        w = img.width
        h = img.height
        cx = w // 2
        cy = h // 2

        # Neon center crosshair
        c.create_line(cx - 20, cy, cx + 20, cy, fill="#00ff88", width=1)
        c.create_line(cx, cy - 20, cx, cy + 20, fill="#00ff88", width=1)

        # Show target marker if present
        if target is not None:
            tx = int(target[0] * w)
            ty = int(target[1] * h)
            r = 18
            c.create_oval(
                tx - r,
                ty - r,
                tx + r,
                ty + r,
                outline="#ff4444",
                width=2,
            )

    # ---------------- PREDATOR MAIN LOOP ----------------

    def _predator_loop(self):
        """Periodic predator-mode scan/track/fire loop."""
        if not getattr(self, "_predator_enabled", False):
            return

        # If window died, stop
        if (
            self._predator_window is None
            or not self._predator_window.winfo_exists()
        ):
            self._stop_predator_mode()
            return

        # 1) Grab frame + detect target
        frame = self._capture_predator_frame()
        target = self._process_predator_frame(frame)
        self._update_predator_overlay(frame, target)
        # Track how long we've gone with / without motion
        if target is None:
            self._predator_no_motion_frames += 1
        else:
            self._predator_no_motion_frames = 0

        # 2) Update shot cooldown
        if self._predator_shot_cooldown > 0.0:
            self._predator_shot_cooldown -= self.PREDATOR_INTERVAL_MS / 1000.0
            if self._predator_shot_cooldown < 0.0:
                self._predator_shot_cooldown = 0.0

        # 3) Lock evaluation
        locked = False
        if target is not None:
            cx_norm = 0.5
            cy_norm = 0.5
            # ~12% of frame around center counts as "locked"
            LOCK_TOL = 0.12
            dx = float(target[0]) - cx_norm
            dy = float(target[1]) - cy_norm
            if abs(dx) < LOCK_TOL and abs(dy) < LOCK_TOL:
                self._predator_lock_frames += 1
            else:
                self._predator_lock_frames = 0
        else:
            self._predator_lock_frames = 0

        if self._predator_lock_frames >= 1 and target is not None:
            locked = True
            print(
                f"UI Predator: target LOCKED (frames={self._predator_lock_frames}) "
                f"at ({target[0]:.3f}, {target[1]:.3f})"
            )

        # 4) Decide whether to fire
        ui_armed = bool(self.autofire_var.get())
        should_fire = (
            locked
            and ui_armed
            and self._predator_shot_cooldown <= 0.0
        )

        # Always aim at the moving target in SAFE mode (no auto-fire)
        if target is not None and not ui_armed:
            # Backend will track but will not fire because auto-fire is off
            safe_call("sentry_fire_at", float(target[0]), float(target[1]))

        if should_fire and target is not None:
            print(f"UI Predator: FIRING at target ({target[0]}, {target[1]})")
            safe_call("sentry_fire_at", float(target[0]), float(target[1]))
            self._predator_shot_cooldown = 0.3  # seconds between shots
        elif locked and (not ui_armed) and target is not None:
            print(
                f"UI Predator: target locked at ({target[0]:.3f}, {target[1]:.3f}) "
                f"but AUTO-FIRE: DISABLED"
            )

        # 5) Sweep when we haven't seen motion for a while
        if target is None and self._predator_no_motion_frames >= self.PREDATOR_LOST_FRAMES:
            safe_call("sentry_scan_step", self._predator_scan_dir)
            self._predator_no_motion_frames = 0

        # Re-arm timer
        if self._predator_enabled:
            self._predator_timer = self.root.after(
                self.PREDATOR_INTERVAL_MS, self._predator_loop
            )

    # ---------------- PREDATOR CLEANUP ----------------

    def _stop_predator_mode(self):
        """Stop predator sentry mode and clean up."""
        self._predator_enabled = False
        # Allow normal camera preview again
        self._camera_mode = "off"

        # Cancel loop
        if getattr(self, "_predator_timer", None) is not None:
            try:
                self.root.after_cancel(self._predator_timer)
            except Exception:
                pass
            self._predator_timer = None

        # Tell backend to stop sentry/tracking/autofire
        safe_call("set_sentry_mode", False)
        safe_call("set_tracking_enabled", False)
        safe_call("set_autofire_enabled", False)

        # Destroy Predator window if still around
        if getattr(self, "_predator_window", None) is not None:
            try:
                if self._predator_window.winfo_exists():
                    self._predator_window.destroy()
            except Exception:
                pass
            self._predator_window = None

    def _on_predator_window_close(self):
        """Handle user closing the Predator window via the title-bar X."""
        # Reset sidebar toggle to OFF visually and logically
        self.sentry_var.set(False)
        self.sentry_btn.configure(
            text="Predator Sentry: OFF",
            bg="#550000",
            fg="#ffeeee",
            activebackground="#aa0000",
            activeforeground="#ffffff",
        )
        self._stop_predator_mode()
    # -----------------------------------------------------------------
    # STATUS POLLING / TOP BAR
    # -----------------------------------------------------------------

    def _update_status_labels(self, status):
        """Update top-bar labels based on turret.get_status() dict."""
        if not isinstance(status, dict):
            return

        estop = status.get("estop")
        x_ok = status.get("x_limit_ok")
        y_ok = status.get("y_limit_ok")
        safe_mode = status.get("safe_mode")

        # E-STOP label
        if estop is True:
            txt = "E-STOP: PRESSED"
            bg = "#880000"
            fg = "#ffdddd"
        elif estop is False:
            txt = "E-STOP: OK"
            bg = "#004422"
            fg = "#ccffdd"
        else:
            txt = "E-STOP: UNKNOWN"
            bg = "#444444"
            fg = "#eeeeee"
        self.estop_label.configure(text=txt, bg=bg, fg=fg)

        # X limit
        if x_ok is True:
            txt = "X LIMIT: OK"
            bg = "#004422"
            fg = "#ccffdd"
        elif x_ok is False:
            txt = "X LIMIT: TRIPPED"
            bg = "#884400"
            fg = "#ffddcc"
        else:
            txt = "X LIMIT: ?"
            bg = "#333333"
            fg = "#eeeeee"
        self.xlimit_label.configure(text=txt, bg=bg, fg=fg)

        # Y limit
        if y_ok is True:
            txt = "Y LIMIT: OK"
            bg = "#004422"
            fg = "#ccffdd"
        elif y_ok is False:
            txt = "Y LIMIT: TRIPPED"
            bg = "#884400"
            fg = "#ffddcc"
        else:
            txt = "Y LIMIT: ?"
            bg = "#333333"
            fg = "#eeeeee"
        self.ylimit_label.configure(text=txt, bg=bg, fg=fg)

        # Safe mode
        if safe_mode is True:
            txt = "SAFE_MODE: LOCKED"
            bg = "#661111"
            fg = "#ffcccc"
        elif safe_mode is False:
            txt = "SAFE_MODE: CLEAR"
            bg = "#003344"
            fg = "#cceeff"
        else:
            txt = "SAFE_MODE: ?"
            bg = "#333333"
            fg = "#eeeeee"
        self.safe_label.configure(text=txt, bg=bg, fg=fg)

    def _status_loop(self):
        """Background thread polling turret status and updating UI."""
        while self._status_running:
            status = safe_call("get_status") or {}
            try:
                self.root.after(0, self._update_status_labels, status)
            except Exception:
                # Root might be closing
                pass
            time.sleep(self.STATUS_INTERVAL)

    # -----------------------------------------------------------------
    # TRACKING / AUTOFIRE TOGGLES
    # -----------------------------------------------------------------

    def on_toggle_tracking(self):
        enabled = self.tracking_var.get()
        safe_call("set_tracking_enabled", bool(enabled))

    def on_toggle_autofire(self):
        """Global auto-fire toggle (sidebar + Settings share this)."""
        enabled = bool(self.autofire_var.get())
        safe_call("set_autofire_enabled", enabled)
        print(f"UI: AUTO-FIRE {'ENABLED' if enabled else 'DISABLED'}")

    # -----------------------------------------------------------------
    # CLEANUP / QUIT
    # -----------------------------------------------------------------

    def _cleanup_camera(self):
        """Stop camera and cancel any scheduled camera callbacks."""
        self._camera_mode = "off"
        if self._camera_after_id is not None:
            try:
                self.root.after_cancel(self._camera_after_id)
            except Exception:
                pass
            self._camera_after_id = None

        if self._camera_win is not None:
            try:
                self._camera_win.destroy()
            except Exception:
                pass
            self._camera_win = None
            self._camera_label = None

        if self.picam is not None:
            try:
                self.picam.stop()
                self.picam.close()
            except Exception:
                pass
            self.picam = None

    def on_quit(self):
        if not messagebox.askokcancel("Quit", "Exit Vector Projectile Painting UI?"):
            return
        # Stop status thread
        self._status_running = False
        # Stop predator / camera
        self._stop_predator_mode()
        self._cleanup_camera()
        # Tell backend to shut down safely
        safe_call("shutdown")
        try:
            self.root.destroy()
        except Exception:
            pass


# ---------------------------------------------------------------------
# MAIN ENTRY POINT
# ---------------------------------------------------------------------

def main():
    root = Tk()
    app = VPPApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
