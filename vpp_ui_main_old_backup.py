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
    - Predator Sentry Mode (sweeping, tracking, auto-fire hooks)
    - Settings… (motor speeds + manual jog + fire + home)
    - Quit
- Start Painting:
    1. User picks an image file
    2. Choose Simple vs Complex:
        * Simple: 4 outline variants + detail slider
        * Complex: multi-colour segmentation into up to 5 passes
    3. Preview simulated paint on main canvas
    4. Choose paint color for each stage (fixed color selector)
    5. Confirm → live camera preview opens + backend paint job starts
- Settings dialog:
    - Editable X / Y speed (units are arbitrary for backend)
    - Jog X+/X-/Y+/Y-, Manual Fire, Home All
    - Aim / Camera Center Calibration tool
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

If a function is missing, the UI will just print a message and keep running.
"""

import sys
import threading
import time
import random
import math
from pathlib import Path

from tkinter import *
from tkinter import ttk, messagebox, filedialog

# Pillow for image work / previews
try:
    from PIL import Image, ImageTk, ImageFilter
except Exception as e:  # pillow not present
    print("WARNING: Pillow (PIL) not available, image functions will be limited:", e, file=sys.stderr)
    Image = None
    ImageTk = None

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
    PREDATOR_INTERVAL_MS = 80

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

        self.current_image_path = None
        self.current_job = None

        # Camera state
        self.picam = None
        self._camera_mode = "off"  # "off" | "preview" | "predator"
        self._camera_after_id = None
        self._camera_win = None    # for preview window
        self._camera_label = None
        self._predator_prev_gray = None
        self._predator_scan_dir = 1  # +1 right, -1 left
        self._predator_lock_timer = 0.0

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

        # Menu bar (only Settings here, rest is sidebar)
        menubar = Menu(self.root, tearoff=False)
        settings_menu = Menu(menubar, tearoff=False)
        settings_menu.add_command(label="Settings…", command=self.open_settings_dialog)
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

    # ----- Start Painting flow ---------------------------------------

    def on_start_paint(self):
        """
        Start the painting flow:
        1) Ask user for an image
        2) Ask for Simple vs Complex mode
        3) Run the chosen flow (which builds job + preview + colors)
        NOTE: Camera preview is opened later in _confirm_and_run_job(),
              after user approves the simulated job.
        """
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

            var = StringVar(value=p.get("color", "#00ffcc"))
            color_vars.append(var)

            preview = Label(row, text="  ", bg=var.get(), width=4, relief=FLAT)
            preview.pack(side=LEFT, padx=(4, 8))

            def make_handler(hex_color, v=var, preview_label=preview):
                def set_color():
                    v.set(hex_color)
                    preview_label.configure(bg=hex_color)
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
            for p, var in zip(passes, color_vars):
                p["color"] = var.get()
            job["_colors_configured"] = True
            win.grab_release()
            win.destroy()
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

    def _animate_paint_job(self, job, blob_delay_ms=120):
        """
        Animate the paint passes on the main canvas:
        draw each blob (point) one by one with a small delay.
        This runs independently of the backend, purely visual.
        """
        passes = job.get("passes") or []
        blobs = []

        for idx, p in enumerate(passes):
            radius = max(3, 9 - idx * 2)
            color = p.get("color", "#ffffff")
            for (xn, yn) in p.get("points") or []:
                blobs.append((xn, yn, color, radius))

        if not blobs:
            return

        # Shuffle for a more “spray” / organic feel
        random.shuffle(blobs)

        # start the sequence
        self._animate_blobs_sequence(blobs, 0, blob_delay_ms)

    def _animate_blobs_sequence(self, blobs, i, delay_ms):
        if i >= len(blobs):
            return
        if not self._status_running:
            return

        xn, yn, color, radius = blobs[i]
        x = int(xn * self.canvas_w)
        y = int(yn * self.canvas_h)

        # draw this one blob
        self.canvas.create_oval(
            x - radius, y - radius,
            x + radius, y + radius,
            fill=color,
            outline=""
        )

        # schedule the next blob
        self.root.after(
            delay_ms,
            lambda: self._animate_blobs_sequence(blobs, i + 1, delay_ms)
        )

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
            print("[UI] User cancelled paint job")
            return

        print("[UI] Confirmed paint job, starting camera + animation")
        print(f"[UI] Job passes: {len(job.get('passes') or [])}")

        # Open live camera preview now that user has confirmed
        self.open_camera_preview()

        # Start UI animation of the paint passes on the main canvas
        self._animate_paint_job(job, blob_delay_ms=120)

        def worker():
            print("[UI] Background worker: calling turret.start_paint_from_image")
            safe_call("start_paint_from_image", job)
            print("[UI] Background worker: calling turret.run_paint_job")
            safe_call("run_paint_job", job)
            print("[UI] Background worker: done")

        threading.Thread(target=worker, daemon=True).start()

    # -----------------------------------------------------------------
    def open_settings_dialog(self):
        """Main Settings dialog: motor speeds, manual jog, and calibration tools."""
        win = Toplevel(self.root)
        win.title("VPP Settings")
        win.configure(bg="#050a0f")
        win.transient(self.root)
        win.grab_set()

        # Title
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

        # ---------------- Calibration tools ----------------
        Label(
            win,
            text="Calibration Tools",
            font=("Segoe UI", 11, "bold"),
            fg="#00ffcc", bg="#050a0f",
            pady=4
        ).pack()

        Button(
            win,
            text="Aim / Camera Center Calibration…",
            command=self.open_aim_calibration,
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=10, pady=4
        ).pack(pady=(0, 8))

        # ---------------- Close button ----------------
        Button(
            win, text="Close",
            command=lambda: (win.grab_release(), win.destroy()),
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=12, pady=4
        ).pack(pady=(0, 8))

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
            pady=6
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
        ).pack(padx=10, pady=(0, 6))

        # Row 1: camera preview
        row_cam = Frame(win, bg="#050a0f")
        row_cam.pack(padx=10, pady=(4, 4), fill=X)

        Button(
            row_cam,
            text="Open Camera Preview",
            command=self.open_camera_preview,
            font=("Segoe UI", 10, "bold"),
            bg="#00bbee", fg="#002222",
            relief=FLAT, padx=10, pady=4
        ).pack(side=LEFT, padx=4)

        # Row 2: home + go to center / combo
        row_home = Frame(win, bg="#050a0f")
        row_home.pack(padx=10, pady=(4, 4), fill=X)

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

        def do_home_and_center():
            def worker():
                safe_call("home_all")
                safe_call("goto_forward")
            threading.Thread(target=worker, daemon=True).start()

        Button(
            row_home,
            text="1) Home turret",
            command=do_home,
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT, padx=10, pady=4
        ).pack(side=LEFT, padx=4)

        Button(
            row_home,
            text="2) Go to saved center",
            command=do_goto_center,
            font=("Segoe UI", 10, "bold"),
            bg="#0088cc", fg="#e0ffff",
            relief=FLAT, padx=10, pady=4
        ).pack(side=LEFT, padx=4)

        Button(
            row_home,
            text="Home + Go to Center",
            command=do_home_and_center,
            font=("Segoe UI", 10, "bold"),
            bg="#00aa88", fg="#e0ffff",
            relief=FLAT, padx=10, pady=4
        ).pack(side=LEFT, padx=4)

        # Row 3: jog + fire
        Label(
            win,
            text="Step 3: Fine-tune aim and test fire:",
            font=("Segoe UI", 9, "bold"),
            fg="#e0ffff", bg="#050a0f"
        ).pack(padx=10, pady=(6, 2))

        jf = Frame(win, bg="#050a0f")
        jf.pack(padx=10, pady=(0, 6))

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
        ).grid(row=0, column=1, pady=2)

        Button(
            jf, text="X-", width=5,
            command=lambda: jog("X", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=0, padx=4, pady=2)

        Button(
            jf, text="X+", width=5,
            command=lambda: jog("X", +1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=1, column=2, padx=4, pady=2)

        Button(
            jf, text="Y-", width=5,
            command=lambda: jog("Y", -1),
            font=("Segoe UI", 10, "bold"),
            bg="#00ffcc", fg="#002222",
            relief=FLAT
        ).grid(row=2, column=1, pady=2)

        Button(
            jf, text="Test Fire", width=10,
            command=lambda: safe_call("manual_fire"),
            font=("Segoe UI", 10, "bold"),
            bg="#ff8800", fg="#221100",
            relief=FLAT
        ).grid(row=1, column=1, padx=4, pady=2)

        # Row 4: save + close
        row_save = Frame(win, bg="#050a0f")
        row_save.pack(padx=10, pady=(6, 8))

        Button(
            row_save,
            text="4) Set Current Aim as Center",
            command=lambda: safe_call("set_current_as_forward"),
            font=("Segoe UI", 10, "bold"),
            bg="#ffaa33", fg="#221100",
            relief=FLAT, padx=10, pady=4
        ).pack(side=LEFT, padx=4)

        Button(
            row_save,
            text="Close",
            command=lambda: (win.grab_release(), win.destroy()),
            font=("Segoe UI", 10),
            bg="#333333", fg="#eeeeee",
            relief=FLAT, padx=10, pady=4
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
                # Smaller preview to keep latency down
                config = self.picam.create_preview_configuration(
                    main={"size": (640, 360)}
                )
                self.picam.configure(config)
                self.picam.start()
            except Exception as e:
                print("Camera init error:", e, file=sys.stderr)
                self.picam = None
                return None
        return self.picam

    # ---- small preview window (Start Painting + calibration) ----

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
            img = Image.fromarray(frame).resize((640, 360))
            photo = ImageTk.PhotoImage(img)
            self._camera_label.configure(image=photo)
            self._camera_label.image = photo

        # Slightly faster refresh for smoother preview
        self._camera_after_id = self.root.after(60, self._camera_preview_loop)

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
            self._start_predator_mode()
        else:
            self._stop_predator_mode()

    def _start_predator_mode(self):
        self.close_camera_preview()  # one camera mode at a time
        cam = self._ensure_camera()
        if cam is None:
            self.sentry_var.set(False)
            messagebox.showerror(
                "Camera error",
                "Could not open turret camera for Predator Sentry Mode.\n\n"
                "Make sure Picamera2 is installed and the camera is enabled."
            )
            return

        self._camera_mode = "predator"
        self._predator_prev_gray = None
        self._predator_scan_dir = 1
        self._predator_lock_timer = 0.0

        self.sentry_btn.configure(
            text="Predator Sentry: ON",
            bg="#006622"
        )

        # Automatically enable autofire when Predator mode starts
        try:
            self.autofire_var.set(True)
        except Exception:
            pass
        safe_call("set_autofire_enabled", True)
        safe_call("set_sentry_mode", True)

        self._predator_loop()

    def _stop_predator_mode(self):
        self._camera_mode = "off"
        if self._camera_after_id is not None:
            try:
                self.root.after_cancel(self._camera_after_id)
            except Exception:
                pass
            self._camera_after_id = None

        self.sentry_btn.configure(
            text="Predator Sentry: OFF",
            bg="#550000"
        )

        # Turn off autofire when leaving Predator mode
        try:
            self.autofire_var.set(False)
        except Exception:
            pass
        safe_call("set_autofire_enabled", False)
        safe_call("set_sentry_mode", False)

        self.redraw_scene()

    def _predator_loop(self):
        if self._camera_mode != "predator" or self.picam is None:
            return

        try:
            frame = self.picam.capture_array()
        except Exception as e:
            print("Predator capture error:", e, file=sys.stderr)
            self._stop_predator_mode()
            return

        h, w, _ = frame.shape
        # Simple luminance
        gray = (0.299 * frame[:, :, 0] +
                0.587 * frame[:, :, 1] +
                0.114 * frame[:, :, 2])

        # Sweep motion: tell backend to scan a bit each tick (direction flips when "edges" seen)
        safe_call("sentry_scan_step", self._predator_scan_dir)

        # Very crude detection: look for high contrast patches vs mean
        mean = gray.mean()
        mask = gray < (mean - 20)  # darker-than-average zones
        # Optionally combine with motion: compare to previous frame
        if self._predator_prev_gray is not None:
            diff = abs(gray - self._predator_prev_gray)
            mask &= diff > 20
        self._predator_prev_gray = gray

        # Find bounding boxes of mask in coarse grid
        boxes = []
        grid = 16
        for gy in range(0, h, grid):
            for gx in range(0, w, grid):
                sub = mask[gy:gy + grid, gx:gx + grid]
                if sub.size == 0:
                    continue
                if sub.mean() > 0.35:
                    boxes.append((gx, gy, gx + grid, gy + grid))

        # Merge into one big box if there are many
        if boxes:
            xs1 = [b[0] for b in boxes]
            ys1 = [b[1] for b in boxes]
            xs2 = [b[2] for b in boxes]
            ys2 = [b[3] for b in boxes]
            box = (min(xs1), min(ys1), max(xs2), max(ys2))
        else:
            box = None

        #   - When no target: slowly adjust scan direction.
        #   - When target present: keep firing with a short cooldown
        #     until the target / motion disappears.
        if box is None:
            # No target: decrement lock timer and occasionally flip scan direction to search
            self._predator_lock_timer = max(0.0, self._predator_lock_timer - 0.1)
            if self._predator_lock_timer <= 0.0:
                self._predator_scan_dir *= -1
                self._predator_lock_timer = 0.5
        else:
            # Target present: keep firing while motion/target persists,
            # with a short cooldown between shots.
            self._predator_lock_timer = max(0.0, self._predator_lock_timer - 0.1)
            if self._predator_lock_timer <= 0.0:
                cx = (box[0] + box[2]) / 2.0
                cy = (box[1] + box[3]) / 2.0
                x_norm = cx / w
                y_norm = cy / h
                safe_call("sentry_fire_at", x_norm, y_norm)
                # small cooldown -> burst of shots while target is active
                self._predator_lock_timer = 0.3

        # Draw on canvas
        self.canvas.delete("all")
        if Image is not None and ImageTk is not None:
            # Scale frame to canvas
            img = Image.fromarray(frame)
            img = img.resize((self.canvas_w, self.canvas_h))
            photo = ImageTk.PhotoImage(img)
            self.canvas.create_image(
                self.canvas_w // 2,
                self.canvas_h // 2,
                image=photo,
                anchor="center"
            )
            # Keep reference
            self.canvas.image = photo
        else:
            # Fallback: just show dark background
            self.canvas.create_rectangle(
                0, 0, self.canvas_w, self.canvas_h,
                fill="#000000", outline=""
            )

        # Draw box overlays
        if box is not None:
            scale_x = self.canvas_w / w
            scale_y = self.canvas_h / h
            x1 = box[0] * scale_x
            y1 = box[1] * scale_y
            x2 = box[2] * scale_x
            y2 = box[3] * scale_y
            self.canvas.create_rectangle(
                x1, y1, x2, y2,
                outline="#ff3333", width=3
            )
            self.canvas.create_text(
                x1 + 6, y1 + 6,
                text="TARGET",
                fill="#ff3333",
                font=("Segoe UI", 10, "bold"),
                anchor="nw"
            )

        # Schedule next frame
        self._camera_after_id = self.root.after(
            self.PREDATOR_INTERVAL_MS,
            self._predator_loop
        )

    # -----------------------------------------------------------------
    # STATUS POLLING
    # -----------------------------------------------------------------

    def _status_loop(self):
        while self._status_running:
            status = {}
            try:
                if turret is not None:
                    get_status = getattr(turret, "get_status", None)
                    if callable(get_status):
                        status = get_status() or {}
            except Exception as e:
                print("Status loop error:", e, file=sys.stderr)

            self.root.after(0, self._update_status_labels, status)
            time.sleep(self.STATUS_INTERVAL)

    def _update_status_labels(self, status):
        estop = status.get("estop", None)
        if estop is True:
            self.estop_label.configure(text="E-STOP: PRESSED",
                                       bg="#aa0000", fg="#ffdddd")
        elif estop is False:
            self.estop_label.configure(text="E-STOP: OK",
                                       bg="#004422", fg="#ccffdd")
        else:
            self.estop_label.configure(text="E-STOP: UNKNOWN",
                                       bg="#444444", fg="#eeeeee")

        x_ok = status.get("x_limit_ok", True)
        y_ok = status.get("y_limit_ok", True)

        self.xlimit_label.configure(
            text=f"X LIMIT: {'OK' if x_ok else 'TRIPPED'}",
            bg="#004422" if x_ok else "#aa0000",
            fg="#ccffdd" if x_ok else "#ffdddd"
        )
        self.ylimit_label.configure(
            text=f"Y LIMIT: {'OK' if y_ok else 'TRIPPED'}",
            bg="#004422" if y_ok else "#aa0000",
            fg="#ccffdd" if y_ok else "#ffdddd"
        )

        safe_mode = status.get("safe_mode", False)
        self.safe_label.configure(
            text=f"SAFE_MODE: {'ON' if safe_mode else 'OFF'}",
            bg="#0055aa" if safe_mode else "#002244",
            fg="#cce6ff"
        )

    # -----------------------------------------------------------------
    # TRACK / AUTOFIRE TOGGLES
    # -----------------------------------------------------------------

    def on_toggle_tracking(self):
        enabled = self.tracking_var.get()
        safe_call("set_tracking_enabled", enabled)

    def on_toggle_autofire(self):
        enabled = self.autofire_var.get()
        safe_call("set_autofire_enabled", enabled)

    # -----------------------------------------------------------------
    # CLEANUP
    # -----------------------------------------------------------------

    def on_quit(self):
        if not messagebox.askokcancel("Quit", "Exit Vector Projectile Painting UI?"):
            return
        self._status_running = False
        self._stop_predator_mode()
        self.close_camera_preview()
        if self.picam is not None:
            try:
                self.picam.stop()
            except Exception:
                pass
        safe_call("shutdown")
        self.root.destroy()


# ---------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------

def main():
    root = Tk()
    app = VPPApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
