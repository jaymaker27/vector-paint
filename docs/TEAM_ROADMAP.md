Vector-Paint Team Roadmap (Code Access & Editing)

Pi hostname: vpp
LAN UI: http://vpp.local:8080/

Repo on Pi: /home/jdiamond/vector-paint
Team users: jdiamond, colton, tim, mike

Note: SSH keys are assumed set up. This doc covers accessing the code, running the system, editing, Git workflow, and service controls.

Log in & open the project

SSH into the Pi (same Wi-Fi):
ssh <your-username>@vpp.local
(fallback: ssh <your-username>@192.168.0.39)

Go to the project:
cd /home/jdiamond/vector-paint
(Some users may have a shortcut at: ~/vector-paint)

Python environment
Activate venv:
source /home/jdiamond/vector-paint/.venv/bin/activate
python --version
pip --version

Dependencies:
pip install -r requirements.txt
pip install <package>
pip freeze > requirements.txt (pin versions after changes)

Deactivate:
deactivate

Notes: keep deps pinned in requirements.txt. Do NOT commit .venv/.

Running & viewing
Web UI:

On the Pi: http://localhost:8080/

On the LAN: http://vpp.local:8080/

Health check (on the Pi):
curl http://localhost:8080/health

Expected: {"status":"ok"}

Service management (backend):
sudo systemctl status vector-backend.service
sudo systemctl restart vector-backend.service

Logs (follow live):
sudo journalctl -u vector-backend.service -n 100 -f

User services (visuals, if used):
systemctl --user status splash-video.service # splash video
systemctl --user status ui-kiosk.service # kiosk browser

Day-to-day Git workflow
Start clean:
cd /home/jdiamond/vector-paint
git checkout main
git pull

Create a feature branch:
git checkout -b feat/<short-name>

Commit & push:
git add -A
git commit -m "feat: short description"
git push -u origin HEAD

Open a Pull Request on GitHub:
jaymaker27/vector-paint

Set your author (once per user, per repo):
git config user.name "Your Name"
git config user.email "you@example.com
"

Repository layout (what each part does & why)
Folder tree (reference):
vector-paint/
├─ controller/ # GPIO/devices boundary (mock-friendly)
│ ├─ driver.py # testfire(), calibrate(), paint()
│ └─ init.py
├─ planner/ # Pixels/graphs → vector paths (future)
│ ├─ raster_to_paths.py # Edge-trace & simplify
│ └─ path_optimizer.py # Smooth/merge; accel/jerk limits
├─ renderer/ # Paths → motor/solenoid timing (future)
│ ├─ kinematics.py # Step/dir timing with constraints
│ └─ gcode_export.py # Optional export for sim/debug
├─ sim/ # Plots/sims to validate plans (future)
│ └─ notebook.ipynb
├─ ui_backend/ # FastAPI app (HTTP API + static server)
│ └─ main.py # /health, /command; mounts ui_frontend at '/'
├─ ui_frontend/ # Static Main Menu
│ └─ index.html # Calibrate / Test Fire / Paint (stub)
├─ docs/ # Team docs
├─ tests/ # Unit/integration tests (fast on Pi)
├─ infra/ # Ops (systemd units, scripts)
├─ .venv/ # Project venv (not committed)
├─ requirements.txt # Pinned Python deps
├─ README.md # Overview + quickstart
└─ .gitignore # Excludes venv, caches, generated media

Why this structure (short version):

controller/: isolates hardware I/O so the rest can run with mocks.

planner/: decides what to draw (paths), independent of motion.

renderer/: decides how to move (timing), independent of planning.

sim/: visualize motion/timing early.

ui_backend/: single, testable API surface; serves the UI.

ui_frontend/: simple operator panel that calls the backend.

tests/: quick checks that run on the Pi.

infra/: boot/log/restart config separate from app logic.

.venv + requirements.txt: reproducible Python environment.

Adding a feature (end-to-end example)
Goal: add a new backend action (e.g., "settings") and a UI button.

Branch:
git checkout main && git pull
git checkout -b feat/settings-endpoint

Backend (ui_backend/main.py):

Add a case in /command for {"action":"settings"}.

Keep logic small; move heavy work into controller/ or helpers.

Front-end (ui_frontend/index.html):

Add a new button that POSTs {"action":"settings"} to /command.

Append results to the log.

Test:
curl http://localhost:8080/health

curl -X POST http://localhost:8080/command
 -H "Content-Type: application/json" -d '{"action":"settings"}'
Then open the UI and click the new button.

Logs:
sudo journalctl -u vector-backend.service -n 100 -f

Commit & push:
git add -A
git commit -m "feat(ui+backend): add settings endpoint and button"
git push -u origin HEAD

Open a PR on GitHub and request review.

Helpful commands — what they do & when to use them

SSH & networking:
ssh <user>@vpp.local
Start a secure shell into the Pi on the same LAN.
Use whenever you want to work directly on the Pi.

ssh -N -L 18080:127.0.0.1:8080 <user>@vpp.local
Create a local tunnel to view the Pi's web UI at http://localhost:18080/
.
Use this on guest/isolated Wi-Fi that blocks device-to-device traffic.

hostname -I
Show the Pi's current IPs; useful if mDNS (.local) fails.

ping vpp.local
Quick connectivity check from your laptop.

Service & logs:
sudo systemctl status vector-backend.service
Show if the backend is running + recent logs.

sudo systemctl restart vector-backend.service
Restart the backend after pulling code or changing Python.

sudo journalctl -u vector-backend.service -n 100 -f
Follow live logs to debug requests/errors/crashes.

HTTP checks:
curl http://localhost:8080/health

Fast health probe from the Pi; expect {"status":"ok"}.

curl -X POST http://localhost:8080/command
 -H "Content-Type: application/json" -d '{"action":"testfire"}'
Trigger a backend command without opening the UI.

Git (on laptops):
git clone git@github.com
:jaymaker27/vector-paint.git
First clone using your GitHub SSH key.

git checkout -b feat/<name>
Create a feature branch for your changes.

git add -A && git commit -m "feat: ..."
Save your work with a clear message.

git push -u origin HEAD
Publish your branch and set upstream.

git fetch origin && git rebase origin/main
Update your branch with latest main before PR (optional).

Git (on the Pi):
cd /home/jdiamond/vector-paint && git checkout main && git pull
Sync the Pi to the latest code before testing.

sudo systemctl restart vector-backend.service
Apply new code by restarting the service.

Infra/Access: Justin (user: jdiamond)
PRs/Issues: GitHub jaymaker27/vector-paint
Last updated: Oct 9, 2025
