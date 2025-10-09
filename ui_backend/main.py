"""
main.py - FastAPI backend for Vector Projectile Painting

Exposes simple HTTP endpoints for the UI to control the system. This file
is intentionally simple and heavily commented so new teammates can follow.
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import subprocess

# Create the FastAPI app instance that uvicorn will serve.
app = FastAPI(title="Vector Projectile Painting Backend", version="1.0")

# Define the schema for incoming commands sent by the UI.
class Command(BaseModel):
    action: str                 # e.g., "paint", "testfire", "calibrate"
    params: dict | None = None  # optional parameters for that action

@app.get("/health")
def health():
    """
    Health check endpoint.
    Returns a tiny JSON document so monitoring tools (or curl) can verify the API is alive.
    """
    return {"status": "ok"}

@app.post("/command")
def command(cmd: Command):
    """
    Accepts JSON like:
      { "action": "testfire" }
      { "action": "paint", "params": {"file": "/path/to/image.png"} }
    Routes the request to the correct subsystem (stubbed with echo for now).
    Raises HTTP 400 for unknown actions or missing params.
    """
    # Log to stdout so `journalctl --user -u ui-backend.service` shows it.
    print(f"[BACKEND] action={cmd.action} params={cmd.params}", flush=True)

    if cmd.action == "testfire":
        subprocess.Popen(["echo", "Test fire triggered"])
    elif cmd.action == "calibrate":
        subprocess.Popen(["echo", "Calibration routine started"])
    elif cmd.action == "paint":
        if not cmd.params or "file" not in cmd.params:
            raise HTTPException(status_code=400, detail="Missing 'file' in params for paint")
        subprocess.Popen(["echo", f"Painting file {cmd.params['file']}"])
    else:
        raise HTTPException(status_code=400, detail=f"Unknown action: {cmd.action}")

    return {"status": "ok", "received": cmd.model_dump()}

from fastapi.staticfiles import StaticFiles
app.mount("/", StaticFiles(directory="/home/jdiamond/vector-paint/ui_frontend", html=True), name="ui")
