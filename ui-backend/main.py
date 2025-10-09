"""
main.py - FastAPI backend for Vector Projectile Painting

Exposes simple HTTP endpoints for the UI to control the system. This file
is intentionally simple and heavily commented so new teammates can follow.
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import subprocess

app = FastAPI(title="Vector Projectile Painting Backend", version="1.0")

class Command(BaseModel):
    action: str                 # e.g., "paint", "testfire", "calibrate"
    params: dict | None = None  # optional key/value parameters

@app.get("/health")
def health():
    """Health check for monitoring and scripts."""
    return {"status": "ok"}

@app.post("/command")
def command(cmd: Command):
    """
    Accepts JSON like:
      { "action": "testfire" }
      { "action": "paint", "params": {"file": "/path/to/image.png"} }
    """
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
