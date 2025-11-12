from flask import Flask, render_template
import subprocess, sys

app = Flask(__name__)

# Function to run commands
def run(cmd_list):
    try:
        subprocess.Popen(cmd_list)
    except Exception as e:
        print("ERR:", e, file=sys.stderr, flush=True)

# Serve main UI
@app.get("/")
def index():
    return render_template("index.html")

# Handle buttons
@app.get("/<cmd>")
def handle(cmd):
    print("ACTION:", cmd, flush=True)
    if cmd == "calibrate":
        run(["echo", "Calibrate turret"])
    elif cmd == "testfire":
        run(["echo", "Test fire"])
    elif cmd == "paint":
        run(["echo", "Start painting"])
    elif cmd == "settings":
        run(["echo", "Open settings"])
    else:
        print("Unknown:", cmd, flush=True)
    return "OK\n"

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8080)

