# Vector-Paint Team Roadmap (Code Access & Editing)

**Pi hostname:** `vpp`  
**LAN UI:** `http://vpp.local:8080/`  
**Repo on Pi:** `/home/jdiamond/vector-paint`  
**Team users:** `jdiamond`, `colton`, `tim`, `mike`

> SSH keys are assumed set up. This doc covers accessing the code, running the system, editing, Git workflow, and service controls.

---

## 1) Log in & open the project
- SSH into the Pi:
  ```bash
  ssh <your-username>@vpp.local
  # fallback: ssh <your-username>@192.168.0.39

