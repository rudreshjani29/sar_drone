# Phase 0 — Toolchain Validation

**Status: Done**

## Goal

Get PX4 SITL running inside Gazebo, and fly a scripted mission from Python —
validating the full toolchain (PX4 + Gazebo + MAVSDK-Python) end to end before
building anything on top of it.

## What's Here

- `phase0_mission.py` — connects to a running PX4 SITL instance, arms, takes
  off, flies a fixed 3-waypoint pattern, then returns and lands. Proves the
  Python → MAVSDK → PX4 → Gazebo pipeline works.

## Environment Setup (macOS, Apple Silicon)

1. Install Xcode Command Line Tools: `xcode-select --install`
2. Install Homebrew (if needed):
   `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
3. Raise the open-file limit: add `ulimit -S -n 2048` to `~/.zshrc`, then
   `source ~/.zshrc`
4. Clone PX4: `git clone https://github.com/PX4/PX4-Autopilot.git`
5. `cd PX4-Autopilot && git submodule update --init --recursive --force`
6. Run the macOS setup script: `./Tools/setup/macos.sh --sim-tools`
   (installs Gazebo Harmonic + build tools; 20-40 min)
7. If Gazebo doesn't install via the setup script, install directly:
   `brew tap osrf/simulation && brew install gz-harmonic`
8. Install MAVSDK-Python in its own venv:
   ```
   python3 -m venv ~/sar-drone-env
   source ~/sar-drone-env/bin/activate
   pip install mavsdk
   ```

## Known macOS-Specific Issue (and fix)

On macOS, `gz sim` cannot run the physics server and GUI in one process —
attempting the standard `make px4_sitl gz_x500` will hang on "Waiting for
Gazebo world..." and time out. Also, local Gazebo Transport discovery
between the server and GUI can silently fail without an explicit IP.

**Fix — run headless (no GUI), with an explicit local IP:**

```bash
export GZ_IP=127.0.0.1
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

If you do want to visually watch a flight (optional, and noticeably slower):
open a second terminal, `export GZ_IP=127.0.0.1`, then `gz sim -g`.

## Running the Mission

With PX4 SITL running (per above), in another terminal:

```bash
source ~/sar-drone-env/bin/activate
python3 phase0_mission.py
```

Expected: connects, arms, takes off, flies 3 waypoints, returns to launch,
lands, and the script exits cleanly.
