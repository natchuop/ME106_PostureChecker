# Posture Checker

Host script: `PiCode/laptopPostureCheck.py`. Firmware: `PicoCode/` (MicroPython on the Pico).

---

## Pico firmware (required)

Upload these from the repo onto the **Raspberry Pi Pico** whenever you change them (and once after flashing MicroPython):

- `PicoCode/MotorControlFunctions.py`
- `PicoCode/main.py`

**Cursor + MicroPico (Windows):** right-click each file → *Upload file to Pico*, then **disconnect MicroPico** so the USB serial port is free for `laptopPostureCheck.py`.

**Pi only:** use the same two paths from this repo; upload with Thonny, `mpremote`, or your usual tool.

---

## Windows + Pico (MicroPico in Cursor)

**Flash MicroPython once:** hold BOOTSEL, plug Pico in, drag the `.uf2` from [micropython.org/download/RPI_PICO](https://micropython.org/download/RPI_PICO/) onto the `RPI-RP2` drive, then upload the Pico files above.

**Venv once** (from repo root):

```bat
python -m venv .venv
.\.venv\Scripts\activate.bat
pip install mediapipe opencv-python-headless pyserial flask numpy
```

**Each run** (from repo root; set `COM7` to your Pico’s port):

```bat
.\.venv\Scripts\activate.bat
set SERIAL_PORT=COM7
cd PiCode
python laptopPostureCheck.py
```

In that terminal: `on` / `off` / `h` / etc. **Ctrl+C** to exit.

---

## Raspberry Pi Zero 2 W + Pico

**One-time** (64-bit Pi OS; change `~/Project` if your clone is elsewhere):

```bash
sudo apt update
sudo apt install -y rpicam-apps libopenblas-dev libgl1 v4l-utils cpufrequtils curl
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
uv python install 3.11
uv venv --python 3.11 ~/posture-venv
source ~/posture-venv/bin/activate
pip install mediapipe opencv-python-headless pyserial flask numpy
sudo systemctl disable --now bluetooth
```

**Each boot:**

```bash
sudo cpufreq-set -g ondemand
cd ~/Project/PiCode
~/posture-venv/bin/python laptopPostureCheck.py
```

Upload the two `PicoCode/` files to the Pico whenever you change them. Stream: `http://<pi-ip>:5000/`. **Ctrl+C** to quit.
