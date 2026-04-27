"""
testPicoComms.py — Run on laptop/Pi to test serial communication with the Pico.

Mirrors the testMotorControl.py interface but sends commands over USB serial
to the Pico running main.py instead of controlling hardware directly.

Commands (type and press Enter):
  on          — activate the Pico (starts motor loop)
  off         — deactivate the Pico
  P <int>     — rotate platform (e.g. "P 100", "P -50")
  F           — fire flywheel
  stop        — stop platform rotation
  U           — read ultrasonic sensor once
  q / quit    — exit this script
"""

import os
import sys
import threading
import time

import serial

IS_WINDOWS = sys.platform == "win32"
SERIAL_PORT = os.environ.get("SERIAL_PORT", "COM10" if IS_WINDOWS else "/dev/ttyACM0")
BAUD_RATE = 9600

# ── Connect ───────────────────────────────────────────────────────────────────

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
    # Interrupt any running script (e.g. testMotorControl.py left in REPL)
    # then soft-reboot so main.py starts fresh.
    ser.write(b'\x03')      # Ctrl+C — stop any running script
    time.sleep(0.5)
    ser.write(b'\x04')      # Ctrl+D — MicroPython soft reboot → runs main.py
    time.sleep(3)           # wait for main.py to initialize hardware
except Exception as e:
    print(f"[SERIAL] Could not open {SERIAL_PORT}: {e}")
    print("Set the SERIAL_PORT environment variable if your port is different.")
    sys.exit(1)


# ── Background reader — prints everything the Pico sends ─────────────────────

def _reader():
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"[PICO] {line}")
        except Exception:
            pass

threading.Thread(target=_reader, daemon=True).start()


# ── Send helper ───────────────────────────────────────────────────────────────

def send(cmd: str):
    try:
        ser.write(f"{cmd}\n".encode())
        print(f"[SENT] {cmd}")
    except Exception as e:
        print(f"[ERROR] {e}")


# ── Main REPL ─────────────────────────────────────────────────────────────────

print()
print("Commands: on, off, P <int>, F, stop, U, q")
print("Waiting for Pico startup message...")
print()

while True:
    try:
        raw = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\nExiting.")
        break

    if not raw:
        continue

    upper = raw.upper()

    if upper in ("Q", "QUIT", "EXIT"):
        print("Exiting.")
        break
    elif raw == "on":
        send("on")
    elif raw == "off":
        send("off")
    elif raw == "stop":
        send("stop")
    elif upper == "F":
        send("fire")
    elif upper == "U":
        send("ultrasonic")
    elif upper.startswith("P "):
        parts = raw.split()
        if len(parts) == 2:
            try:
                val = int(parts[1])
                send(f"move:{val}")
            except ValueError:
                print("Use: P <int>  (e.g. P 100 or P -50)")
        else:
            print("Use: P <int>  (e.g. P 100 or P -50)")
    else:
        print(f"Unknown command: {raw!r}")
        print("Commands: on, off, P <int>, F, stop, U, q")
