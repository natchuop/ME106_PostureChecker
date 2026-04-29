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
# One lock for all serial I/O. On Windows, readline() in a background thread
# concurrent with write() causes WriteFile / PermissionError on the COM port.

serial_lock = threading.Lock()

try:
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0.1,
        write_timeout=1.0,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )
    print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
    # If the Pico (or a previous Thonny session) is in *raw* REPL, Ctrl+D
    # does not soft-reboot — it ends a raw transfer ("OK" / stays in raw).
    # 1) Ctrl+B → normal REPL, 2) Ctrl+C → stop script, 3) Ctrl+D → soft reset.
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(b"\x02")  # Ctrl+B — exit raw REPL
    time.sleep(0.2)
    with serial_lock:
        ser.write(b"\x03")  # Ctrl+C — stop any running main.py
    time.sleep(0.4)
    with serial_lock:
        ser.write(b"\x04")  # Ctrl+D — soft reboot; boot → main.py
    time.sleep(3)  # main.py + hardware init
except Exception as e:
    print(f"[SERIAL] Could not open {SERIAL_PORT}: {e}")
    print("Set the SERIAL_PORT environment variable if your port is different.")
    sys.exit(1)


# ── Background reader — prints everything the Pico sends ─────────────────────

def _reader():
    buf = b""
    while True:
        try:
            with serial_lock:
                n = ser.in_waiting
                if n:
                    buf += ser.read(n)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"[PICO] {text}")
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_reader, daemon=True).start()


# ── Send helper ───────────────────────────────────────────────────────────────

def send(cmd: str):
    try:
        data = f"{cmd}\n".encode()
        with serial_lock:
            ser.write(data)
            ser.flush()
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
