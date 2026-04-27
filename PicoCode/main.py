"""
postureCheckWithMotors.py — Runs on Raspberry Pi Pico (MicroPython)

Receives \n-delimited commands from the host (Raspberry Pi or Windows laptop)
over USB serial:

    on          — enable the active loop
    off         — disable the active loop
    move:<int>  — rotate the platform (positive = right, negative = left)
    fire        — trigger the flywheel/servo sequence

Loop structure:
    outer loop  — idle; waits for 'on'
    inner loop  — active; checks ultrasonic gate, then drives motors
        ultrasonic gate: if something is within 0.5 ft, pause and wait for
        clearance (or 'off') before proceeding with motor commands.
"""

import sys
import uselect as select
import time
import MotorControlFunctions as mcf

# ── Non-blocking serial reader ────────────────────────────────────────────────
# Reads one character at a time from USB-CDC stdin using select so the main
# loop is never blocked waiting for host data.

_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)


def _read_command():
    """
    Return the next complete \\n-terminated command string, or None if no full
    line is available yet.
    """
    if _poll.poll(0):            # 0 ms timeout → non-blocking
        line = sys.stdin.readline()
        if line:
            stripped = line.strip()
            return stripped if stripped else None
    return None


# ── Initialisation ────────────────────────────────────────────────────────────

def _initialize():
    mcf.initializeUltrasonic()
    mcf.initializeFlywheel()
    mcf.initializeCrankServo()
    mcf.initializePlatformMotor()
    print("Pico ready. Waiting for 'on'...")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    _initialize()

    active       = False   # True while host has sent 'on'
    fire_pending = False   # True when a 'fire' command is queued
    latest_move  = 0       # most recent move:<int> value from host
    proximity_blocked = False
    block_threshold_ft = 0.5
    clear_threshold_ft = 0.6  # hysteresis: require more clearance to exit blocked

    while True:
        # ── Outer loop: idle, waiting for 'on' ───────────────────────────────
        cmd = _read_command()
        if cmd == "on":
            active = True
            fire_pending = False
            latest_move  = 0
            proximity_blocked = False
            print("Active.")
        elif cmd == "ultrasonic":
            dist = mcf.readUltrasonic()
            if dist == -1:
                print("U: -1 (timeout/no echo)")
            else:
                print("U:", round(dist, 3), "ft")

        if not active:
            time.sleep_ms(20)
            continue

        # ── Inner loop: active ────────────────────────────────────────────────
        while active:
            # Always drain serial first so 'off' is never missed
            cmd = _read_command()
            if cmd == "off":
                active = False
                proximity_blocked = False
                mcf.rotatePlatform(0)
                print("Inactive.")
                break
            elif cmd == "stop":
                latest_move = 0
            elif cmd is not None and cmd.startswith("move:"):
                try:
                    latest_move = int(cmd[5:])
                except ValueError:
                    pass
            elif cmd == "fire":
                fire_pending = True
                print("Fire command received.")
            elif cmd == "ultrasonic":
                dist = mcf.readUltrasonic()
                if dist == -1:
                    print("U: -1 (timeout/no echo)")
                else:
                    print("U:", round(dist, 3), "ft")

            # ── Ultrasonic proximity gate ─────────────────────────────────────
            dist = mcf.readUltrasonic()
            if dist == -1:
                # Keep previous blocked state on sensor timeout to avoid flicker.
                too_close = proximity_blocked
            elif proximity_blocked:
                too_close = (dist < clear_threshold_ft)
            else:
                too_close = (dist < block_threshold_ft)

            if too_close:
                if not proximity_blocked:
                    print("BLOCKED — object too close, pausing.")
                    proximity_blocked = True
                # While blocked: keep motors stopped, then re-check next loop pass.
                mcf.rotatePlatform(0)
                if mcf.flywheel_active:
                    mcf.stopFlywheel()
                time.sleep_ms(50)
                continue
            elif proximity_blocked:
                proximity_blocked = False
                print("CLEAR — resuming.")

            # ── Actual motor code (ultrasonic clear) ──────────────────────────

            # Rotate platform toward the target
            mcf.rotatePlatform(latest_move)

            # Advance flywheel state machine (non-blocking every loop pass)
            if mcf.flywheel_active:
                mcf.tryFlywheel()
            elif fire_pending:
                fire_pending = False
                print("Flywheel starting.")
                mcf.tryFlywheel()   # starts a new cycle

            # ── Add your additional code below this line ──────────────────────
            # (user will fill this section in the next step)

            time.sleep_ms(20)   # ~50 Hz loop rate


try:
    main()
except Exception as e:
    import sys as _sys
    print("CRASH:", e)
    _sys.print_exception(e)
