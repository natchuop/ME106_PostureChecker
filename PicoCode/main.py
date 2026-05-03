"""
main.py — Posture Checker motor firmware (MicroPython on Raspberry Pi Pico)

Receives \n-delimited commands from the host (Raspberry Pi or Windows laptop)
over USB serial:

    on          — enable the active loop
    off         — disable the active loop
    h           — enter blocking platform homing mode
    move:<int>  — image error in pixels; converted to bounded jog degrees on Pico
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


def _blocking_home_sequence():
    """
    Blocking readline-based homing in MotorControlFunctions.homePlatformMotor()
    conflicts with stdin being registered on uselect.poll(). Temporarily
    unregister for homing, then restore. (Handles MicroPython stdout without
    flush().)
    """
    try:
        _poll.unregister(sys.stdin)
    except OSError:
        pass
    try:
        mcf.homePlatformMotor()
    finally:
        try:
            _poll.register(sys.stdin, select.POLLIN)
        except OSError:
            pass


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
    print("Pico ready. Send 'on' then 'h' to home the platform.")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    _initialize()

    active       = False   # True while host has sent 'on'
    fire_pending = False   # True when a 'fire' command is queued
    proximity_blocked = False
    block_threshold_ft = 0.5
    clear_threshold_ft = 0.6  # hysteresis: require more clearance to exit blocked

    while True:
        # ── Outer loop: idle, waiting for 'on' ───────────────────────────────
        cmd = _read_command()
        if cmd == "on":
            active = True
            fire_pending = False
            proximity_blocked = False
            print("Active.")
        elif cmd == "h":
            print("Entering HOME MODE (jog +/- degrees, then type 'home').")
            _blocking_home_sequence()
            print("HOME complete.")
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
                mcf.stopPlatformMotor()
                print("Inactive.")
                break
            elif cmd == "stop":
                mcf.stopPlatformMotor()
            elif cmd == "h":
                print("Entering HOME MODE (jog +/- degrees, then type 'home').")
                mcf.stopPlatformMotor()
                _blocking_home_sequence()
                print("HOME complete. Staying active.")
            elif cmd is not None and cmd.startswith("move:"):
                try:
                    pixel_error = int(cmd[5:])
                    mcf.jogPlatformPixelsBlocking(pixel_error, duty=60000, timeout_ms=4000)
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
                mcf.stopPlatformMotor()
                if mcf.flywheel_active:
                    mcf.stopFlywheel()
                time.sleep_ms(50)
                continue
            elif proximity_blocked:
                proximity_blocked = False
                print("CLEAR — resuming.")

            # ── Actual motor code (ultrasonic clear) ──────────────────────────
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
