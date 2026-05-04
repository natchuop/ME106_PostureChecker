"""Pico firmware: serial on/off/h/fire/stop/ultrasonic/P,<px>. Flywheel gated by distance when amUsingUltrasonicSensor (see ULTRASONIC_MAX_RANGE_FT, -1 = no echo)."""

import sys
import uselect as select
import time
import MotorControlFunctions as mcf

amUsingUltrasonicSensor = True
ULTRASONIC_MAX_RANGE_FT = 4.0

_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)


def _blocking_home_sequence():
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
    if _poll.poll(0):
        line = sys.stdin.readline()
        if line:
            s = line.strip()
            return s if s else None
    return None


def _parse_p(cmd):
    if cmd and cmd.lower().startswith("p,"):
        try:
            return int(cmd.split(",", 1)[1])
        except (ValueError, IndexError):
            pass
    return None


def _print_ultrasonic():
    if not amUsingUltrasonicSensor:
        print("Ultrasonic disabled (amUsingUltrasonicSensor=False).")
        return
    d = mcf.readUltrasonic()
    print("U: -1 (timeout/no echo)" if d == -1 else "U: " + str(round(d, 3)) + " ft")


def _initialize():
    if amUsingUltrasonicSensor:
        mcf.initializeUltrasonic()
    mcf.initializeFlywheel()
    mcf.initializeCrankServo()
    mcf.initializePlatformMotor()
    print("Pico ready. Send 'on' then 'h' to home the platform.")


def main():
    _initialize()
    active = False
    fire_pending = False
    us_prev_in_range = None

    def announce_range(in_range):
        nonlocal us_prev_in_range
        if us_prev_in_range is not None and in_range == us_prev_in_range:
            return
        us_prev_in_range = in_range
        if in_range:
            print("TARGET IN RANGE")
            print("USONIC_WEB:in_range")
        else:
            print("TARGET OUT OF RANGE")
            print("USONIC_WEB:out_of_range")

    while True:
        cmd = _read_command()
        if cmd == "on":
            active = True
            fire_pending = False
            us_prev_in_range = None
            print("Active.")
        elif cmd == "h":
            print("Entering HOME MODE (jog +/- degrees, then type 'home').")
            _blocking_home_sequence()
            print("HOME complete.")
        elif cmd == "ultrasonic":
            _print_ultrasonic()
        else:
            err = _parse_p(cmd)
            if err is not None:
                mcf.rotatePlatformMotor(err)

        if not active:
            time.sleep_ms(20)
            continue

        while active:
            breaker = False
            while True:
                cmd = _read_command()
                if cmd is None:
                    break
                if cmd == "off":
                    breaker = True
                    active = False
                    mcf.stopPlatformMotor()
                    print("Inactive.")
                    break
                if cmd == "stop":
                    mcf.stopPlatformMotor()
                elif cmd == "h":
                    print("Entering HOME MODE (jog +/- degrees, then type 'home').")
                    mcf.stopPlatformMotor()
                    _blocking_home_sequence()
                    print("HOME complete. Staying active.")
                elif cmd == "fire":
                    fire_pending = True
                    print("Fire command received.")
                elif cmd == "ultrasonic":
                    _print_ultrasonic()
                else:
                    err = _parse_p(cmd)
                    if err is not None:
                        mcf.rotatePlatformMotor(err)

            if breaker:
                break

            if amUsingUltrasonicSensor:
                d = mcf.readUltrasonic()
                in_range = d != -1 and d <= ULTRASONIC_MAX_RANGE_FT
                announce_range(in_range)
                blocked = not in_range
            else:
                blocked = False

            if blocked:
                if mcf.flywheel_active:
                    mcf.stopFlywheel()
                time.sleep_ms(50)
                continue

            if mcf.flywheel_active:
                mcf.tryFlywheel()
            elif fire_pending:
                fire_pending = False
                print("Flywheel starting.")
                mcf.tryFlywheel()

            time.sleep_ms(20)


try:
    main()
except Exception as e:
    import sys as _sys
    print("CRASH:", e)
    _sys.print_exception(e)
