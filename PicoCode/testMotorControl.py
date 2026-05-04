import sys
import time
import uselect

import MotorControlFunctions as mcf


def _read_line(poller):
    if poller.poll(0):
        line = sys.stdin.readline()
        if line:
            return line.strip()
    return None


def main():
    print("Initializing motors...")
    mcf.initializeFlywheel()
    mcf.initializePlatformMotor()
    mcf.setPlatformHomeHere()
    mcf.initializeCrankServo()
    mcf.initializeUltrasonic()
    print("Ready. Commands: P <px>, H, F, C <deg>, U, stop")

    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)
    spinning_error_px = None

    while True:
        cmd = _read_line(poller)

        if cmd:
            if cmd.startswith("P ") and len(cmd.split()) == 2:
                try:
                    spinning_error_px = int(cmd.split()[1])
                    print("Spin error_px:", spinning_error_px, "| angle:", round(mcf.getPlatformAngleDeg(), 2), "deg")
                except ValueError:
                    print("Invalid P. Use: P <signed_pixels>")
            elif cmd == "stop":
                spinning_error_px = None
                mcf.stopPlatformMotor()
                print("Stopped. angle:", round(mcf.getPlatformAngleDeg(), 2), "deg")
            elif cmd.startswith("C ") and len(cmd.split()) == 2:
                try:
                    deg = int(cmd.split()[1])
                    mcf.moveCrankServo(deg)
                    print("Crank servo:", deg, "deg")
                except ValueError:
                    print("Invalid C. Use: C <deg>")
            elif cmd == "F":
                spinning_error_px = None
                print("Flywheel sequence...")
                started = False
                while True:
                    mcf.tryFlywheel()
                    if mcf.flywheel_active:
                        started = True
                    elif started:
                        break
                    time.sleep_ms(10)
                print("Flywheel done.")
            elif cmd == "U":
                for i in range(25):
                    d = mcf.readUltrasonic()
                    print("U", i + 1, ":", -1 if d == -1 else round(d, 3), "ft")
                    time.sleep_ms(200)
            elif cmd.upper() == "H":
                spinning_error_px = None
                mcf.stopPlatformMotor()
                print("Homing (blocking)...")
                mcf.homePlatformMotor()
                print("Homing done.")
            else:
                print("Unknown:", cmd)

        if spinning_error_px is not None and mcf.rotatePlatformMotor(spinning_error_px):
            spinning_error_px = None
            print("Aim stopped (limit or stall). angle:", round(mcf.getPlatformAngleDeg(), 2), "deg")

        time.sleep_ms(10)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            mcf.stopPlatformMotor()
        except Exception:
            pass
