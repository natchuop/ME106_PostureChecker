import sys
import time
import uselect

import MotorControlFunctions as mcf


def _read_command_nonblocking(poller):
    """Return one command line if available, otherwise None."""
    if poller.poll(0):
        line = sys.stdin.readline()
        if line:
            return line.strip()
    return None


def main():
    print("Initializing motors...")
    mcf.initializeFlywheel()
    mcf.initializePlatformMotor()
    # Declares current shaft position as 0° so P jog commands work immediately.
    # (same idea as typing 'home' after jog — see H below).
    mcf.setPlatformHomeHere()
    mcf.initializeCrankServo()
    mcf.initializeUltrasonic()
    print("Ready.")
    print("Commands: P <deg>, H (re-home / jog), F, C <deg>, U")

    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)

    while True:
        cmd = _read_command_nonblocking(poller)
        if cmd is not None:
            if not cmd:
                pass
            elif cmd.startswith("P "):
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        jog_deg = int(parts[1])
                        ok = mcf.jogPlatformDegBlocking(jog_deg, duty=30000, timeout_ms=4000)
                        if not ok:
                            print("Platform not homed. Use H then type home to set zero.")
                        else:
                            print(
                                "Platform jogged by:",
                                jog_deg,
                                "deg | angle now:",
                                round(mcf.getPlatformAngleDeg(), 2),
                                "deg",
                            )
                    except ValueError:
                        print("Invalid P command. Use: P <deg>")
                else:
                    print("Invalid P command. Use: P <deg>")
            elif cmd.startswith("C "):
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        deg = int(parts[1])
                        mcf.moveCrankServo(deg)
                        print("Crank servo moved to:", deg, "deg")
                    except ValueError:
                        print("Invalid C command. Use: C <deg>")
                else:
                    print("Invalid C command. Use: C <deg>")
            elif cmd == "F":
                print("Running flywheel sequence...")
                flywheel_started = False
                while True:
                    mcf.tryFlywheel()
                    if mcf.flywheel_active:
                        flywheel_started = True
                    elif flywheel_started:
                        break  # one full cycle complete, do not restart
                    time.sleep_ms(10)
                print("Flywheel sequence complete.")
            elif cmd == "U":
                print("Ultrasonic readout: 5x/sec for 5 seconds...")
                for i in range(25):
                    distance_ft = mcf.readUltrasonic()
                    if distance_ft == -1:
                        print("U", i + 1, ":", -1)
                    else:
                        print("U", i + 1, ":", round(distance_ft, 3), "ft")
                    time.sleep_ms(200)
                print("Ultrasonic readout complete.")
            elif cmd.upper() == "H":
                print("Entering platform homing mode (blocking)...")
                mcf.homePlatformMotor()
                print("Homing complete.")
            else:
                print("Unknown command:", cmd)

            # Clear command so it does not repeat.
            cmd = None

        # Keep flywheel state machine advancing without blocking.
        #mcf.tryFlywheel()
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
