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
    mcf.initializeCrankServo()
    mcf.initializeUltrasonic()
    print("Ready.")
    print("Commands: P <int>, F, C <deg>, U")

    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)

    last_platform_distance = 0

    while True:
        # Keep platform continuously updated with latest distance command.
        mcf.rotatePlatform(last_platform_distance)

        cmd = _read_command_nonblocking(poller)
        if cmd is not None:
            if not cmd:
                pass
            elif cmd.startswith("P "):
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        last_platform_distance = int(parts[1])
                        print("Platform distance set to:", last_platform_distance)
                    except ValueError:
                        print("Invalid P command. Use: P <int>")
                else:
                    print("Invalid P command. Use: P <int>")
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
            else:
                print("Unknown command:", cmd)

            # Clear command so it does not repeat.
            cmd = None

        # Keep flywheel state machine advancing without blocking.
        #mcf.tryFlywheel()
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
