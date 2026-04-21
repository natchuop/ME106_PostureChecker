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
    mcf.initializeCrankShaft()
    mcf.initializeUltrasonic()
    print("Ready.")
    print("Commands: P <int>, C, F, U")

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
            elif cmd == "C":
                print("Starting crank shaft...")
                mcf.startCrankShaft()
                print("Crank shaft complete.")
            elif cmd == "F":
                print("Running flywheel sequence for 15 seconds...")
                flywheel_end = time.ticks_add(time.ticks_ms(), 15000)
                while time.ticks_diff(flywheel_end, time.ticks_ms()) > 0:
                    mcf.tryFlywheel()
                    time.sleep_ms(10)
                print("Flywheel sequence attempted (crank may start).")
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
