from machine import Pin, PWM
import time

# Flywheel sequence state (non-blocking; updated by tryFlywheel each main-loop call)
flywheel_active = False
flywheel_t0 = 0
servo_at_180 = False


def initializeUltrasonic():
    """Initialize HC-SR04 pins (trig=GP0, echo=GP1)."""
    global ultrasonic_trig, ultrasonic_echo
    ultrasonic_trig = Pin(0, Pin.OUT)
    ultrasonic_echo = Pin(1, Pin.IN)
    ultrasonic_trig.value(0)
    time.sleep_us(2)


def readUltrasonic():
    """
    Read HC-SR04 distance and return value in feet.
    Blocking Operation
    trig = GP0, echo = GP1
    """

    # Ensure clean low pulse before trigger
    ultrasonic_trig.value(0)
    time.sleep_us(2)

    # 10 us trigger pulse
    ultrasonic_trig.value(1)
    time.sleep_us(10)
    ultrasonic_trig.value(0)

    # Wait for echo to go high (with timeout)
    timeout_us = 30000 # 30ms timeout
    start_wait = time.ticks_us()
    while ultrasonic_echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start_wait) > timeout_us:
            return -1
    pulse_start = time.ticks_us()

    # Measure high pulse duration (with timeout)
    while ultrasonic_echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), pulse_start) > timeout_us:
            return -1
    pulse_end = time.ticks_us()

    pulse_duration_us = time.ticks_diff(pulse_end, pulse_start)

    # microseconds -> centimeters -> inches -> feet
    distance_cm = pulse_duration_us / 58.0
    distance_in = distance_cm / 2.54
    distance_ft = distance_in / 12.0
    return distance_ft


def initializeFlywheel():
    """Initialize Pico pins for L298N flywheel control."""
    global flywheel_pwm

    # ENA + ENB tied to GP13, IN1 + IN4 tied to Pin 31 (GP26), IN2 + IN3 tied to Pin 29 (GP22)
    in_a = Pin(26, Pin.OUT)
    in_b = Pin(22, Pin.OUT)
    flywheel_pwm = PWM(Pin(27))
    flywheel_pwm.freq(1000)  # 1 kHz PWM

    # Fix one direction only (no reversing)
    in_a.value(0)
    in_b.value(1)

    # Initially stay off (speed = 0)
    flywheel_pwm.duty_u16(0)


def tryFlywheel():
    """
    When flywheel_active is False: starts the flywheel (sets t0), resets servo to 0°.
    When True: ramp up, hold at full speed, then blocking cooldown.
    After cooldown ends, flywheel_active becomes False and the next call can start again.

    Servo timeline:
      - Start of call  -> 0°
      - Ramp complete  -> 180°
      - 1/4 through cooldown -> back to 0°
    """
    global flywheel_active
    global flywheel_t0
    global servo_at_180  # tracks whether the 180° move has been sent this cycle

    ramp_up_time_ms = 1000
    hold_time_ms = 2000
    cooldown_time_ms = 5000

    if not flywheel_active:  # starting a new cycle
        flywheel_active = True
        flywheel_t0 = time.ticks_ms()
        servo_at_180 = False
        moveCrankServo(0)

    current_time = time.ticks_ms()
    elapsed = time.ticks_diff(current_time, flywheel_t0)

    t_ramp_end = ramp_up_time_ms
    t_hold_end = t_ramp_end + hold_time_ms

    if elapsed < t_ramp_end:  # ramp up to full speed
        duty = int(65535 * elapsed / ramp_up_time_ms)
        flywheel_pwm.duty_u16(duty)
    elif elapsed < t_hold_end:  # hold at full speed
        flywheel_pwm.duty_u16(65535)
        if not servo_at_180:  # move servo once, right as ramp ends
            moveCrankServo(180)
            servo_at_180 = True
    else:
        flywheel_pwm.duty_u16(0)
        fourth_cooldown_ms = cooldown_time_ms // 4
        time.sleep_ms(fourth_cooldown_ms)   # wait half the cooldown
        moveCrankServo(0)                 # return servo to home
        time.sleep_ms(cooldown_time_ms - fourth_cooldown_ms)  # finish cooldown
        flywheel_active = False


# Servo pulse width bounds (µs) — adjust if your servo needs calibration
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_PERIOD_US = 20000  # 50 Hz


def initializeCrankServo():
    """Initialize micro servo PWM on GP15 at 50 Hz, defaulting to 0°."""
    global crank_servo_pwm
    crank_servo_pwm = PWM(Pin(15))
    crank_servo_pwm.freq(50)
    moveCrankServo(0)


def moveCrankServo(deg: int):
    """
    Move the crank servo to the requested angle (0–180°).
    Clamps the input to the valid range before writing the pulse.
    """
    deg = max(0, min(180, deg))
    pulse_us = SERVO_MIN_US + int(deg / 180 * (SERVO_MAX_US - SERVO_MIN_US))
    duty = int(pulse_us / SERVO_PERIOD_US * 65535)
    crank_servo_pwm.duty_u16(duty)


def initializePlatformMotor():
    """Initialize platform motor pins and PWM."""
    global platform_pwm, platform_in1, platform_in2

    platform_in1 = Pin(7, Pin.OUT)
    platform_in2 = Pin(8, Pin.OUT)
    platform_pwm = PWM(Pin(6))
    platform_pwm.freq(1000)
    platform_pwm.duty_u16(0)


def rotatePlatform(distance: int):
    """
    Non-blocking: call every main-loop pass with the latest distance.

    Recomputes direction and PWM from the current distance each time (no internal state).
    +distance -> one direction, -distance -> opposite; abs(distance) <= 20 -> stop.
    """

    error = abs(distance)
    deadband = 20
    if error <= deadband:
        platform_in1.value(0)
        platform_in2.value(0)
        platform_pwm.duty_u16(0)
        return

    # Speed scales with distance: farther target -> faster motor
    max_error = 300
    if error > max_error:
        error = max_error

    min_duty = 25000
    max_duty = 65535
    duty = min_duty + int((error - deadband) * (max_duty - min_duty) / (max_error - deadband))

    if distance > 0:
        platform_in1.value(1)
        platform_in2.value(0)
    else:
        platform_in1.value(0)
        platform_in2.value(1)

    platform_pwm.duty_u16(duty)