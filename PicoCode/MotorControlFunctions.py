from machine import Pin, PWM
import sys
import time

# Flywheel sequence state (non-blocking; updated by tryFlywheel each main-loop call)
flywheel_active = False
flywheel_t0 = 0
crank_extruded = False   # True after crank servo has moved to fire angle (0°)
servo_returned = False   # True after servo returned to rest (180°) during cooldown

# Flywheel L298N wiring (Pico GPIO numbers).
FLYWHEEL_EN_GPIO = 27
FLYWHEEL_IN_A_GPIO = 26
FLYWHEEL_IN_B_GPIO = 22

# Platform motor + encoder config
PLATFORM_PWM_GPIO = 6
PLATFORM_IN1_GPIO = 7
PLATFORM_IN2_GPIO = 8
ENCODER_A_GPIO = 13
ENCODER_B_GPIO = 16
ENCODER_COUNTS_PER_REV = 3200.0  # output-shaft counts/rev for this motor+decoder mode
ENCODER_COUNTS_PER_DEG = ENCODER_COUNTS_PER_REV / 360.0
# Reject implausibly-fast encoder IRQ bursts (often wiring noise/EMI spikes).
# With a 1k/1k divider this helps suppress chatter while preserving normal motion.
ENCODER_MIN_EDGE_US = 180
PLATFORM_RANGE_MIN_DEG = -75.0
PLATFORM_RANGE_MAX_DEG = 75.0
PLATFORM_SPEED_SCALE = 1.0  # near-2x faster than 0.55 while staying within duty_u16 range
# Brief high-duty pulse to break stiction, then scaled duty (short pulse limits small-jog overshoot).
PLATFORM_KICK_DUTY = 35000
PLATFORM_KICK_MS = 55
PLATFORM_MIN_HOLD_DUTY = 14000
# Homing jog shaping: slow down close to target to reduce overshoot.
PLATFORM_JOG_NEAR_COUNTS = 60
PLATFORM_JOG_FINE_COUNTS = 20
PLATFORM_JOG_NEAR_SCALE = 0.60
PLATFORM_JOG_FINE_SCALE = 0.42
PLATFORM_JOG_TOL_COUNTS = 5      # accept target once within this many encoder counts
PLATFORM_JOG_SETTLE_MS = 60      # stay within tolerance this long before declaring done
PLATFORM_JOG_REVERSE_KICK_MS = 18  # short kick when correcting in opposite direction
# Host aiming conversion: move:<pixels> -> platform jog degrees.
PIXELS_TO_DEG = 0.20
PIXEL_JOG_MAX_DEG = 75.0
# Stall — homing jogs: usually slower starts than tracking; avoid false trips on small jogs
PLATFORM_HOME_STALL_GRACE_MS = 1000
PLATFORM_HOME_STALL_HOLD_MS = 1350

# Platform encoder/range state
platform_encoder_count = 0
platform_homed = False
platform_limit_min_count = 0
platform_limit_max_count = 0
_enc_last_state = 0
_enc_last_irq_us = 0


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


def stopFlywheel():
    """Immediately kill the flywheel motor and reset cycle state."""
    global flywheel_active, crank_extruded, servo_returned
    flywheel_pwm.duty_u16(0)
    flywheel_active = False
    crank_extruded = False
    servo_returned = False


def initializeFlywheel():
    """Initialize Pico pins for L298N flywheel control."""
    global flywheel_pwm, flywheel_in_a, flywheel_in_b

    # Flywheel EN on GP27; IN_A GP26, IN_B GP22 (see FLYWHEEL_*_GPIO above).
    flywheel_in_a = Pin(FLYWHEEL_IN_A_GPIO, Pin.OUT)
    flywheel_in_b = Pin(FLYWHEEL_IN_B_GPIO, Pin.OUT)
    flywheel_pwm = PWM(Pin(FLYWHEEL_EN_GPIO))
    flywheel_pwm.freq(1000)  # 1 kHz PWM

    # Fix one direction only (no reversing)
    flywheel_in_a.value(0)
    flywheel_in_b.value(1)

    # Initially stay off (speed = 0)
    flywheel_pwm.duty_u16(0)


def tryFlywheel():
    """
    Fully non-blocking flywheel state machine — call every main-loop pass.

    When flywheel_active is False: starts a new cycle (sets t0, resets flags).
    When True: advances through ramp → hold → cooldown phases using elapsed time;
    returns immediately each call so the caller can poll serial between calls.

    Servo timeline (all timer-based, no sleep):
      t=0                        -> servo to 180° (resting)
      t=ramp_end                 -> servo to 0° (extruded/fire position)
      t=hold_end + cooldown/4    -> servo back to 180°
      t=hold_end + cooldown      -> cycle complete, flywheel_active = False
    """
    global flywheel_active, flywheel_t0, crank_extruded, servo_returned

    ramp_up_time_ms      = 750
    pre_fire_spin_ms     = 250   # full-speed settle time before moving servo
    post_fire_hold_ms    = 1000  # keep wheel at speed after firing motion
    cooldown_time_ms     = 8000
    start_kick_duty      = 22000 # helps overcome motor stiction at spin start

    if not flywheel_active:  # start a new cycle
        flywheel_active = True
        flywheel_t0 = time.ticks_ms()
        crank_extruded = False
        servo_returned = False
        moveCrankServo(180)
        return  # let the next call begin the ramp

    elapsed    = time.ticks_diff(time.ticks_ms(), flywheel_t0)
    t_ramp_end = ramp_up_time_ms
    t_fire     = t_ramp_end + pre_fire_spin_ms
    t_hold_end = t_fire + post_fire_hold_ms
    t_srv_ret  = t_hold_end + cooldown_time_ms // 4   # when to walk servo back
    t_cycle_end = t_hold_end + cooldown_time_ms

    if elapsed < t_ramp_end:  # ramp up to full speed
        duty = int(65535 * elapsed / ramp_up_time_ms)
        if duty < start_kick_duty:
            duty = start_kick_duty
        # Re-assert direction lines each pass so electrical noise never leaves
        # the bridge in a coast/brake state during spin-up.
        flywheel_in_a.value(0)
        flywheel_in_b.value(1)
        flywheel_pwm.duty_u16(duty)

    elif elapsed < t_hold_end:  # hold at full speed (fire once after settle)
        flywheel_in_a.value(0)
        flywheel_in_b.value(1)
        flywheel_pwm.duty_u16(65535)
        if not crank_extruded and elapsed >= t_fire:
            moveCrankServo(0)
            crank_extruded = True

    else:  # cooldown — motor already off; just manage servo return and cycle end
        flywheel_pwm.duty_u16(0)
        if not servo_returned and elapsed >= t_srv_ret:
            moveCrankServo(180)
            servo_returned = True
        if elapsed >= t_cycle_end:
            flywheel_active = False


# Servo pulse width bounds (µs) — adjust if your servo needs calibration
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_PERIOD_US = 20000  # 50 Hz


def initializeCrankServo():
    """Initialize micro servo PWM on GP15 and set resting position (180°)."""
    global crank_servo_pwm
    crank_servo_pwm = PWM(Pin(15))
    crank_servo_pwm.freq(50)
    moveCrankServo(180)


def moveCrankServo(deg: int):
    """Move the crank servo to the requested angle (0–180°). 180° = resting; 0° = extruded."""
    deg = max(0, min(180, deg))
    pulse_us = SERVO_MIN_US + int(deg / 180 * (SERVO_MAX_US - SERVO_MIN_US))
    duty = int(pulse_us / SERVO_PERIOD_US * 65535)
    crank_servo_pwm.duty_u16(duty)


def initializePlatformMotor():
    """Initialize platform motor pins, PWM, and quadrature encoder."""
    global platform_pwm, platform_in1, platform_in2
    global encoder_a, encoder_b, _enc_last_state, _enc_last_irq_us
    global platform_encoder_count, platform_homed
    global platform_limit_min_count, platform_limit_max_count

    platform_in1 = Pin(PLATFORM_IN1_GPIO, Pin.OUT)
    platform_in2 = Pin(PLATFORM_IN2_GPIO, Pin.OUT)
    platform_pwm = PWM(Pin(PLATFORM_PWM_GPIO))
    platform_pwm.freq(1000)
    platform_pwm.duty_u16(0)

    encoder_a = Pin(ENCODER_A_GPIO, Pin.IN)
    encoder_b = Pin(ENCODER_B_GPIO, Pin.IN)
    _enc_last_state = (encoder_a.value() << 1) | encoder_b.value()
    _enc_last_irq_us = time.ticks_us()
    encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)
    encoder_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)

    platform_encoder_count = 0
    platform_homed = False
    platform_limit_min_count = 0
    platform_limit_max_count = 0


def _encoder_irq(_pin):
    """Quadrature decode using a compact transition lookup table."""
    global platform_encoder_count, _enc_last_state, _enc_last_irq_us
    now_us = time.ticks_us()
    if time.ticks_diff(now_us, _enc_last_irq_us) < ENCODER_MIN_EDGE_US:
        return
    new_state = (encoder_a.value() << 1) | encoder_b.value()
    if new_state == _enc_last_state:
        return
    key = (_enc_last_state << 2) | new_state
    # +1 for forward transitions, -1 for reverse transitions, 0 for invalid/noise.
    delta = (
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    )[key]
    platform_encoder_count += delta
    _enc_last_state = new_state
    _enc_last_irq_us = now_us


def _deg_to_counts(deg):
    return int(round(deg * ENCODER_COUNTS_PER_DEG))


def getPlatformAngleDeg():
    """Current platform angle relative to the most recent home position."""
    return platform_encoder_count / ENCODER_COUNTS_PER_DEG


def stopPlatformMotor():
    """Public immediate stop for platform motor."""
    _set_platform_drive(0, 0)


def jogPlatformDegBlocking(delta_deg, duty=30000, timeout_ms=4000):
    """
    Public blocking jog helper for command/test scripts.
    Positive = one direction, negative = opposite, relative to current angle.
    """
    if not platform_homed:
        _set_platform_drive(0, 0)
        return False
    current = platform_encoder_count
    requested_delta_counts = _deg_to_counts(delta_deg)
    target_counts = current + requested_delta_counts
    if target_counts > platform_limit_max_count:
        target_counts = platform_limit_max_count
    elif target_counts < platform_limit_min_count:
        target_counts = platform_limit_min_count
    delta_counts = target_counts - current
    if delta_counts == 0 and delta_deg != 0:
        delta_counts = 1 if delta_deg > 0 else -1
        # Respect limits even for tiny rounded commands.
        if current + delta_counts > platform_limit_max_count or current + delta_counts < platform_limit_min_count:
            return True
    _move_platform_counts_blocking(delta_counts, duty=duty, timeout_ms=timeout_ms)
    return True


def pixelsToPlatformDegrees(pixel_error):
    """
    Convert image-space horizontal error (pixels) to platform jog degrees.
    Positive pixels -> positive degrees. Output is capped for safety.
    """
    deg = float(pixel_error) * PIXELS_TO_DEG
    if deg > PIXEL_JOG_MAX_DEG:
        return PIXEL_JOG_MAX_DEG
    if deg < -PIXEL_JOG_MAX_DEG:
        return -PIXEL_JOG_MAX_DEG
    return deg


def jogPlatformPixelsBlocking(pixel_error, duty=30000, timeout_ms=4000):
    """Blocking jog helper that first converts pixels to degrees."""
    return jogPlatformDegBlocking(
        pixelsToPlatformDegrees(pixel_error),
        duty=duty,
        timeout_ms=timeout_ms,
    )


def _set_platform_drive(direction, duty):
    """direction: +1 forward, -1 reverse, 0 stop."""
    if direction == 0 or duty <= 0:
        platform_in1.value(0)
        platform_in2.value(0)
        platform_pwm.duty_u16(0)
        return
    if direction > 0:
        platform_in1.value(1)
        platform_in2.value(0)
    else:
        platform_in1.value(0)
        platform_in2.value(1)
    platform_pwm.duty_u16(duty)


def _move_platform_counts_blocking(delta_counts, duty=30000, timeout_ms=4000):
    """
    Blocking helper for homing jogs: move by signed encoder counts.
    Uses a small closed-loop correction: if we overshoot/undershoot, reverse and
    creep back until within tolerance for a short settle window.
    """
    if delta_counts == 0:
        return
    start = time.ticks_ms()
    target_count = platform_encoder_count + delta_counts

    scaled_duty = max(PLATFORM_MIN_HOLD_DUTY, int(duty * PLATFORM_SPEED_SCALE))
    near_duty = max(PLATFORM_MIN_HOLD_DUTY, int(scaled_duty * PLATFORM_JOG_NEAR_SCALE))
    fine_duty = max(PLATFORM_MIN_HOLD_DUTY, int(scaled_duty * PLATFORM_JOG_FINE_SCALE))

    # Small jogs get a shorter kick to reduce overshoot.
    kick_ms = 15 if abs(delta_counts) <= _deg_to_counts(8) else PLATFORM_KICK_MS
    direction = 1 if delta_counts > 0 else -1
    _set_platform_drive(direction, PLATFORM_KICK_DUTY)
    time.sleep_ms(kick_ms)

    jog_seg_start = time.ticks_ms()
    jog_quiet_since = jog_seg_start
    jog_last_enc = platform_encoder_count
    last_dir = direction
    within_tol_since = None

    while True:
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, start)
        current = platform_encoder_count
        err = target_count - current
        rem = abs(err)

        if rem <= PLATFORM_JOG_TOL_COUNTS:
            if within_tol_since is None:
                within_tol_since = now
            if time.ticks_diff(now, within_tol_since) >= PLATFORM_JOG_SETTLE_MS:
                _set_platform_drive(0, 0)
                return
        else:
            within_tol_since = None

        if elapsed >= timeout_ms:
            _set_platform_drive(0, 0)
            print("[PLATFORM] Homing jog timeout — stopping")
            return
        if current != jog_last_enc:
            jog_last_enc = current
            jog_quiet_since = now
        elif (
            time.ticks_diff(now, jog_seg_start) >= PLATFORM_HOME_STALL_GRACE_MS
            and time.ticks_diff(now, jog_quiet_since) >= PLATFORM_HOME_STALL_HOLD_MS
        ):
            _set_platform_drive(0, 0)
            print("[PLATFORM] Homing jog stalled — stopping")
            return

        direction = 1 if err > 0 else -1
        if rem <= PLATFORM_JOG_FINE_COUNTS:
            cmd_duty = fine_duty
        elif rem <= PLATFORM_JOG_NEAR_COUNTS:
            cmd_duty = near_duty
        else:
            cmd_duty = scaled_duty

        # Correction reversals use a tiny kick to overcome backlash/stiction.
        if direction != last_dir:
            _set_platform_drive(direction, PLATFORM_KICK_DUTY)
            time.sleep_ms(PLATFORM_JOG_REVERSE_KICK_MS)
            last_dir = direction

        _set_platform_drive(direction, cmd_duty)
        time.sleep_ms(5)


def homePlatformMotor():
    """
    Blocking interactive homing.
    Enter signed degree jogs (example: +5, -10). Type 'home' to lock the
    current position as 0°, then apply software limits of -60°..+60°.
    """
    _set_platform_drive(0, 0)
    print("HOME MODE: enter signed degrees (+/-). Type 'home' to set current angle to 0°.")
    while True:
        try:
            sys.stdout.write("home> ")
            _flush = getattr(sys.stdout, "flush", None)
            if callable(_flush):
                _flush()
            line = sys.stdin.readline()
            raw = line.strip().lower() if line else ""
        except EOFError:
            raw = "home"

        if not raw:
            continue
        if raw == "home":
            setPlatformHomeHere()
            _set_platform_drive(0, 0)
            print(
                "HOME SET: 0.0 deg  | range:",
                PLATFORM_RANGE_MIN_DEG,
                "to",
                PLATFORM_RANGE_MAX_DEG,
                "deg",
            )
            return

        try:
            jog_deg = float(raw)
        except ValueError:
            print("Invalid input. Use signed degrees like +5, -10, or type 'home'.")
            continue

        # During homing, platform_homed may still be False. Move by raw counts
        # so manual +/- jog always works before the final "home" command.
        jog_counts = _deg_to_counts(jog_deg)
        _move_platform_counts_blocking(jog_counts, duty=28000, timeout_ms=3500)
        print("Angle now:", round(getPlatformAngleDeg(), 2), "deg")


def setPlatformHomeHere():
    """
    Set current encoder position as 0° home and apply software range limits.
    Intended for command-driven flows (e.g., serial 'home' command in main.py).
    """
    global platform_encoder_count, platform_homed
    global platform_limit_min_count, platform_limit_max_count
    platform_encoder_count = 0
    platform_limit_min_count = _deg_to_counts(PLATFORM_RANGE_MIN_DEG)
    platform_limit_max_count = _deg_to_counts(PLATFORM_RANGE_MAX_DEG)
    platform_homed = True