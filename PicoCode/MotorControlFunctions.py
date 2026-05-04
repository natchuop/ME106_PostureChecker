from machine import Pin, PWM
import sys
import time

flywheel_active = False
flywheel_t0 = 0
crank_extruded = False
servo_returned = False

FLYWHEEL_EN_GPIO = 27
FLYWHEEL_IN_A_GPIO = 26
FLYWHEEL_IN_B_GPIO = 22

PLATFORM_PWM_GPIO = 6
PLATFORM_IN1_GPIO = 7
PLATFORM_IN2_GPIO = 8
ENCODER_A_GPIO = 13
ENCODER_B_GPIO = 16
ENCODER_COUNTS_PER_REV = 3200.0
ENCODER_COUNTS_PER_DEG = ENCODER_COUNTS_PER_REV / 360.0
ENCODER_MIN_EDGE_US = 180
PLATFORM_RANGE_MIN_DEG = -75.0
PLATFORM_RANGE_MAX_DEG = 75.0
PLATFORM_SPEED_SCALE = 1.0
PLATFORM_KICK_DUTY = 35000
PLATFORM_KICK_MS = 55
PLATFORM_MIN_HOLD_DUTY = 14000
PLATFORM_JOG_NEAR_COUNTS = 60
PLATFORM_JOG_FINE_COUNTS = 20
PLATFORM_JOG_NEAR_SCALE = 0.60
PLATFORM_JOG_FINE_SCALE = 0.42
PLATFORM_JOG_TOL_COUNTS = 5
PLATFORM_JOG_SETTLE_MS = 60
PLATFORM_JOG_REVERSE_KICK_MS = 18
PLATFORM_HOME_STALL_GRACE_MS = 1000
PLATFORM_HOME_STALL_HOLD_MS = 850

AIM_FAST_PX_THRESHOLD = 50
AIM_MEDIUM_PX_THRESHOLD = 20
AIM_DEADBAND_PX = 10
AIM_FAST_DUTY = 30000
AIM_MEDIUM_DUTY = 22000
AIM_SLOW_DUTY = 14000
AIM_LIMIT_NEAR_COUNTS = 26
# Must be > AIM_LIMIT_NEAR so after recovery we are not still in "near limit" for the same command.
AIM_LIMIT_RECOVER_CLEAR_COUNTS = 32
AIM_LIMIT_RECOVER_DUTY = 15000
AIM_LIMIT_RECOVER_TIMEOUT_MS = 5000
AIM_EDGE_RAMP_COUNTS = 140
AIM_EDGE_MIN_SCALE = 0.10
AIM_EDGE_MIN_DUTY = 9000
# One-time stiction break when |error| is in (deadband, 25); re-arm on deadband, dir flip, or |error|>=25.
AIM_SMALL_BOOST_PX = 25
AIM_SMALL_START_KICK_DUTY = 28500
AIM_SMALL_START_KICK_MS = 32

platform_encoder_count = 0
platform_homed = False
platform_limit_min_count = 0
platform_limit_max_count = 0
_enc_last_state = 0
_enc_last_irq_us = 0
_aim_small_boost_consumed = False
_aim_last_move_dir = 0
_aim_stall_seg_start_ms = None
_aim_stall_quiet_since_ms = None
_aim_stall_last_enc = None


def _aim_stall_segment_reset():
    global _aim_stall_seg_start_ms, _aim_stall_quiet_since_ms, _aim_stall_last_enc
    _aim_stall_seg_start_ms = None
    _aim_stall_quiet_since_ms = None
    _aim_stall_last_enc = None


def _aim_try_stall_stop(now, current):
    """True if aim stall fired (motor stopped, aim state cleared)."""
    global _aim_stall_seg_start_ms, _aim_stall_quiet_since_ms, _aim_stall_last_enc
    global _aim_small_boost_consumed, _aim_last_move_dir
    if _aim_stall_seg_start_ms is None:
        return False
    if current != _aim_stall_last_enc:
        _aim_stall_last_enc = current
        _aim_stall_quiet_since_ms = now
        return False
    if (
        time.ticks_diff(now, _aim_stall_seg_start_ms) >= PLATFORM_HOME_STALL_GRACE_MS
        and time.ticks_diff(now, _aim_stall_quiet_since_ms) >= PLATFORM_HOME_STALL_HOLD_MS
    ):
        _set_platform_drive(0, 0)
        print("[PLATFORM] Aim stalled — stopping")
        _aim_stall_segment_reset()
        _aim_small_boost_consumed = False
        _aim_last_move_dir = 0
        return True
    return False


def initializeUltrasonic():
    global ultrasonic_trig, ultrasonic_echo
    ultrasonic_trig = Pin(0, Pin.OUT)
    ultrasonic_echo = Pin(1, Pin.IN)
    ultrasonic_trig.value(0)
    time.sleep_us(2)


def readUltrasonic():
    """Return distance in feet, or -1 on timeout."""
    ultrasonic_trig.value(0)
    time.sleep_us(2)

    ultrasonic_trig.value(1)
    time.sleep_us(10)
    ultrasonic_trig.value(0)

    timeout_us = 30000
    start_wait = time.ticks_us()
    while ultrasonic_echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start_wait) > timeout_us:
            return -1
    pulse_start = time.ticks_us()

    while ultrasonic_echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), pulse_start) > timeout_us:
            return -1
    pulse_end = time.ticks_us()

    pulse_duration_us = time.ticks_diff(pulse_end, pulse_start)

    distance_cm = pulse_duration_us / 58.0
    distance_in = distance_cm / 2.54
    distance_ft = distance_in / 12.0
    return distance_ft


def stopFlywheel():
    global flywheel_active, crank_extruded, servo_returned
    flywheel_pwm.duty_u16(0)
    flywheel_active = False
    crank_extruded = False
    servo_returned = False


def initializeFlywheel():
    global flywheel_pwm, flywheel_in_a, flywheel_in_b

    flywheel_in_a = Pin(FLYWHEEL_IN_A_GPIO, Pin.OUT)
    flywheel_in_b = Pin(FLYWHEEL_IN_B_GPIO, Pin.OUT)
    flywheel_pwm = PWM(Pin(FLYWHEEL_EN_GPIO))
    flywheel_pwm.freq(1000)
    flywheel_in_a.value(0)
    flywheel_in_b.value(1)
    flywheel_pwm.duty_u16(0)


def tryFlywheel():
    """Non-blocking flywheel + crank servo cycle; call each active-loop tick."""
    global flywheel_active, flywheel_t0, crank_extruded, servo_returned

    ramp_up_time_ms = 750
    pre_fire_spin_ms = 250
    post_fire_hold_ms = 1000
    cooldown_time_ms = 8000
    start_kick_duty = 22000

    if not flywheel_active:
        flywheel_active = True
        flywheel_t0 = time.ticks_ms()
        crank_extruded = False
        servo_returned = False
        moveCrankServo(180)
        return

    elapsed    = time.ticks_diff(time.ticks_ms(), flywheel_t0)
    t_ramp_end = ramp_up_time_ms
    t_fire     = t_ramp_end + pre_fire_spin_ms
    t_hold_end = t_fire + post_fire_hold_ms
    t_srv_ret = t_hold_end + cooldown_time_ms // 4
    t_cycle_end = t_hold_end + cooldown_time_ms

    if elapsed < t_ramp_end:
        duty = int(65535 * elapsed / ramp_up_time_ms)
        if duty < start_kick_duty:
            duty = start_kick_duty
        flywheel_in_a.value(0)
        flywheel_in_b.value(1)
        flywheel_pwm.duty_u16(duty)

    elif elapsed < t_hold_end:
        flywheel_in_a.value(0)
        flywheel_in_b.value(1)
        flywheel_pwm.duty_u16(65535)
        if not crank_extruded and elapsed >= t_fire:
            moveCrankServo(0)
            crank_extruded = True

    else:
        flywheel_pwm.duty_u16(0)
        if not servo_returned and elapsed >= t_srv_ret:
            moveCrankServo(180)
            servo_returned = True
        if elapsed >= t_cycle_end:
            flywheel_active = False


SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_PERIOD_US = 20000


def initializeCrankServo():
    global crank_servo_pwm
    crank_servo_pwm = PWM(Pin(15))
    crank_servo_pwm.freq(50)
    moveCrankServo(180)


def moveCrankServo(deg: int):
    deg = max(0, min(180, deg))
    pulse_us = SERVO_MIN_US + int(deg / 180 * (SERVO_MAX_US - SERVO_MIN_US))
    duty = int(pulse_us / SERVO_PERIOD_US * 65535)
    crank_servo_pwm.duty_u16(duty)


def initializePlatformMotor():
    global platform_pwm, platform_in1, platform_in2
    global encoder_a, encoder_b, _enc_last_state, _enc_last_irq_us
    global platform_encoder_count, platform_homed
    global platform_limit_min_count, platform_limit_max_count
    global _aim_small_boost_consumed, _aim_last_move_dir

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
    _aim_small_boost_consumed = False
    _aim_last_move_dir = 0
    _aim_stall_segment_reset()


def _encoder_irq(_pin):
    global platform_encoder_count, _enc_last_state, _enc_last_irq_us
    now_us = time.ticks_us()
    if time.ticks_diff(now_us, _enc_last_irq_us) < ENCODER_MIN_EDGE_US:
        return
    new_state = (encoder_a.value() << 1) | encoder_b.value()
    if new_state == _enc_last_state:
        return
    key = (_enc_last_state << 2) | new_state
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
    return platform_encoder_count / ENCODER_COUNTS_PER_DEG


def stopPlatformMotor():
    _set_platform_drive(0, 0)


def stepRelPlatformAngle(delta_deg, duty=30000, timeout_ms=4000):
    """Blocking relative move in degrees; clamps to homed limits."""
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
        if current + delta_counts > platform_limit_max_count or current + delta_counts < platform_limit_min_count:
            return True
    _move_platform_counts_blocking(delta_counts, duty=duty, timeout_ms=timeout_ms)
    return True


def rotatePlatformMotor(error_px):
    """Aim: signed pixel error → PWM. True = limit recovery or stall (caller should stop spin)."""
    global _aim_small_boost_consumed, _aim_last_move_dir
    global _aim_stall_seg_start_ms, _aim_stall_quiet_since_ms, _aim_stall_last_enc

    if not platform_homed:
        return False

    abs_err = abs(error_px)
    if abs_err <= AIM_DEADBAND_PX:
        _set_platform_drive(0, 0)
        _aim_small_boost_consumed = False
        _aim_last_move_dir = 0
        _aim_stall_segment_reset()
        return False

    prev_dir = _aim_last_move_dir
    direction = 1 if error_px > 0 else -1
    if prev_dir != 0 and direction != prev_dir:
        _aim_small_boost_consumed = False
        _aim_stall_segment_reset()
    if abs_err >= AIM_SMALL_BOOST_PX:
        _aim_small_boost_consumed = False

    now = time.ticks_ms()
    current = platform_encoder_count

    if _aim_try_stall_stop(now, current):
        return True

    near_max = direction > 0 and current >= platform_limit_max_count - AIM_LIMIT_NEAR_COUNTS
    near_min = direction < 0 and current <= platform_limit_min_count + AIM_LIMIT_NEAR_COUNTS
    if near_max or near_min:
        _recover_platform_limit(direction)
        print("[PLATFORM] Aim travel limit")
        _aim_stall_segment_reset()
        _aim_small_boost_consumed = False
        _aim_last_move_dir = 0
        return True

    if AIM_DEADBAND_PX < abs_err < AIM_SMALL_BOOST_PX and not _aim_small_boost_consumed:
        _set_platform_drive(direction, AIM_SMALL_START_KICK_DUTY)
        time.sleep_ms(AIM_SMALL_START_KICK_MS)
        _aim_small_boost_consumed = True

    now = time.ticks_ms()
    current = platform_encoder_count

    if _aim_stall_seg_start_ms is None:
        _aim_stall_seg_start_ms = now
        _aim_stall_quiet_since_ms = now
        _aim_stall_last_enc = current

    if _aim_try_stall_stop(now, current):
        return True

    if abs_err > AIM_FAST_PX_THRESHOLD:
        duty = AIM_FAST_DUTY
    elif abs_err > AIM_MEDIUM_PX_THRESHOLD:
        duty = AIM_MEDIUM_DUTY
    else:
        duty = AIM_SLOW_DUTY

    duty = _aim_edge_scale_duty(direction, duty)
    _set_platform_drive(direction, duty)
    _aim_last_move_dir = direction
    return False


def _aim_edge_scale_duty(direction, duty):
    if direction > 0:
        margin = platform_limit_max_count - platform_encoder_count
    else:
        margin = platform_encoder_count - platform_limit_min_count
    if margin >= AIM_EDGE_RAMP_COUNTS:
        return duty
    if margin <= 0:
        return AIM_EDGE_MIN_DUTY
    scale = margin / AIM_EDGE_RAMP_COUNTS
    if scale < AIM_EDGE_MIN_SCALE:
        scale = AIM_EDGE_MIN_SCALE
    scaled = int(duty * scale)
    if scaled < AIM_EDGE_MIN_DUTY:
        scaled = AIM_EDGE_MIN_DUTY
    return scaled


def _set_platform_drive(direction, duty):
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


def _recover_platform_limit(direction_into_limit):
    rev = -direction_into_limit
    t0 = time.ticks_ms()
    rec_seg_start = t0
    rec_quiet_since = t0
    rec_last_enc = platform_encoder_count

    def _timed_out():
        return time.ticks_diff(time.ticks_ms(), t0) >= AIM_LIMIT_RECOVER_TIMEOUT_MS

    def _recovery_stalled():
        nonlocal rec_last_enc, rec_quiet_since
        now = time.ticks_ms()
        cur = platform_encoder_count
        if cur != rec_last_enc:
            rec_last_enc = cur
            rec_quiet_since = now
            return False
        return (
            time.ticks_diff(now, rec_seg_start) >= PLATFORM_HOME_STALL_GRACE_MS
            and time.ticks_diff(now, rec_quiet_since) >= PLATFORM_HOME_STALL_HOLD_MS
        )

    if rev < 0:
        while platform_encoder_count > platform_limit_max_count:
            if _timed_out():
                break
            if _recovery_stalled():
                print("[PLATFORM] Limit recovery stalled — stopping")
                break
            _set_platform_drive(rev, AIM_LIMIT_RECOVER_DUTY)
            time.sleep_ms(5)
    else:
        while platform_encoder_count < platform_limit_min_count:
            if _timed_out():
                break
            if _recovery_stalled():
                print("[PLATFORM] Limit recovery stalled — stopping")
                break
            _set_platform_drive(rev, AIM_LIMIT_RECOVER_DUTY)
            time.sleep_ms(5)

    inner_max = platform_limit_max_count - AIM_LIMIT_RECOVER_CLEAR_COUNTS
    inner_min = platform_limit_min_count + AIM_LIMIT_RECOVER_CLEAR_COUNTS
    while True:
        cur = platform_encoder_count
        if rev < 0:
            if cur <= inner_max:
                break
        else:
            if cur >= inner_min:
                break
        if _timed_out():
            break
        if _recovery_stalled():
            print("[PLATFORM] Limit recovery stalled — stopping")
            break
        _set_platform_drive(rev, AIM_LIMIT_RECOVER_DUTY)
        time.sleep_ms(5)
    _set_platform_drive(0, 0)


def _move_platform_counts_blocking(delta_counts, duty=30000, timeout_ms=4000):
    """Blocking homing jog by signed encoder counts."""
    if delta_counts == 0:
        return
    start = time.ticks_ms()
    target_count = platform_encoder_count + delta_counts

    scaled_duty = max(PLATFORM_MIN_HOLD_DUTY, int(duty * PLATFORM_SPEED_SCALE))
    near_duty = max(PLATFORM_MIN_HOLD_DUTY, int(scaled_duty * PLATFORM_JOG_NEAR_SCALE))
    fine_duty = max(PLATFORM_MIN_HOLD_DUTY, int(scaled_duty * PLATFORM_JOG_FINE_SCALE))

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

        if direction != last_dir:
            _set_platform_drive(direction, PLATFORM_KICK_DUTY)
            time.sleep_ms(PLATFORM_JOG_REVERSE_KICK_MS)
            last_dir = direction

        _set_platform_drive(direction, cmd_duty)
        time.sleep_ms(5)


def homePlatformMotor():
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

        jog_counts = _deg_to_counts(jog_deg)
        _move_platform_counts_blocking(jog_counts, duty=28000, timeout_ms=3500)
        print("Angle now:", round(getPlatformAngleDeg(), 2), "deg")


def setPlatformHomeHere():
    global platform_encoder_count, platform_homed
    global platform_limit_min_count, platform_limit_max_count
    global _aim_small_boost_consumed, _aim_last_move_dir
    _aim_small_boost_consumed = False
    _aim_last_move_dir = 0
    _aim_stall_segment_reset()
    platform_encoder_count = 0
    platform_limit_min_count = _deg_to_counts(PLATFORM_RANGE_MIN_DEG)
    platform_limit_max_count = _deg_to_counts(PLATFORM_RANGE_MAX_DEG)
    platform_homed = True