from machine import Pin, PWM
import time

# Flywheel sequence state (non-blocking; updated by tryFlywheel each main-loop call)
flywheel_active = False
flywheel_t0 = 0

# Crank: one blocking run per flywheel hold; step count ~= duration ms at 1 ms/step
crank_ready = True
total_crank_time_ms = 2048  # full steps per revolution (28BYJ-48 typical)
CRANK_STEP_DELAY_MS = 1  # must match startCrankShaft()


def initializeFlywheel():
    """Initialize Pico pins for L298N flywheel control."""
    global flywheel_pwm

    # ENA + ENB tied to GP13, IN1 + IN4 tied to Pin 31 (GP26), IN2 + IN3 tied to Pin 29 (GP22)
    in_a = Pin(31, Pin.OUT)
    in_b = Pin(29, Pin.OUT)
    flywheel_pwm = PWM(Pin(32))
    flywheel_pwm.freq(1000)  # 1 kHz PWM

    # Fix one direction only (no reversing)
    in_a.value(1)
    in_b.value(0)

    # Initially stay off (speed = 0)
    flywheel_pwm.duty_u16(0)


def tryFlywheel():
    """
    When flywheel_active is False: starts the flywheel (sets t0).
    When True: ramp / hold / cooldown using elapsed time only.
    After cooldown ends, flywheel_active becomes False and the next call can start again.

    Ramp (non-blocking), then hold: one blocking crank rotation (startCrankShaft),
    then cooldown (nonblocking). Sequence finishes after cooldown; crank completes inside hold.

    
    """
    global flywheel_active  # true when flywheel is active, otherwise false
    global flywheel_t0  # time when flywheel was started
    global crank_ready

    ramp_up_time_ms = 1000
    cooldown_time_ms = 5000

    if not flywheel_active:  # flywheel is not active, so we start it
        flywheel_active = True
        flywheel_t0 = time.ticks_ms()
        crank_ready = True

    current_time = time.ticks_ms()
    elapsed = time.ticks_diff(current_time, flywheel_t0)

    # Hold window = wall time the crank runs (steps * ms per step), after ramp ends
    crank_duration_ms = total_crank_time_ms * CRANK_STEP_DELAY_MS
    t_ramp_end = ramp_up_time_ms
    t_hold_end = t_ramp_end + crank_duration_ms
    t_cooldown_end = t_hold_end + cooldown_time_ms

    if elapsed >= t_cooldown_end:  # cooldown time has elapsed, so we allow the next call to start the flywheel again
        flywheel_pwm.duty_u16(0)
        flywheel_active = False
        return

    if elapsed < t_ramp_end:  # give the flywheel time to ramp up to max speed
        duty = int(65535 * elapsed / ramp_up_time_ms)
        flywheel_pwm.duty_u16(duty)
    elif elapsed < t_hold_end:
        if crank_ready:
            flywheel_pwm.duty_u16(65535)
            startCrankShaft()
            crank_ready = False
    else:
        flywheel_pwm.duty_u16(0)  # stop the flywheel, but keep the active flag true so we can cooldown

def initializePlatformMotor():
    """Initialize platform motor pins and PWM."""
    global platform_pwm, platform_in1, platform_in2

    platform_in1 = Pin(10, Pin.OUT)
    platform_in2 = Pin(11, Pin.OUT)
    platform_pwm = PWM(Pin(9))
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


def initializeCrankShaft():
    """Initialize 4-phase stepper pins for the crank shaft motor."""
    global crank_pins
    crank_pins = [
        Pin(17, Pin.OUT),  # IN1
        Pin(16, Pin.OUT),  # IN2
        Pin(15, Pin.OUT),  # IN3
        Pin(14, Pin.OUT),  # IN4
    ]

    # Start with all coils off
    for p in crank_pins:
        p.value(0)


def startCrankShaft():
    """
    Blocking: one full rotation, full steps, 1 ms per step. From ~60% of steps, flywheel PWM off.
    """
    global flywheel_pwm

    total_steps = total_crank_time_ms
    step_delay_ms = CRANK_STEP_DELAY_MS
    flywheel_off_at = int(total_steps * 0.6)

    sequence = [
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0, 0, 0, 1),
    ]

    for step in range(total_steps):
        if step >= flywheel_off_at:
            flywheel_pwm.duty_u16(0)
        pattern = sequence[step % 4]
        for pin, value in zip(crank_pins, pattern):
            pin.value(value)
        time.sleep_ms(step_delay_ms)

    # Release coils so motor does not hold torque
    for p in crank_pins:
        p.value(0)
