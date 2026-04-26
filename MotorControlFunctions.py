from machine import Pin, PWM
import time

# Flywheel sequence state (non-blocking; updated by tryFlywheel each main-loop call)
flywheel_active = False
flywheel_t0 = 0

# Crank: one blocking run per flywheel hold; step count ~= duration ms at 1 ms/step
crank_ready = True
total_crank_time_ms = 2048  # full steps per revolution (28BYJ-48 typical)
CRANK_STEP_DELAY_MS = 1  # must match startCrankShaft()


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
    When flywheel_active is False: starts the flywheel (sets t0).
    When True: ramp / hold using elapsed time, then blocking cooldown.
    After cooldown ends, flywheel_active becomes False and the next call can start again.

    Ramp (non-blocking), then hold: one blocking crank rotation (startCrankShaft),
    then blocking cooldown sleep.

    
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

    # Hold window = time the crank shaft runs (steps * ms per step), after ramp ends
    t_ramp_end = ramp_up_time_ms
    crank_duration_ms = total_crank_time_ms * CRANK_STEP_DELAY_MS
    t_hold_end = t_ramp_end + crank_duration_ms
    # conditions after activation
    if elapsed < t_ramp_end:  # give the flywheel some time to ramp up to max speed
        duty = int(65535 * elapsed / ramp_up_time_ms)
        flywheel_pwm.duty_u16(duty)
    elif elapsed < t_hold_end: # after ramp, we start the crank shaft
        if crank_ready: # only start the crank shaft if it is ready (hasn't already been started)
            flywheel_pwm.duty_u16(65535)
            startCrankShaft() 
            crank_ready = False # set the crank shaft to not ready so we don't start it again
            flywheel_pwm.duty_u16(0)
            time.sleep_ms(cooldown_time_ms)  # blocking cooldown
            flywheel_active = False
            return
    else:
        flywheel_pwm.duty_u16(0)


def initializeCrankShaft():
    """Initialize 4-phase stepper pins for the crank shaft motor."""
    global crank_pins
    crank_pins = [
        Pin(13, Pin.OUT),  # IN1
        Pin(12, Pin.OUT),  # IN2
        Pin(11, Pin.OUT),  # IN3
        Pin(10, Pin.OUT),  # IN4
    ]

    # Start with all coils off
    for p in crank_pins:
        p.value(0)


def startCrankShaft():
    """
    Blocking: one full rotation, full steps, 1 ms per step. From ~60% of steps, flywheel PWM off.
    """
    global flywheel_pwm # we need to turn off the flywheel during the crank shaft rotation

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
        if step >= flywheel_off_at: # after 60% of the steps, we turn off the flywheel and continue with the crank shaft rotation to fully reset it
            flywheel_pwm.duty_u16(0)

        # rotate the crank shaft following the pattern
        pattern = sequence[step % 4] 
        for pin, value in zip(crank_pins, pattern):
            pin.value(value)
        time.sleep_ms(step_delay_ms)

    # Release coils so motor does not hold torque
    for p in crank_pins:
        p.value(0)



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