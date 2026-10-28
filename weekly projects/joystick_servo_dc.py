# Desk-pet phone lock (MicroPython on Cytron ROBO ESP32, run from Thonny)
#
# HOW IT WORKS
#   SETUP     : joystick left/right sweeps the servo hand = sets the timer (1 degree = 1 minute;
#               in TEST_MODE the full 180 degrees = 1 minute).
#   LOCKED    : click starts the timer. DC motor locks the phone, the servo hand sweeps back toward 0
#               as time runs out. Accelerometer watches for movement -> red flash + buzzer.
#   UNLOCKING : timer hit zero. DC motor turns the opposite way (unlock), NeoPixels flash green.
#   then back to SETUP.
#   OVERRIDE  : while LOCKED, the joystick code U U D D L R L R reverses the motor direction.
#               The next click turns the motor (unlock), resets the timer, direction returns to
#               forward, and the system goes back to SETUP.
#
# PINS
#   Joystick      : Grove 6 -> X (left/right + click) = GPIO39, Y (up/down) = GPIO32
#   Servo         : Servo port -> GPIO4
#   DC motor      : MOTOR 1 terminals -> M1A = GPIO12, M1B = GPIO13
#   Encoder       : Grove 1 -> A = GPIO17, B = GPIO16
#   Accelerometer : Grove 2 (I2C) -> SDA = GPIO21, SCL = GPIO22
#   NeoPixels     : on board, GPIO15      Buzzer : on board, GPIO23
#
# ENCODER WIRING WARNING: the FIT0450 encoder is specified for 4.5-7.5 V, so its A/B outputs
# may swing higher than the ESP32's 3.3 V limit. Measure A/B first; use voltage dividers if needed.

from machine import Pin, ADC, PWM, I2C
from neopixel import NeoPixel
import struct
import math
import time

# ---------- Timer settings ----------
TEST_MODE = True                       # True: 1 degree = 1 second, max 60 s. False: 1 degree = 1 minute.
SECONDS_PER_DEGREE = (60.0 / 180.0) if TEST_MODE else 60.0   # test: full 180 deg sweep = 60 s
MAX_TIMER_DEG = 180.0                  # use the servo's full 180 degrees
MIN_START_DEG = 15.0                   # must dial at least this much (5 s in test mode)
DIAL_SPEED_DPS = 60.0 if TEST_MODE else 40.0   # how fast full joystick deflection sweeps the hand
GREEN_FLASH_MS = 3000                  # how long the "unlocked" green flash lasts
SERVO_FLIP = True                      # True: hand ticks the other way (empty = 180 deg, full = 0 deg)
SERVO_MIN_US = 500                     # pulse width at 0 deg   (tune if the servo strains at the ends)
SERVO_MAX_US = 2400                    # pulse width at 180 deg

# ---------- Pins ----------
JOY_X_PIN = 39
JOY_Y_PIN = 32
SERVO_PIN = 4
MOTOR_IN1 = 12           # M1A
MOTOR_IN2 = 13           # M1B
ENC_A_PIN = 17
ENC_B_PIN = 16
I2C_SDA_PIN = 21
I2C_SCL_PIN = 22 
NEOPIXEL_PIN = 15
BUZZER_PIN = 23

# ---------- Joystick settings ----------
DEADZONE = 300
PRESS_THRESHOLD = 3500
FULL_DEFLECTION = 1000
INVERT = False
INVERT_Y = False
INVERT_GESTURE_X = False
SHOW_GESTURES = True
OVERRIDE_INDICATOR = True      # solid amber NeoPixels while the override is armed

# ---------- Secret code (emergency override; only works while LOCKED) ----------
CODE = ['U', 'U', 'D', 'D', 'L', 'R', 'L', 'R']
FLICK_THRESHOLD = 600
GESTURE_STABLE_SAMPLES = 4
SEQ_TIMEOUT_MS = 2000
POST_CLICK_IGNORE_MS = 300

# ---------- Motor / encoder settings ----------
COUNTS_PER_ROTATION = 1920   # CALIBRATE: hand-turn the output shaft one rotation, read enc_count
ROTATIONS = 0.15                                 # rotations to lock / unlock
MOTOR_SPEED = 0.8
MOTOR_DIRECTION = 1          # LOCK direction: 1 or -1 (unlock is always the opposite)
ENCODER_SIGN = 1
MOTOR_TIMEOUT_MS = 4000
DEBOUNCE_MS = 80

# ---------- Motion alarm settings ----------
NUM_PIXELS = 2
MOTION_THRESHOLD_MG = 120
ACCEL_PERIOD_MS = 20
BASELINE_ALPHA = 0.02
BLANK_MS = 500
ALARM_MS = 3000
ALARM_FLASH_MS = 150
BUZZER_FREQ = 2200
ACCEL_DEBUG = False

# ---------- Safety check ----------
used = [JOY_X_PIN, JOY_Y_PIN, SERVO_PIN, MOTOR_IN1, MOTOR_IN2, ENC_A_PIN, ENC_B_PIN,
        I2C_SDA_PIN, I2C_SCL_PIN, NEOPIXEL_PIN, BUZZER_PIN]
if len(set(used)) != len(used):
    raise ValueError("Two devices share a GPIO! Check the pin constants.")

# ---------- Setup: joystick, servo, motor, encoder ----------
joy_x = ADC(Pin(JOY_X_PIN))
joy_y = ADC(Pin(JOY_Y_PIN))
for _adc in (joy_x, joy_y):
    _adc.atten(ADC.ATTN_11DB)
    _adc.width(ADC.WIDTH_12BIT)

servo = PWM(Pin(SERVO_PIN), freq=50)
m1 = PWM(Pin(MOTOR_IN1), freq=5000)
m2 = PWM(Pin(MOTOR_IN2), freq=5000)

enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)
enc_count = 0


def enc_isr(pin):
    global enc_count
    if enc_a.value() != enc_b.value():
        enc_count += ENCODER_SIGN
    else:
        enc_count -= ENCODER_SIGN


enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc_isr)

# ---------- Setup: NeoPixels + buzzer ----------
pixels = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)
buzzer = PWM(Pin(BUZZER_PIN), freq=BUZZER_FREQ, duty_u16=0)


def set_pixels(color):
    for i in range(NUM_PIXELS):
        pixels[i] = color
    pixels.write()


def alarm_outputs(on):
    if on:
        set_pixels((255, 0, 0))
        buzzer.freq(BUZZER_FREQ)
        buzzer.duty_u16(32768)
    else:
        set_pixels((0, 0, 0))
        buzzer.duty_u16(0)


alarm_outputs(False)

# ---------- Setup: LIS3DHTR accelerometer (I2C) ----------
REG_WHO_AM_I = 0x0F
REG_CTRL1 = 0x20
REG_CTRL4 = 0x23
REG_OUT_X_L = 0x28

i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=400000)
found = i2c.scan()
ACC_ADDR = None
for addr in (0x19, 0x18):
    if addr in found:
        ACC_ADDR = addr
        break
if ACC_ADDR is None:
    raise OSError("LIS3DHTR not found on I2C (looked for 0x18/0x19). Devices seen: "
                  + str([hex(a) for a in found]) + ". Check the Grove 2 cable.")

who = i2c.readfrom_mem(ACC_ADDR, REG_WHO_AM_I, 1)[0]
print("Accelerometer at", hex(ACC_ADDR), "WHO_AM_I =", hex(who), "(expected 0x33)")
i2c.writeto_mem(ACC_ADDR, REG_CTRL1, b'\x57')   # 100 Hz, normal mode, X/Y/Z enabled
i2c.writeto_mem(ACC_ADDR, REG_CTRL4, b'\x88')   # block data update, high-res, +/-2 g
time.sleep_ms(20)


def read_accel():
    raw = i2c.readfrom_mem(ACC_ADDR, REG_OUT_X_L | 0x80, 6)
    x, y, z = struct.unpack('<hhh', raw)
    return x >> 4, y >> 4, z >> 4


# ---------- Helper functions ----------
motor_running = False
motor_start_time = 0
motor_start_count = 0


def motor_stop():
    m1.duty_u16(0)
    m2.duty_u16(0)


def motor_run(direction):
    duty = int(MOTOR_SPEED * 65535)
    if direction >= 0:
        m1.duty_u16(duty)
        m2.duty_u16(0)
    else:
        m1.duty_u16(0)
        m2.duty_u16(duty)


def start_motor(direction):
    global motor_running, motor_start_time, motor_start_count
    motor_start_count = enc_count
    motor_start_time = time.ticks_ms()
    motor_run(direction)
    motor_running = True


def set_servo(timer_deg):
    """timer_deg is the logical timer position (0 = empty .. 180 = full)."""
    phys = 180.0 - timer_deg if SERVO_FLIP else timer_deg
    phys = clamp(phys, 0.0, 180.0)
    pulse_us = SERVO_MIN_US + (phys / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    servo.duty_u16(int(pulse_us / 20000 * 65535))


def read_adc(adc):
    total = 0
    for _ in range(8):
        total += adc.read()
    return total // 8


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def classify(x_off, y_off):
    """Return 'L','R','U','D' for a clear move, 'N' for neutral, None for in-between."""
    sx = -x_off if INVERT else x_off
    if INVERT_GESTURE_X:
        sx = -sx
    sy = -y_off if INVERT_Y else y_off
    if abs(sx) < DEADZONE and abs(sy) < DEADZONE:
        return 'N'
    if abs(sx) >= FLICK_THRESHOLD and abs(sx) >= abs(sy):
        return 'R' if sx > 0 else 'L'
    if abs(sy) >= FLICK_THRESHOLD and abs(sy) > abs(sx):
        return 'U' if sy > 0 else 'D'
    return None


# ---------- Initial state ----------
motor_stop()
angle = 0.0                      # servo hand = timer (degrees)
set_servo(angle)

time.sleep_ms(300)
tx = ty = 0
for _ in range(50):
    tx += joy_x.read()
    ty += joy_y.read()
    time.sleep_ms(5)
joy_center = tx // 50
joy_center_y = ty // 50
print("Joystick center X =", joy_center, " Y =", joy_center_y)

target_counts = int(ROTATIONS * COUNTS_PER_ROTATION)
motor_direction = MOTOR_DIRECTION      # current LOCK direction (the secret code flips this)
state = 'SETUP'
lock_dir_used = motor_direction
timer_end = 0
unlock_started = 0
last_servo_update = 0
last_print_s = -1
green_phase = -1
override_armed = False

last_update = time.ticks_ms()
was_pressed = False
last_press_time = 0
last_pressed_ms = time.ticks_add(time.ticks_ms(), -10000)

seq = []
cand = None
cand_count = 0
armed = False
last_gesture_ms = 0

base = None
last_accel_ms = 0
accel_blank_until = time.ticks_add(time.ticks_ms(), 1000)
alarm_active = False
alarm_until = 0
alarm_phase = -1
last_accel_debug = 0

print("Ready. Mode:", "TEST" if TEST_MODE else "NORMAL", "- full sweep =",
      int(MAX_TIMER_DEG * SECONDS_PER_DEGREE), "seconds")

# ---------- Main loop ----------
while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_update) / 1000.0
    last_update = now

    x = read_adc(joy_x)
    y = read_adc(joy_y)
    pressed = x > PRESS_THRESHOLD
    if pressed:
        last_pressed_ms = now
    offset = x - joy_center
    click = pressed and not was_pressed and time.ticks_diff(now, last_press_time) > DEBOUNCE_MS
    if pressed and not was_pressed:
        last_press_time = now
    was_pressed = pressed

    # ======================= STATE: SETUP =======================
    if state == 'SETUP':
        # --- Dial: joystick sweeps the servo hand (the timer) ---
        if not pressed and abs(offset) > DEADZONE:
            span = FULL_DEFLECTION - DEADZONE
            norm = clamp((abs(offset) - DEADZONE) / span, 0.0, 1.0)
            if offset < 0:
                norm = -norm
            if INVERT:
                norm = -norm
            angle = clamp(angle + norm * DIAL_SPEED_DPS * dt, 0.0, MAX_TIMER_DEG)
            set_servo(angle)
            accel_blank_until = time.ticks_add(now, BLANK_MS)
            if time.ticks_diff(now, last_servo_update) > 500:
                print("Timer set to", int(angle * SECONDS_PER_DEGREE), "seconds")
                last_servo_update = now

        # --- Click: start the timer and lock ---
        if click and not motor_running:
            if angle >= MIN_START_DEG:
                lock_dir_used = motor_direction
                timer_end = time.ticks_add(now, int(angle * SECONDS_PER_DEGREE * 1000))
                start_motor(lock_dir_used)
                state = 'LOCKED'
                base = None
                last_print_s = -1
                last_servo_update = 0
                print(">>> LOCKED for", int(angle * SECONDS_PER_DEGREE), "seconds")
            else:
                print("Dial at least", int(MIN_START_DEG * SECONDS_PER_DEGREE),
                      "seconds before starting.")

    # ======================= STATE: LOCKED =======================
    elif state == 'LOCKED':
        remaining_ms = time.ticks_diff(timer_end, now)

        if override_armed and click and not motor_running:
            # Steps 2 + 3: click turns the motor (reverse), resets the timer, direction returns to forward
            angle = 0.0
            set_servo(angle)                     # timer reset: hand back to empty
            start_motor(-lock_dir_used)          # reverse of the lock direction = unlock
            override_armed = False               # next lock uses the normal forward direction again
            state = 'RELEASING'
            print(">>> OVERRIDE CLICK -> unlocking, timer reset")
        elif remaining_ms <= 0 and not motor_running:
            # Timer finished -> unlock (motor reverses) and flash green
            override_armed = False
            angle = 0.0
            set_servo(angle)
            if alarm_active:
                alarm_active = False
                alarm_outputs(False)
            start_motor(-lock_dir_used)
            unlock_started = now
            green_phase = -1
            state = 'UNLOCKING'
            print(">>> TIMER DONE -> UNLOCKING")
        elif remaining_ms > 0:
            # Live countdown on the servo hand
            if time.ticks_diff(now, last_servo_update) >= 100:
                angle = remaining_ms / 1000.0 / SECONDS_PER_DEGREE
                set_servo(angle)
                last_servo_update = now
            secs_left = remaining_ms // 1000
            if secs_left != last_print_s and (TEST_MODE or secs_left % 60 == 0):
                print("Time left:", secs_left, "s")
                last_print_s = secs_left

    # ======================= STATE: UNLOCKING =======================
    elif state == 'UNLOCKING':
        phase = ((now - unlock_started) // ALARM_FLASH_MS) % 2
        if phase != green_phase:
            green_phase = phase
            set_pixels((0, 255, 0) if phase == 0 else (0, 0, 0))
        if time.ticks_diff(now, unlock_started) >= GREEN_FLASH_MS and not motor_running:
            set_pixels((0, 0, 0))
            state = 'SETUP'
            base = None
            accel_blank_until = time.ticks_add(now, BLANK_MS)
            print(">>> UNLOCKED. Back to setup.")

    elif state == 'RELEASING':
        # Override click happened: motor is unlocking. When it stops, back to setup.
        if not motor_running:
            set_pixels((0, 0, 0))
            state = 'SETUP'
            base = None
            accel_blank_until = time.ticks_add(now, BLANK_MS)
            print(">>> RELEASED. Timer reset, direction back to forward. Back to setup.")

    # --- Secret code (only while LOCKED and not already armed) ---
    if state == 'LOCKED' and not override_armed:
        # --- Secret code: arms the emergency override ---
        if pressed or time.ticks_diff(now, last_pressed_ms) < POST_CLICK_IGNORE_MS:
            cand = None
            cand_count = 0
            armed = False
        else:
            g = classify(offset, y - joy_center_y)
            if g == cand:
                cand_count = min(cand_count + 1, 1000)
            else:
                cand = g
                cand_count = 1
            if cand_count == GESTURE_STABLE_SAMPLES:
                if cand == 'N':
                    armed = True
                elif cand in ('L', 'R', 'U', 'D') and armed:
                    armed = False
                    if time.ticks_diff(now, last_gesture_ms) > SEQ_TIMEOUT_MS:
                        seq = []
                    last_gesture_ms = now
                    seq.append(cand)
                    seq = seq[-len(CODE):]
                    if SHOW_GESTURES:
                        print("move:", cand, " sequence:", "".join(seq))
                    if seq == CODE:
                        seq = []
                        override_armed = True            # step 1: motor direction is now REVERSE
                        if alarm_active:
                            alarm_active = False
                            alarm_outputs(False)
                        if OVERRIDE_INDICATOR:
                            set_pixels((255, 140, 0))    # solid amber = override armed
                        print("*** CODE ACCEPTED -> motor direction REVERSED. Click to release.")

    else:
        cand = None
        cand_count = 0
        armed = False


    # --- Motor: stop when the encoder says we've turned far enough ---
    if motor_running:
        accel_blank_until = time.ticks_add(now, BLANK_MS)
        moved = abs(enc_count - motor_start_count)
        timed_out = time.ticks_diff(now, motor_start_time) > MOTOR_TIMEOUT_MS
        if moved >= target_counts:
            motor_stop()
            motor_running = False
            print("Motor done. Counts moved:", moved)
        elif timed_out:
            motor_stop()
            motor_running = False
            print("TIMEOUT. Counts moved:", moved, "- stalled or encoder not wired?")

    # --- Motion alarm: only armed while LOCKED ---
    if time.ticks_diff(now, last_accel_ms) >= ACCEL_PERIOD_MS:
        last_accel_ms = now
        ax, ay, az = read_accel()
        blanking = time.ticks_diff(accel_blank_until, now) > 0

        if state != 'LOCKED' or override_armed or base is None or blanking or alarm_active:
            base = [ax, ay, az]
        else:
            dx = ax - base[0]
            dy = ay - base[1]
            dz = az - base[2]
            delta = math.sqrt(dx * dx + dy * dy + dz * dz)

            if ACCEL_DEBUG and time.ticks_diff(now, last_accel_debug) > 200:
                print("move amount (mg):", int(delta))
                last_accel_debug = now

            if delta > MOTION_THRESHOLD_MG:
                alarm_active = True
                alarm_until = time.ticks_add(now, ALARM_MS)
                alarm_phase = -1
                print("!!! MOVEMENT DETECTED (", int(delta), "mg ) -> ALARM")
            else:
                base[0] += BASELINE_ALPHA * dx
                base[1] += BASELINE_ALPHA * dy
                base[2] += BASELINE_ALPHA * dz

    if alarm_active:
        if time.ticks_diff(now, alarm_until) >= 0:
            alarm_active = False
            alarm_outputs(False)
            base = None
            accel_blank_until = time.ticks_add(now, 300)
            print("Alarm over.")
        else:
            phase = (now // ALARM_FLASH_MS) % 2
            if phase != alarm_phase:
                alarm_phase = phase
                alarm_outputs(phase == 0)

    time.sleep_ms(2)