# Desk-pet phone lock (MicroPython on Cytron ROBO ESP32, run from Thonny)
#
# HOW IT WORKS
#   SETUP     : joystick left/right sweeps the servo hand = sets the timer (1 degree = 1 minute;
#               in TEST_MODE the full sweep = 1 minute).
#   LOCKED    : click starts the timer. DC motor locks the phone, the servo hand sweeps back toward 0
#               as time runs out. Accelerometer watches for movement -> red flash + buzzer.
#   UNLOCKING : timer hit zero. DC motor turns the opposite way (unlock), NeoPixels flash green.
#   then back to SETUP.
#   OVERRIDE  : while LOCKED, the joystick code U U D D L R L R reverses the motor direction.
#               The next click turns the motor (unlock), resets the timer, direction returns to
#               forward, and the system goes back to SETUP.
#   GREEN KEY : the override code only works after a GREEN brick is shown to the color sensor
#               (short "right" jingle). Any other color plays a short "wrong" buzz. The code window
#               closes after CODE_WINDOW_MS, and then the green brick is needed again.
#
# PINS
#   Joystick      : Grove 6 -> X (left/right + click) = GPIO39, Y (up/down) = GPIO32
#   Servo         : Servo port -> GPIO4
#   DC motor      : MOTOR 1 terminals -> M1A = GPIO12, M1B = GPIO13
#   Encoder       : Grove 1 -> A = GPIO17, B = GPIO16
#   Accelerometer : Grove 2 (I2C) -> SDA = GPIO21, SCL = GPIO22
#   Color sensor  : VEML6040 on Grove 3 (its own software-I2C bus) -> SDA = GPIO26, SCL = GPIO25
#   NeoPixels     : on board, GPIO15      Buzzer : on board, GPIO23
#
# ENCODER WIRING WARNING: the FIT0450 encoder is specified for 4.5-7.5 V, so its A/B outputs
# may swing higher than the ESP32's 3.3 V limit. Measure A/B first; use voltage dividers if needed.

from machine import Pin, ADC, PWM, I2C, SoftI2C
from neopixel import NeoPixel
import struct
import math
import time

# ---------- Timer settings ----------
TEST_MODE = True                       # True: 1 degree = 1 second, max 60 s. False: 1 degree = 1 minute.
SERVO_MIN_DEG = 30.0                   # servo travel limits (physical degrees)
SERVO_MAX_DEG = 165.0
SECONDS_PER_DEGREE = (60.0 / (SERVO_MAX_DEG - SERVO_MIN_DEG)) if TEST_MODE else 60.0   # test: full sweep = 60 s
MAX_TIMER_DEG = SERVO_MAX_DEG - SERVO_MIN_DEG   # 120 degrees of timer travel
MIN_START_DEG = 10.0                   # must dial at least this much (5 s in test mode, 10 min normal)
DIAL_SPEED_DPS = 40.0                  # how fast full joystick deflection sweeps the hand
GREEN_FLASH_MS = 3000                  # how long the "unlocked" green flash lasts
SERVO_FLIP = True                      # True: hand ticks the other way (empty = SERVO_MAX_DEG, full = SERVO_MIN_DEG)
SERVO_MIN_US = 27                     # pulse width at 0 deg   (tune if the servo strains at the ends)
SERVO_MAX_US = 2200                    # pulse width at 180 deg

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
ROTATIONS = 0.15                # rotations to lock / unlock
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

# ---------- Color key settings ----------
CODE_WINDOW_MS = 20000       # after the green brick, how long you have to enter the code
COLOR_PERIOD_MS = 60         # read the color sensor about every 60 ms
COLOR_STABLE_SAMPLES = 5     # a color must hold this many readings (~300 ms) to count
PRESENCE_CHANGE = 0.35       # brick "present" when clear light differs this much from the empty baseline
COLOR_MIN_BASE = 100         # floor for the baseline so a dark room doesn't cause false presence
CHROMA_CHANGE = 0.04          # brick also counts as "present" if the R/G/B mix shifts this much from the empty baseline
GREEN_OVER_RED = 0.95         # green counts / red counts must be at least this (measured: green 1.15, blue 1.12, red 0.72)
GREEN_OVER_BLUE = 1.5         # green counts / blue counts must be at least this (measured: green 1.86, blue 1.17, red 1.76)
USE_WHITE_BALANCE = False     # True: scale channels by the empty-sensor baseline first (thresholds above were measured WITHOUT it)
VEML_IT = 1                  # VEML6040 integration time: 0=40ms 1=80ms 2=160ms 3=320ms (longer = more sensitive)
COLOR_SEPARATE_BUS = True    # True: color sensor on its own bus via Grove 3 (False: share the Maker/Grove 2 bus)
COLOR_SDA_PIN = 26           # Grove 3 white wire.  If the sensor isn't found, try swapping these two.
COLOR_SCL_PIN = 25           # Grove 3 yellow wire
COLOR_DEBUG = False          # True: prints C R G B and what it decided, so you can tune the ratios
RIGHT_TONES = [(1200, 90), (0, 40), (1800, 140)]    # (frequency Hz, ms); 0 Hz = silence
WRONG_TONES = [(300, 160), (0, 60), (220, 260)]
LOCK_TONES = [(660, 90), (0, 30), (440, 90), (0, 30), (330, 220)]   # descending, firm: "locked"
UNLOCK_TONES = [(800, 90), (1000, 90), (1250, 90), (1600, 220)]      # rising, cheerful: "unlocked"
SUCCESS_MELODY = [(523, 110), (0, 25), (659, 110), (0, 25), (784, 110), (0, 25),
                  (659, 90), (0, 25), (784, 110), (0, 25), (1047, 350)]   # little victory tune
OVERRIDE_PLAYS_MELODY = True   # True: the melody also plays when the secret key releases the lock

# ---------- Safety check ----------
used = [JOY_X_PIN, JOY_Y_PIN, SERVO_PIN, MOTOR_IN1, MOTOR_IN2, ENC_A_PIN, ENC_B_PIN,
        I2C_SDA_PIN, I2C_SCL_PIN, NEOPIXEL_PIN, BUZZER_PIN]
if COLOR_SEPARATE_BUS:
    used += [COLOR_SDA_PIN, COLOR_SCL_PIN]
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

# ---------- Non-blocking buzzer jingles ----------
tone_queue = []
tone_end = 0
tone_playing = False


def play_tones(tones):
    global tone_queue, tone_end, tone_playing
    tone_queue = list(tones)
    tone_end = time.ticks_ms()
    tone_playing = True


def tone_update(now):
    global tone_end, tone_playing
    if not tone_playing:
        return
    if time.ticks_diff(now, tone_end) >= 0:
        if tone_queue:
            freq, ms = tone_queue.pop(0)
            if freq > 0:
                buzzer.freq(freq)
                buzzer.duty_u16(32768)
            else:
                buzzer.duty_u16(0)
            tone_end = time.ticks_add(now, ms)
        else:
            buzzer.duty_u16(0)
            tone_playing = False


def tones_length(tones):
    return sum(ms for _, ms in tones)


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


# ---------- Setup: VEML6040 color sensor (I2C address 0x10, written from the datasheet registers) ----------
# Registers (16-bit, low byte first): 0x00 = CONF, 0x08 = RED, 0x09 = GREEN, 0x0A = BLUE, 0x0B = WHITE.
# CONF low byte: bit0 SD (0 = on), bit1 AF (0 = auto), bit2 TRIG, bits6:4 = integration time (IT).
VEML_ADDR = 0x10
VEML_REG_CONF = 0x00
VEML_REG_R = 0x08
VEML_REG_G = 0x09
VEML_REG_B = 0x0A
VEML_REG_W = 0x0B

if COLOR_SEPARATE_BUS:
    ci2c = SoftI2C(sda=Pin(COLOR_SDA_PIN), scl=Pin(COLOR_SCL_PIN), freq=100000)
else:
    ci2c = i2c
cfound = ci2c.scan()
print("Color bus devices:", [hex(a) for a in cfound])
if VEML_ADDR not in cfound:
    raise OSError("VEML6040 not found at 0x10. Devices seen: " + str([hex(a) for a in cfound])
                  + ". Check the cable, try swapping COLOR_SDA_PIN/COLOR_SCL_PIN, and check that "
                  "the module has I2C pull-up resistors.")

ci2c.writeto_mem(VEML_ADDR, VEML_REG_CONF, bytes([(VEML_IT & 0x07) << 4, 0x00]))
time.sleep_ms((40 << VEML_IT) + 30)        # wait for the first measurement
print("VEML6040 ready, integration time =", 40 << VEML_IT, "ms")

WB = [1.0, 1.0, 1.0]                       # white-balance gains (r, g, b), set from the empty-sensor baseline


def veml_read(reg):
    d = ci2c.readfrom_mem(VEML_ADDR, reg, 2)
    return d[0] | (d[1] << 8)


def read_color():
    """Return (white, red, green, blue) raw counts."""
    r = veml_read(VEML_REG_R)
    g = veml_read(VEML_REG_G)
    b = veml_read(VEML_REG_B)
    w = veml_read(VEML_REG_W)
    return w, r, g, b


def classify_color(r, g, b):
    """'GREEN' if green beats both red and blue by the measured margins, otherwise 'OTHER'."""
    if USE_WHITE_BALANCE:
        r = r * WB[0]
        g = g * WB[1]
        b = b * WB[2]
    if g <= 0:
        return 'OTHER'
    g_over_r = g / max(r, 1)
    g_over_b = g / max(b, 1)
    if g_over_r >= GREEN_OVER_RED and g_over_b >= GREEN_OVER_BLUE:
        return 'GREEN'
    return 'OTHER'


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
    """timer_deg is the logical timer position (0 = empty .. MAX_TIMER_DEG = full)."""
    phys = (SERVO_MAX_DEG - timer_deg) if SERVO_FLIP else (SERVO_MIN_DEG + timer_deg)
    phys = clamp(phys, SERVO_MIN_DEG, SERVO_MAX_DEG)
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

# did the alarm go off during this lock session?
session_alarmed = False

# color-key state
code_unlocked = False            # True after a green brick, until CODE_WINDOW_MS runs out
code_window_end = 0
last_color_ms = 0
color_cand = None
color_cand_count = 0
color_latched = None
print("Color sensor: keep the sensor clear (no brick) for calibration...")
_tot = [0, 0, 0, 0]
for _ in range(10):
    _v = read_color()
    for _i in range(4):
        _tot[_i] += _v[_i]
    time.sleep_ms(100)
color_base_c = _tot[0] / 10.0              # white channel = "how much light" baseline
_r0, _g0, _b0 = _tot[1] / 10.0, _tot[2] / 10.0, _tot[3] / 10.0
if min(_r0, _g0, _b0) >= 20:
    WB[0] = _g0 / _r0                      # scale so the empty-sensor light looks neutral (1:1:1)
    WB[1] = 1.0
    WB[2] = _g0 / _b0
else:
    print("Room is very dark - skipping white balance (add light for better color accuracy).")
print("Color baseline (white) =", int(color_base_c), " white-balance gains =",
      [round(w, 2) for w in WB])
_t0 = max(_r0 + _g0 + _b0, 1.0)
color_base_chroma = [_r0 / _t0, _g0 / _t0, _b0 / _t0]
print("Empty-sensor baseline R G B =", int(_r0), int(_g0), int(_b0),
      " g/r:", round(_g0 / max(_r0, 1), 2), " g/b:", round(_g0 / max(_b0, 1), 2))
if classify_color(_r0, _g0, _b0) == 'GREEN':
    print("WARNING: the empty-sensor baseline looks GREEN. Remove any brick and restart the script.")

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
                play_tones(LOCK_TONES)
                session_alarmed = False
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
            play_tones(SUCCESS_MELODY if OVERRIDE_PLAYS_MELODY else UNLOCK_TONES)
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
            if not session_alarmed:
                play_tones(SUCCESS_MELODY)       # clean session, no alarm: victory tune
                print("Clean session - no alarms!")
            else:
                play_tones(UNLOCK_TONES)
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

    # --- Secret code (only while LOCKED and after the green brick) ---
    if state == 'LOCKED' and not override_armed and code_unlocked:
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
                        code_unlocked = False
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

    # --- Color key: green brick opens the code window; other colors get a "wrong" buzz ---
    if state != 'LOCKED' or override_armed:
        code_unlocked = False
    if time.ticks_diff(now, last_color_ms) >= COLOR_PERIOD_MS:
        last_color_ms = now
        cc, cr, cg, cb = read_color()
        ref = max(color_base_c, COLOR_MIN_BASE)
        light_changed = abs(cc - color_base_c) > PRESENCE_CHANGE * ref
        tot = cr + cg + cb
        chroma_dist = 0.0
        if tot > 0:
            chroma_dist = (abs(cr / tot - color_base_chroma[0])
                           + abs(cg / tot - color_base_chroma[1])
                           + abs(cb / tot - color_base_chroma[2]))
        present = light_changed or chroma_dist > CHROMA_CHANGE
        label = None
        if present:
            label = classify_color(cr, cg, cb)
        else:
            color_base_c += 0.01 * (cc - color_base_c)       # follow slow lighting changes
            if tot > 0:
                color_base_chroma[0] += 0.01 * (cr / tot - color_base_chroma[0])
                color_base_chroma[1] += 0.01 * (cg / tot - color_base_chroma[1])
                color_base_chroma[2] += 0.01 * (cb / tot - color_base_chroma[2])
        if COLOR_DEBUG:
            print("W R G B:", cc, cr, cg, cb, " g/r:", round(cg / max(cr, 1), 2),
                  " g/b:", round(cg / max(cb, 1), 2), " baseW:", int(color_base_c),
                  " dChroma:", round(chroma_dist, 3), " present:", present, " ->", label)

        if label == color_cand:
            color_cand_count = min(color_cand_count + 1, 1000)
        else:
            color_cand = label
            color_cand_count = 1

        if color_cand_count == COLOR_STABLE_SAMPLES:
            if label is None:
                color_latched = None                          # brick removed
            elif label != color_latched:
                color_latched = label
                if (state == 'LOCKED' and not override_armed and not code_unlocked
                        and not motor_running and not alarm_active):
                    if label == 'GREEN':
                        code_unlocked = True
                        code_window_end = time.ticks_add(now, CODE_WINDOW_MS)
                        seq = []
                        play_tones(RIGHT_TONES)
                        accel_blank_until = time.ticks_add(now, tones_length(RIGHT_TONES) + 300)
                        print("GREEN accepted -> enter the secret code within",
                              CODE_WINDOW_MS // 1000, "seconds")
                    else:
                        play_tones(WRONG_TONES)
                        accel_blank_until = time.ticks_add(now, tones_length(WRONG_TONES) + 300)
                        print("Wrong color.")

    if code_unlocked and time.ticks_diff(now, code_window_end) >= 0:
        code_unlocked = False
        seq = []
        print("Code window closed. Show the green brick again.")

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
                session_alarmed = True
                tone_playing = False
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

    if not alarm_active:
        tone_update(now)
    time.sleep_ms(2)