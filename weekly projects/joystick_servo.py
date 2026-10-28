# Joystick -> Servo "time hand" (MicroPython on ESP32, run from Thonny)
# Joystick X axis on GPIO32, Y axis on GPIO39, servo signal on GPIO4.

from machine import Pin, ADC, PWM
import time

JOY_X_PIN = 32          # left/right axis (swap with 39 if directions seem wrong)
JOY_Y_PIN = 39          # not used yet
SERVO_PIN = 4

DEADZONE = 300          # ignore small drift around center (ADC units, 0-4095)
MAX_SPEED_DPS = 90.0    # degrees/second at full deflection
MIN_ANGLE = 10.0        # stay away from the servo's end stops
MAX_ANGLE = 170.0
INVERT = False          # set True if "right" turns the arm the wrong way

# --- Setup ---
joy_x = ADC(Pin(JOY_X_PIN))
joy_x.atten(ADC.ATTN_11DB)      # full 0-3.3V range
joy_x.width(ADC.WIDTH_12BIT)    # 0-4095

servo = PWM(Pin(SERVO_PIN), freq=50)


def set_servo(angle):
    """Move servo to angle (0-180 degrees)."""
    pulse_us = 500 + (angle / 180.0) * (2400 - 500)    # 500-2400 us pulse
    servo.duty_u16(int(pulse_us / 20000 * 65535))       # 20 ms period at 50 Hz


def read_x():
    """Average a few samples to reduce ADC noise."""
    total = 0
    for _ in range(8):
        total += joy_x.read()
    return total // 8


def clamp(value, low, high):
    return max(low, min(high, value))


angle = MIN_ANGLE
set_servo(angle)

# Calibrate center: leave the joystick untouched at start
time.sleep_ms(300)
total = 0
for _ in range(50):
    total += joy_x.read()
    time.sleep_ms(5)
joy_center = total // 50
print("Joystick center =", joy_center)

last_update = time.ticks_ms()

# --- Main loop ---
while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_update) / 1000.0
    last_update = now

    offset = read_x() - joy_center

    if abs(offset) > DEADZONE:
        # Scale deflection (beyond deadzone) to -1.0 .. +1.0
        span = 4095 - joy_center - DEADZONE
        norm = clamp((abs(offset) - DEADZONE) / span, 0.0, 1.0)
        if offset < 0:
            norm = -norm
        if INVERT:
            norm = -norm

        # Right = increase angle, left = decrease. Harder push = faster.
        angle += norm * MAX_SPEED_DPS * dt
        angle = clamp(angle, MIN_ANGLE, MAX_ANGLE)
        set_servo(angle)

    print("offset:", offset, " angle:", round(angle, 1))
    time.sleep_ms(20)      # ~50 Hz update