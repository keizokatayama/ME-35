# arm_ik.py -- two-link arm: inverse kinematics + servo control (MicroPython, ESP32)
#
# Draws a vertical line down the y-axis, starting with the arm straight up at (0, 167.52).
# Self-contained: no other files needed.

from machine import Pin, PWM
import math
import time

# ---- Arm setup ------------------------------------------------------------
L1 = 87.52   # mm, first link
L2 = 80      # mm, second link

# Vertical line from straight up (0, 167.52) down to (0, 122.52), 5 mm steps.
# With 90 = straight on the elbow, the elbow can bend at most 90 degrees, so the
# lowest reachable point on this line is about y = 118.6.
points = [(-21.00,121.00),(-16.83,131.00),(-12.28,131.00),(-6.21,130.30),(0.62,131.00),(8.21,132.00),(15.79,132.00),(23.38,131.70),(30.21,130.00),(33.24,127.00),(35.52,122.00),(37.03,119.20),(38.55,119.50),(39.31,125.00),(39.69,133.00),(40.45,140.00),(43.10,142.00),(46.14,143.50),(48.79,146.00),(47.66,147.50),(41.59,145.00),(33.24,142.50),(23.38,140.00),(12.00,139.00),(0.62,139.00),(-10.76,140.00),(-20.62,141.00),(-27.45,145.00),(-30.48,144.00),(-33.52,140.00),(-34.66,133.00),(-33.52,126.00),(-30.86,124.00),(-27.83,127.00),(-25.93,135.00),(-27.45,145.00),(-29.72,149.50),(-44.90,141.00),(-54.00,134.50),(-34.66,133.00)]
PAUSE = 0.5    # seconds to wait at each point
ELBOW_UP = True

# ---- Pins -----------------------------------------------------------------
SERVO1_PIN = 4    # shoulder servo
SERVO2_PIN = 5    # elbow servo
LED_PIN = 26      # status LED: on while the arm is running

# ---- Servo calibration (from the 0-180 sweep) ------------------------------
MIN_US = 500      # pulse for 0 degrees
MAX_US = 2500     # pulse for 180 degrees

# Servo 1: 0 = right (+x), 90 = up (+y)        -> servo angle = theta1
OFFSET1 = 0
DIR1 = 1
# Servo 2: 90 = straight, 180 = tilts left (CCW) -> servo angle = 90 + theta2
OFFSET2 = 90
DIR2 = 1
# ---------------------------------------------------------------------------


class Servo(object):
    """Standard hobby servo on one signal pin (50 Hz)."""

    PERIOD_US = 20000   # 50 Hz

    def __init__(self, signal, min_us=MIN_US, max_us=MAX_US):
        self.min_us = min_us
        self.max_us = max_us
        self.pwm = PWM(Pin(signal), freq=50, duty_u16=0)

    def angle(self, degrees):
        degrees = max(0, min(180, degrees))
        us = self.min_us + degrees * (self.max_us - self.min_us) / 180
        self.pwm.duty_u16(int(us * 65535 / self.PERIOD_US))

    def off(self):
        self.pwm.duty_u16(0)  # stop sending pulses: servo goes limp


def ik(x, y, l1, l2, elbow_up=True):
    """Return (theta1, theta2) in degrees, or None if (x, y) is out of reach."""
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # floating-point rounding at full reach can give 1.0000000000000004
    if 1 < c2 < 1.000001:
        c2 = 1.0
    elif -1.000001 < c2 < -1:
        c2 = -1.0
    if c2 < -1 or c2 > 1:
        return None
    s2 = math.sqrt(1 - c2**2)
    if not elbow_up:
        s2 = -s2  # the +/- picks the elbow configuration
    theta2 = math.atan2(s2, c2)
    theta1 = math.atan2(y, x) - math.atan2(l2 * s2, l1 + l2 * c2)
    return math.degrees(theta1), math.degrees(theta2)


def to_servo(theta, offset, direction):
    """Convert a joint angle to a servo angle, or None if outside 0-180."""
    a = offset + direction * theta
    if a < 0 or a > 180:
        return None
    return a


servo1 = Servo(SERVO1_PIN)
servo2 = Servo(SERVO2_PIN)
led = Pin(LED_PIN, Pin.OUT)

try:
    led.value(0.7)
    for x, y in points:
        result = ik(x, y, L1, L2, ELBOW_UP)
        if result is None:
            print("({}, {}) is out of reach, skipping".format(x, y))
            continue
        theta1, theta2 = result
        a1 = to_servo(theta1, OFFSET1, DIR1)
        a2 = to_servo(theta2, OFFSET2, DIR2)
        if a1 is None or a2 is None:
            print("({}, {}) needs a servo angle outside 0-180, skipping".format(x, y))
            continue
        print("({}, {:.2f}) -> theta1 = {:.2f}, theta2 = {:.2f} | servo1 = {:.1f}, servo2 = {:.1f}"
              .format(x, y, theta1, theta2, a1, a2))
        servo1.angle(a1)
        servo2.angle(a2)
        time.sleep(PAUSE)
finally:
    servo1.off()
    servo2.off()
    led.value(0)