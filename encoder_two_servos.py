# servo_follow.py -- turn an encoder by hand, its servo follows.
#
# The DC motors are NOT powered or driven; they are only used as knobs.
# Encoder 1 -> Servo 1, Encoder 2 -> Servo 2.
# Needs encoder.py on the board next to this file.
import encoder_updated
import time

encoder_updated.Servo.MIN_US = 500     # pulse for 0 degrees  (tested: works)
encoder_updated.Servo.MAX_US = 2500    # pulse for 180 degrees (tested: works)

# ---- Pin setup: change these to match your wiring ------------------------
ENC1_A, ENC1_B = 39, 32    # encoder on DC motor 1
ENC2_A, ENC2_B = 25, 33    # encoder on DC motor 2
SERVO1_PIN = 4            # servo 1 signal wire
SERVO2_PIN = 5            # servo 2 signal wire
# --------------------------------------------------------------------------

# Counts for one full turn of the encoder shaft. Measure it: run
# encoder.py by itself, turn the shaft exactly once, read the number.
COUNTS_PER_REV = 4000

# Degrees the servo moves per degree the encoder turns (1 = follow 1:1).
GAIN = 0.5

START_ANGLE = 90           # servo angle when the program starts

SMOOTH = 0.15      # 0-1: lower = smoother but slower to follow
DEADBAND = 2.0    # only move the servo if the angle changed this many degrees

enc1 = encoder_updated.Count(ENC1_A, ENC1_B)
enc2 = encoder_updated.Count(ENC2_A, ENC2_B)
servo1 = encoder_updated.Servo(SERVO1_PIN)
servo2 = encoder_updated.Servo(SERVO2_PIN)

enc1.reset()
enc2.reset()

def counts_to_angle(counts):
    turned = counts * 360 / COUNTS_PER_REV          # encoder_updated angle in degrees
    return max(0, min(180, START_ANGLE + GAIN * turned))

MIN_COUNT = -int(90 / (GAIN * 360 / COUNTS_PER_REV))   # -4000
MAX_COUNT = int(90 / (GAIN * 360 / COUNTS_PER_REV))    # +4000

def clamp_count(enc):
    if enc.counter < MIN_COUNT:
        enc.counter = MIN_COUNT
    elif enc.counter > MAX_COUNT:
        enc.counter = MAX_COUNT

servo1.angle(START_ANGLE)
servo2.angle(START_ANGLE)
s1 = s2 = START_ANGLE         # smoothed angles
last1 = last2 = START_ANGLE   # last angles actually sent to the servos

try:
    while True:
        clamp_count(enc1)
        clamp_count(enc2)
        c1 = enc1.value()
        c2 = enc2.value()

        s1 += SMOOTH * (counts_to_angle(c1) - s1)
        s2 += SMOOTH * (counts_to_angle(c2) - s2)

        if abs(s1 - last1) >= DEADBAND:
            servo1.angle(s1)
            last1 = s1
        if abs(s2 - last2) >= DEADBAND:
            servo2.angle(s2)
            last2 = s2

        print("enc1:", c1, "servo1:", round(last1), "   enc2:", c2, "servo2:", round(last2))
        time.sleep(0.02)
except KeyboardInterrupt:
    servo1.off()
    servo2.off()