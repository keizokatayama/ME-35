# encoder.py -- quadrature encoder + motor/servo helpers for MicroPython (ESP32)
#
# Copy this file to the board (same folder as your main program) and import it:
#     import encoder
#     count = encoder.Count(32, 39)
#
# The demo at the bottom only runs when you run this file directly, not when
# another program imports it.

from machine import Pin
from machine import PWM

class Count(object):
    def __init__(self, A, B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0

        self.A.irq(self.cb, Pin.IRQ_FALLING | Pin.IRQ_RISING)  # interrupt on line A
        self.B.irq(self.cb, Pin.IRQ_FALLING | Pin.IRQ_RISING)  # interrupt on line B

    def cb(self, msg):
        other, inc = (self.B, 1) if msg == self.A else (self.A, -1)  # define other line and increment
        self.counter += -inc if msg.value() != other.value() else inc

    def value(self):
        return self.counter

    def reset(self):
        self.counter = 0


class Servo(object):
    """Standard hobby servo on one signal pin (50 Hz, 1-2 ms pulse)."""

    PERIOD_US = 20000   # 50 Hz
    
    def __init__(self, signal):
        self.pwm = PWM(Pin(signal), freq=50, duty_u16=0)

    def angle(self, degrees):
        degrees = max(0, min(180, degrees))
        us = self.MIN_US + degrees * (self.MAX_US - self.MIN_US) / 180
        self.pwm.duty_u16(int(us * 65535 / self.PERIOD_US))

    def off(self):
        self.pwm.duty_u16(0)  # stop sending pulses: servo goes limp


if __name__ == "__main__":
    # Run this file by itself to just read one encoder
    import time
    count = Count(32, 39)
    count2 = Count(25, 33)
    while True:
        print(count.value())
        print(count2.value())
        time.sleep(0.1)
    