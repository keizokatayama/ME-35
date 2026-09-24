from machine import Pin, PWM
import time

servo = PWM(Pin(4), freq=50)
servo.duty_u16(4915)