import neopixel
import time
from machine import Pin

lights = neopixel.NeoPixel(Pin(15), 4)

x, y, z = 0, 0, 0  # start all channels at 0 (off)

while True:
# Fade IN (increase brightness)
    for i in range(0, 51):  # go from 0 up to 50
        x = i
        y = 0
        z = i
        for pixel in range(4):
            lights[pixel] = (x, y, z)
        lights.write()
        time.sleep(0.02)

    # Fade OUT (decrease brightness)
    for i in range(50, -1, -1):  # go from 50 down to 0
        x = i
        y = 0
        z = i
        for pixel in range(4):
            lights[pixel] = (x, y, z)
        lights.write()
        time.sleep(0.02)

