from machine import Pin
import neopixel
import time
from time import ticks_ms, ticks_diff

lights = neopixel.NeoPixel(Pin(15), 4)

btn = Pin(34, Pin.IN, Pin.PULL_UP) 
DEBOUNCE_MS = 200
last_press = 0

def button_handler(pin):
    global last_press
    now = ticks_ms()
    if ticks_diff(now, last_press) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press = now
            print("Button pressed!")
            lights[0] = (20, 20, 20)
            lights.write()
            time.sleep(0.2)

btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

while True:
    lights[0] = (0, 0, 0)
    lights.write()

