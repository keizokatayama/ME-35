import neopixel #importing the library
import time
from machine import Pin # another way of importing a library
lights = neopixel.NeoPixel(Pin(15),4)# 0 is the Pin for neopixel and 4 is the number of lights
lights[0] = (20,0,20)
lights[1] = (25,30,0)# set the color of 0th light to purple
lights.write()

