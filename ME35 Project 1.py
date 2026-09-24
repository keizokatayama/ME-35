from machine import Pin, PWM
import time
from time import ticks_ms, ticks_diff
import network
import urequests
import time
import wifi
import urequests

button1 = Pin(23, Pin.IN, Pin.PULL_UP) 
button2 = Pin(22, Pin.IN, Pin.PULL_UP)
pwm1 = PWM(Pin(4), freq = 50)
pwm2 = PWM(Pin(5), freq = 50)
red = PWM(Pin(18), freq = 1000)
green = PWM(Pin(19), freq = 1000)
blue = PWM(Pin(21), freq = 1000)

DEBOUNCE_MS = 200
last_press_1 = 0
last_press_2 = 0

city = -1;
update_needed = True;
# city 1: Boston, city 2: London, city 3: Tokyo, city 4: Taipei, city 5: LA

def set_rgb(r, g, b):
    red.duty_u16(int(((255-r) / 255) * 65535))
    green.duty_u16(int(((255-g) / 255) * 65535))
    blue.duty_u16(int(((255-b) / 255) * 65535))

set_rgb(0, 0, 0)

def weather_handler(pin):
    global last_press_1
    global state
    global city
    global update_needed
    now = ticks_ms()
    if ticks_diff(now, last_press_1) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press_1 = now
            state = 1
            if (city >= 4):
                city = 0
            else:
                city = city + 1
            update_needed = True

def timer_handler(pin):
    global last_press_2
    global state
    now = ticks_ms()
    global update_needed
    if ticks_diff(now, last_press_2) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press_2 = now
            state = 0
            set_rgb(0, 0, 0)
            update_needed = True

button1.irq(trigger=Pin.IRQ_FALLING, handler=weather_handler)
button2.irq(trigger=Pin.IRQ_FALLING, handler=timer_handler)

state = 0
set_rgb(0, 0, 0)
wifi.connect_wifi()
set_rgb(0, 0, 0)
print(state)
set_rgb(0, 0, 0)

while True:
    if update_needed:
        if (state == 0):
            set_rgb(0, 0, 0)
            DATE_URL = "https://aisenseapi.com/services/v1/datetime/-0400"
            reply = urequests.get(DATE_URL)
            string = reply.json()['datetime']
            hour = string[11:13]
            hour_int = int(hour)
            minute = string[14:16]
            minute_int = int(minute)
            if (hour_int > 12):
                hour_int = hour_int - 12 
                print(hour_int)
                print(minute)
                
            else:
                print(hour_int)
                print(minute_int)

            servo_h = 1638 + ((hour_int - 1)/11)*6554
            servo_h = round(servo_h)
            servo_m = 1638 + ((180 - (3*minute_int))/180)*6554
            servo_m = round(servo_m)
            print(servo_h)
            print (servo_m)
            pwm1.duty_u16(servo_h)
            pwm2.duty_u16(servo_m)
        elif (state == 1):
            if (city == 0):
                WEATHER_URL = "https://api.open-meteo.com/v1/forecast?latitude=51.5008&longitude=-0.1247&daily=weather_code&timezone=auto"
            elif (city == 1):
                WEATHER_URL = "https://api.open-meteo.com/v1/forecast?latitude=34.1341&longitude=-118.3215&daily=weather_code&timezone=auto"
            elif (city == 2):
                WEATHER_URL = "https://api.open-meteo.com/v1/forecast?latitude=42.4063&longitude=71.1193&daily=weather_code&timezone=auto"
            elif (city == 3):
                WEATHER_URL = "https://api.open-meteo.com/v1/forecast?latitude=25.0339&longitude=121.5645&daily=weather_code&timezone=auto"
            else:
                WEATHER_URL = "https://api.open-meteo.com/v1/forecast?latitude=35.6762&longitude=139.6503&daily=weather_code&timezone=auto"
            reply = urequests.get(WEATHER_URL)
            weather_code = reply.json()['daily']['weather_code'][0]
            weather_state = 0;
            if (weather_code <= 1):
                weather_state = 0
                set_rgb(128, 60, 0)
            elif (weather_code <= 48): 
                weather_state = 1
                set_rgb(60, 70, 80)
            else:
                weather_state = 2
                set_rgb(0, 25, 128)
                
            print(city, weather_state)
            
            if (city == 0):
                pwm1.duty_u16(1638)
            elif (city == 1):
                pwm1.duty_u16(3277)
            elif (city == 2):
                pwm1.duty_u16(4915)
            elif (city == 3):
                pwm1.duty_u16(6554) 
            else:
                pwm1.duty_u16(8192)
                
            if (weather_state == 0):
                pwm2.duty_u16(8192)
            elif (weather_state == 1):
                pwm2.duty_u16(4915)
            else:
                pwm2.duty_u16(1638)
            
        update_needed = False
    time.sleep_ms(100)
