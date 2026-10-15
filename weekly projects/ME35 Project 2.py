from machine import Pin, SoftI2C
from machine import PWM          # >>> ADDED: PWM is how we control a servo
import time
import math

button_Play = Pin(34, Pin.IN, Pin.PULL_UP)
button_Train = Pin(35, Pin.IN, Pin.PULL_UP)

i2c = SoftI2C(scl = Pin(22), sda = Pin(21))

print(i2c.scan())

# ===================== ADDED: servo turn =====================
servo_r = PWM(Pin(4), freq=50)    
servo_k = PWM(Pin(5), freq=50)

def set_servo_angle(motor,angle):
    min_duty = 1638.375   # 0.5 ms / 20 ms * 65535
    max_duty = 8191.875   # 2.5 ms / 20 ms * 65535
    duty = int(min_duty + (angle / 180) * (max_duty-min_duty))
    motor.duty_u16(duty)
    print("Servo angle:", angle)

    
set_servo_angle(servo_r,0)
set_servo_angle(servo_k,65)


 
# ==============================================================

DEBOUNCE_MS = 200
last_press = 0

pressed_flag = False
STATE_PLAY = False
STATE_TRAIN = True

def playButton(p):
    global STATE_PLAY
    global STATE_TRAIN
    STATE_PLAY = True
    STATE_TRAIN = False


def trainButton(p):
    global pressed_flag
    global STATE_TRAIN
    global last_press                                   # >>> NEW
    now = time.ticks_ms()                               # >>> NEW: current time in ms
    if time.ticks_diff(now, last_press) < DEBOUNCE_MS:  # >>> NEW: too soon after last press?
        return                                          # >>> NEW: ignore it (button bounce)
    last_press = now                                    # >>> NEW
    STATE_TRAIN = True
    pressed_flag = True

    
button_Train.irq(trigger=Pin.IRQ_RISING, handler=trainButton)
button_Play.irq(trigger=Pin.IRQ_RISING, handler=playButton)


import veml6040
sensor = veml6040.VEML6040(i2c)

sensor.trigger_measurement()
   
def k_nearest_neighbor(x,y,z, k =1):
    distances = []
    for index, d in enumerate(data):
        dist = math.sqrt((x-d[0])**2+(y-d[1])**2+(z-d[2])**2)
        distances.append([dist,d[3]])
    #print("distances", distances)
    distances.sort()
    distances = distances[:k] #get k distances
    classes = []
    for dist in distances:
        classes.append(dist[1])
    print("k classes", classes)
    most_number_of_closest_classes = max(set(classes), key = classes.count)
    print("max classes ", most_number_of_closest_classes)
    
    return most_number_of_closest_classes

   
data = []

# ============ NEW: auto-collect settings ============
COLORS = ["red", "blue", "green"]   # order you'll train in: 1st press, 2nd press, 3rd press
SAMPLES_PER_COLOR = 10
color_index = 0                     # which color we're on (0 = red, 1 = blue, 2 = green)
# ====================================================

while True:
    red, green, blue, white = sensor.read_rgbw()

    # ============ NEW: one press -> 10 readings ============
    if(STATE_TRAIN and pressed_flag):
        if color_index < len(COLORS):
            color = COLORS[color_index]
            print("Collecting", color, "- keep the block still...")
            for i in range(SAMPLES_PER_COLOR):
                r, g, b, w = sensor.read_rgbw()
                data.append((r, g, b, color))
                print(i + 1, r, g, b, color)
                time.sleep(0.2)             # wait so each reading is a fresh measurement
            print("Done with", color)
            color_index = color_index + 1
        else:
            print("All colors trained. Press Play.")
        pressed_flag = False                # reset AFTER collecting, so bounces during collection are ignored
           
    if(STATE_PLAY):
        #print("the colors" , red, green, blue, white)
        what_class = k_nearest_neighbor(red, green, blue,3)
        print(what_class)

        # ============ ADDED: move servo based on the color ============
        if what_class == "red":
            set_servo_angle(servo_r,60)
        elif what_class == "blue":
            set_servo_angle(servo_r,90)
        elif what_class == "green":
            set_servo_angle(servo_r,120)
        # for "no clue" (or anything else), the servo just stays where it is
        
        if what_class == "red" or what_class == "blue" or what_class == "green":
            time.sleep(0.5)                  # give the D4 servo time to arrive first
            set_servo_angle(servo_k, 160) # click: rotate to 30 degrees
            time.sleep(0.5)                  # hold the click briefly
            set_servo_angle(servo_k, 65)  # return, ready for the next click
        # ==============================================================

        time.sleep(0.1)
        STATE_PLAY = False
    time.sleep(0.1)

