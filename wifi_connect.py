#importing the libraries
import network
import urequests
import time
import secrets
 
#setting up SSID and password 
 
SSID = secrets.SSID #use tufts_eecs
PASSWORD = secrets.PASSWORD #foundedin1883


#function definition 
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
    print("Connected! IP address:", wlan.ifconfig()[0])
    return wlan
 
 #function call
connect_wifi()

ISS_URL = "http://api.open-notify.org/iss-now.json"

response = urequests.get(ISS_URL)
data = response.json()
response.close()

print(data)