from machine import Pin, I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
print(i2c.scan())




from machine import SoftI2C, Pin
i2c = SoftI2C(scl = Pin(22), sda = Pin(21))
print(i2c.scan())