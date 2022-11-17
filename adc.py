import smbus
import time
address = 0x48
A0 = 0x40 #photoresistor
A1 = 0x41 #nothing
A2 = 0x42 #thermistor
A3 = 0x43 #ptentiometer
bus = smbus.SMBus(1)
while True:
    bus.write_byte(address,A3)
    value = bus.read_byte(address)
    print(value)
    time.sleep(0.1)