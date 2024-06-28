from smbus2 import SMBus
import time

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1
time.sleep(0.1) #short delay to allow I2C to settle

numb = 1

print("Enter 1 for ON or 0 for OFF")

while numb == 1:
    ledstate = input(">>>>>>>     ")
    # Switch on
    if ledstate == "1":
        bus.write_byte(addr, 0x1)
    # Switch off
    elif ledstate == "0":
        bus.write_byte(addr, 0x0)
    else:
        numb = 0
