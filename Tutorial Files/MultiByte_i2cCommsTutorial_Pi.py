from smbus2 import SMBus
from time import sleep
import struct

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1
sleep(2)

steps = [500, 1000, 1500]
ByteSteps = [0,0,0]

sleep(3)

while True:
    for i in range (0,3):
        ByteSteps[i] = list(struct.pack('>h', steps[i]))
        for j in range (0,2):
            print("ByteSteps[", i, "][", j, "]: ", ByteSteps[i][j])
            bus.write_byte(addr, ByteSteps[i][j])
            sleep(0.05)
        print("steps[", i, "]: ", steps[i])
        steps[i] = steps[i] + 2
