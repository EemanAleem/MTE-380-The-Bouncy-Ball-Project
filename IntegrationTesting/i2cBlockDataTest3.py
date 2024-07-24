from smbus2 import SMBus
from time import sleep
import struct

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1

steps = [1000, 1000, 1000]
speeds = [500, 500, 500]
accels = [200, 200, 200]
ByteSteps = [0,0,0]
ByteSpeeds = [0,0,0]
ByteAccels = [0,0,0]

for i in range(0,3):
    ByteSteps[i] = list(struct.pack('>h', steps[i]))
    ByteSpeeds[i] = list(struct.pack('>h', speeds[i]))
    ByteAccels[i] = list(struct.pack('>h', accels[i]))
    
for i in range (0,3):
    for j in range (0,2):
        print(f"ByteSteps[",i,"][",j,"] = ", ByteSteps[i][j])
        sleep(0.001)
    for j in range (0,2):
        print(f"ByteSpeeds[",i,"][",j,"] = ", ByteSpeeds[i][j])
        sleep(0.001)
    for j in range (0,2):
        print(f"ByteAccels[",i,"][",j,"] = ", ByteAccels[i][j])
        sleep(0.001)

while True:
    for i in range (0,3):
        for j in range (0,2):
            bus.write_byte(addr, ByteSteps[i][j])
            sleep(0.001)
        for j in range (0,2):
            bus.write_byte(addr, ByteSpeeds[i][j])
            sleep(0.001)
        for j in range (0,2):
            bus.write_byte(addr, ByteAccels[i][j])
            sleep(0.001)
    sleep(3)
