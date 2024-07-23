from smbus2 import SMBus
from time import sleep
import struct

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1

steps = [600, 600, 600]
speeds = [500, 500, 500]
accels = [200, 200, 200]
ByteSteps = [0,0,0]
ByteSpeeds = [0,0,0]
ByteAccels = [0,0,0]

for i in range(0,3):
    ByteSteps[i] = list(struct.pack('>h', steps[i]))
    ByteSpeeds[i] = list(struct.pack('>h', speeds[i]))
    ByteAccels[i] = list(struct.pack('>h', accels[i]))
    
#     print(f"ByteSteps ", i, " = ", ByteSteps[i])
#     print(f"ByteSpeeds ", i, " = ",  ByteSpeeds[i])
#     print(f"ByteAccels ", i, " = ", ByteAccels[i])

while True:
    for i in range (0,3):
        bus.write_i2c_block_data(addr, 0, ByteSteps[i])
        sleep(0.001)
        bus.write_i2c_block_data(addr, 0, ByteSpeeds[i])
        sleep(0.001)
        bus.write_i2c_block_data(addr, 0, ByteAccels[i])
        sleep(0.001)
    sleep(2)
