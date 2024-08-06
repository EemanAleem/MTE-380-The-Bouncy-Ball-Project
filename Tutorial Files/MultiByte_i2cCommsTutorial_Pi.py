from smbus2 import SMBus
from time import sleep
import struct

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1
sleep(2)

LargeNum = [500, 1000, 1500]
ByteLargeNum = [0,0,0]

sleep(3)

while True:
    for i in range (0,3):
	# Converts each number into a 2-byte format
        ByteLargeNum[i] = list(struct.pack('>h', LargeNum[i]))
        for j in range (0,2):
	    # Prints each byte of the 2 byte format and sends it to the I2C device
            print("ByteLargeNum[", i, "][", j, "]: ", ByteLargeNum[i][j])
            bus.write_byte(addr, ByteLargeNum[i][j])
            sleep(0.05)
        print("LargeNum[", i, "]: ", LargeNum[i])
        LargeNum[i] = LargeNum[i] + 2
