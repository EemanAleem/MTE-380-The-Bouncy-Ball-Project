from smbus2 import SMBus

# Set bus address to 0x8
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1. Use this bus.

number = 1 # bool flag to stop while loop below

print("Enter 1 for ON or 0 for OFF")

# if user inputs an integer other than 1 or 0, program ends
while number == 1:
    ledstate = input(">>>>>>>     ")
    # Switch on
    if ledstate == "1":
	  # Sends a byte of integer 1 to address ‘addr’
        bus.write_byte(addr, 1)
    # Switch off
    elif ledstate == "0":
        bus.write_byte(addr, 0)
    else:
        number = 0
