import serial

class sPort:
    def __init__(self):
        # When using with Raspberry Pi 4
        self.ser_rp4 = serial.Serial('/dev/ttyAMA1', 115200, timeout=None)
    
    # Method to send a packet
    def send_packet(self, packet):
        self.ser_rp4.write(packet)

    # Close the serial port
    def close_sPort(self):
        self.ser_rp4.close()

class RS304MD:
    def __init__(self, id):
        self.id = id
        self.port = sPort()

    # Method to change torque state (on: 1, off: 0, brake: 2)
    def trq_set(self, status):
        id_str = "00" + str(self.id)
        num_str = "00" + str(status)
        packet = ['0xFA', '0xAF', '0x' + id_str[-2:], '0x00', '0x24', '0x01', '0x01', '0x' + num_str[-2:]]
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    # Method to rotate the servo to the target position with specified time
    def control_time_rotate(self, angle, t):
        a = -angle * 10
        if a < 0:
            a = 65536 + a
        a = ("0000" + format(int(a), 'x'))[-4:]
        an = []
        an.append('0x' + a[-2:])
        an.append('0x' + a[:2])
        t = ("0000" + format(int(t * 100), 'x'))[-4:]
        tn = []
        tn.append('0x' + t[-2:])
        tn.append('0x' + t[:2])
        ID = "00" + str(self.id)
        packet = ['0xFA', '0xAF', '0x' + ID[-2:], '0x00', '0x1E', '0x04', '0x01'] + an + tn
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    # Method to move the servo to the target position
    def control_rotate(self, angle):
        a = -angle * 10
        if a < 0:
            a = 65536 + a
        a = ("0000" + format(int(a), 'x'))[-4:]
        an = []
        an.append('0x' + a[-2:])
        an.append('0x' + a[:2])
        ID = "00" + str(self.id)
        packet = ['0xFA', '0xAF', '0x' + ID[-2:], '0x00', '0x1E', '0x02', '0x01'] + an
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    # Function to change servo ID
    def change_id(self, new_id):
        old_id = "00" + str(self.id)
        new_id = "00" + str(new_id)
        packet = ['0xFA', '0xAF', '0x' + old_id[-2:], '0x00', '0x04', '0x01', '0x01', '0x' + new_id[-2:]]
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        # Write to flash ROM
        packet = ['0xFA', '0xAF', '0x' + new_id[-2:], '0x40', '0xff', '0x00', '0x00']
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        # Create a packet to reboot the servo
        packet = ['0xFA', '0xAF', '0x' + new_id[-2:], '0x20', '0xff', '0x00', '0x00']
        checksum = 0
        for num in range(2, len(packet)):
            checksum ^= int(packet[num], 0)
        packet.append(hex(checksum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)
