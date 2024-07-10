import class_servo as cs
import math
import time

class BBrobot:
    # Constructing the robot's structure
    def __init__(self, ids):  # Receives a list of servo IDs
        # Prepare the serial port
        self.port = cs.sPort()
        # Prepare the servos
        self.s1 = cs.RS304MD(ids[0])
        self.s2 = cs.RS304MD(ids[1])
        self.s3 = cs.RS304MD(ids[2])
        # Link lengths L = [bottom, lower link, upper link, ceiling]
        self.L = [2.0, 1.75, 4.0, 3.125]
        # Initial position (theta, phi, Pz)
        self.ini_pos = [0, 0, 0.0632]
        self.pz_max = 0.0732
        self.pz_min = 0.0532
        self.phi_max = 20


    # Method to set up the robot
    def set_up(self):
        # Turn on the torque of the servos
        self.s1.trq_set(1)
        self.s2.trq_set(1)
        self.s3.trq_set(1)
        
    
    # Method to clean up the robot
    def clean_up(self):
        # Turn off the torque of the servos
        self.s1.trq_set(0)
        self.s2.trq_set(0)
        self.s3.trq_set(0)
        # Clean up the serial port
        self.port.close_sPort()

    # Method to calculate inverse kinematics
    def kinema_inv(self, n, Pz):
        L = self.L
        # Calculate the height Pmz at servo reference time (inverted with Pmz at +-)
        A = (L[0] + L[1]) / Pz
        B = (Pz ** 2 + L[2] ** 2 - (L[0] + L[1]) ** 2 - L[3] ** 2) / (2 * Pz)
        C = A ** 2 + 1
        D = 2 * (A * B - (L[0] + L[1]))
        E = B ** 2 + (L[0] + L[1]) ** 2 - L[2] ** 2
        Pmx = (-D + math.sqrt(D ** 2 - 4 * C * E)) / (2 * C)
        Pmz = math.sqrt(L[2] ** 2 - Pmx ** 2 + 2 * (L[0] + L[1]) * Pmx - (L[0] + L[1]) ** 2)

        # Calculate angle of servo a
        a_m_x = (L[3] / (math.sqrt(n[0] ** 2 + n[2] ** 2))) * (n[2])
        a_m_y = 0
        a_m_z = Pz + (L[3] / (math.sqrt(n[0] ** 2 + n[2] ** 2))) * (-n[0])
        A_m = [a_m_x, a_m_y, a_m_z]
        A = (L[0] - A_m[0]) / A_m[2]
        B = (A_m[0] ** 2 + A_m[1] ** 2 + A_m[2] ** 2 - L[2] ** 2 - L[0] ** 2 + L[1] ** 2) / (2 * A_m[2])
        C = A ** 2 + 1
        D = 2 * (A * B - L[0])
        E = B ** 2 + L[0] ** 2 - L[1] ** 2
        ax = (-D + math.sqrt(D ** 2 - 4 * C * E)) / (2 * C)
        ay = 0
        az = math.sqrt(L[1] ** 2 - ax ** 2 + 2 * L[0] * ax - L[0] ** 2)
        if (a_m_z < Pmz):
            az = -az
        A_2 = [ax, ay, az]
        theta_a = 90 - math.degrees(math.atan2(A_2[0] - L[0], A_2[2]))

        # Calculate angle of servo b
        b_m_x = (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (-n[2])
        b_m_y = (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (-math.sqrt(3) * n[2])
        b_m_z = Pz + (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (math.sqrt(3) * n[1] + n[0])
        B_m = [b_m_x, b_m_y, b_m_z]

        A = -(B_m[0] + math.sqrt(3) * B_m[1] + 2 * L[0]) / B_m[2]
        B = (B_m[0] ** 2 + B_m[1] ** 2 + B_m[2] ** 2 + L[1] ** 2 - L[2] ** 2 - L[0] ** 2) / (2 * B_m[2])
        C = A ** 2 + 4
        D = 2 * A * B + 4 * L[0]
        E = B ** 2 + L[0] ** 2 - L[1] ** 2
        x = (-D - math.sqrt(D ** 2 - 4 * C * E)) / (2 * C)
        y = math.sqrt(3) * x
        z = math.sqrt(L[1] ** 2 - 4 * x ** 2 - 4 * L[0] * x - L[0] ** 2)
        if (b_m_z < Pmz):
            z = -z
        B_2 = [x, y, z]
        theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0] ** 2 + B_2[1] ** 2) - L[0], B_2[2]))

        # Calculate angle of servo c
        c_m_x = (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (-n[2])
        c_m_y = (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (math.sqrt(3) * n[2])
        c_m_z = Pz + (L[3] / (math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (-math.sqrt(3) * n[1] + n[0])
        C_m = [c_m_x, c_m_y, c_m_z]

        A = -(C_m[0] - math.sqrt(3) * C_m[1] + 2 * L[0]) / C_m[2]
        B = (C_m[0] ** 2 + C_m[1] ** 2 + C_m[2] ** 2 + L[1] ** 2 - L[2] ** 2 - L[0] ** 2) / (2 * C_m[2])
        C = A ** 2 + 4
        D = 2 * A * B + 4 * L[0]
        E = B ** 2 + L[0] ** 2 - L[1] ** 2
        x = (-D - math.sqrt(D ** 2 - 4 * C * E)) / (2 * C)
        y = -math.sqrt(3) * x
        z = math.sqrt(L[1] ** 2 - 4 * x ** 2 - 4 * L[0] * x - L[0] ** 2)
        if (c_m_z < Pmz):
            z = -z
        C_2 = [x, y, z]
        theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0] ** 2 + C_2[1] ** 2) - L[0], C_2[2]))
        thetas = [theta_a, theta_b, theta_c]
        return thetas

    # Method to achieve posture (theta, phi, Pz) after t seconds
    def control_t_posture(self, pos, t):
        theta = pos[0]
        phi = pos[1]
        # Constraints on movement
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min 
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        angles = self.kinema_inv(n, Pz)

        print(f"theta_a: {angles[0]}")
        print(f"theta_b: {angles[1]}")
        print(f"theta_c: {angles[2]}")

        self.s1.control_time_rotate(angles[0], t)
        self.s2.control_time_rotate(angles[1], t)
        self.s3.control_time_rotate(angles[2], t)
        time.sleep(t)
    
    def Initialize_posture(self):
        pos = self.ini_pos
        t = 1
        self.control_t_posture(pos, t)
