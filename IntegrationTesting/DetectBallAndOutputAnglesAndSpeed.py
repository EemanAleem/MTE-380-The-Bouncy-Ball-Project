import math
import serial
from cvzone.SerialModule import SerialObject
from smbus2 import SMBus
import time
from time import sleep
import cv2
import numpy as np

arduino = SerialObject("COM3")
# addr = 0x8 # bus address
# bus = SMBus(1) # indicates /dev/i2c-1

# Global variables initialization
error = [0.0, 0.0]
errorPrev = [0.0, 0.0]
integr = [0.0, 0.0]
deriv = [0.0, 0.0]
out = [0.0, 0.0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
pos = [0, 0, 0]

# Constants
angToStep = 3200 / 360
angOrig = 204
Xoffset = 240  # Replace with actual X offset value
Yoffset = 240  # Replace with actual Y offset value
kp = 4E-4  # Replace with actual proportional gain
ki = 2E-6  # Replace with actual integral gain
kd = 7E-3  # Replace with actual derivative gain
ks = 20  # Replace with actual speed gain

A = 0  # Index for stepper A
B = 1  # Index for stepper B
C = 2  # Index for stepper C

#real-time coordinates
x = 0
y = 0

# What point on the platform do we want the ball to remain at?
setpointX = 0
setpointY = 0


# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(0)
    cap.set(3,600) #set width to 100
    cap.set(4,600) #set height to 100

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        frame = frame[200:400, 200:400]

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if radius > 40:  # Only consider large enough objects
                # Draw a circle around the yellow ball
                # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a dot in the center of the yellow ball
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

        # Display the resulting frame
        cv2.imshow('frame', frame)
        sleep(0.1)
        PID(setpointX, setpointY)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    cv2.destroyAllWindows()

def SendData1():
    bus.write_byte(addr, 1)
    sleep(0.001)
    bus.write_byte(addr, pos[0])
    sleep(0.001)
    bus.write_byte(addr, pos[1])
    sleep(0.001)
    bus.write_byte(addr, pos[2])
    sleep(0.001)
    bus.write_byte(addr, speed[A])
    sleep(0.001)
    bus.write_byte(addr, speed[B])
    sleep(0.001)
    bus.write_byte(addr, speed[C])
    sleep(0.001)

def SendData2():
    bus.write_byte(addr, 2)
    sleep(0.001)
    bus.write_byte(addr, out[0])
    sleep(0.001)
    bus.write_byte(addr, out[1])
    sleep(0.001)

def PID(setpointX, setpointY):
    global error, errorPrev, integr, deriv, out, speed, speedPrev, pos
    
    A_CurrentPosition = 0
    B_CurrentPosition = 0
    C_CurrentPosition = 0
    
    if x != 0:
        detected = 1
        
        # Calculate PID values for X and Y
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (Xoffset - x - setpointX) if i == 0 else (Yoffset - y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if (deriv[i] != deriv[i] or abs(deriv[i]) == float('inf')) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(-0.25, min(0.25, out[i]))
        
        # Calculate stepper motor speeds
        for i in range(3):
            speedPrev[i] = speed[i]
            speed[i] = (A_CurrentPosition if i == A else
                        B_CurrentPosition if i == B else
                        C_CurrentPosition if i == C else 0)
            speed[i] = abs(speed[i] - pos[i]) * ks
            speed[i] = max(speedPrev[i] - 200, min(speedPrev[i] + 200, speed[i]))
            speed[i] = max(0, min(1000, speed[i]))
            if i == A: A_CurrentPosition=speed[i]
            if i == B: B_CurrentPosition=speed[i]
            if i == C: C_CurrentPosition=speed[i]
        
        # Print outputs
        print(f"X OUT = {out[0]}   Y OUT = {out[1]}   Speed A: {speed[A]}")
     
    else:
        # Delay and re-check for ball detection
        time.sleep(0.01)  # 10 millis delay
        
        if x == 0:
            detected = 0
    
    if detected == 1:
        pos[0] = round((angOrig - theta(A,4.5,-out[0],-out[1])) * angToStep)
        pos[1] = round((angOrig - theta(B,4.5,-out[0],-out[1]))* angToStep)
        pos[2] = round((angOrig - (C,4.5,-out[0],-out[1])) * angToStep)
        
        speed[A] = A_CurrentPosition
        speed[B] = B_CurrentPosition
        speed[C] = C_CurrentPosition
    else:
        pos[0] = round((angOrig - theta(A,4.5,0,0)) * angToStep
        pos[1] = round((angOrig - theta(B,4.5,0,0)) * angToStep
        pos[2] = round((angOrig - theta(C,4.5,0,0)) * angToStep
        speed[A] = 800
        speed[B] = 800
        speed[C] = 800

    timeI = time.time()
    while (time.time() - timeI) < 0.02:  # 20 millis = 0.02 seconds
        SendData1()
        sleep(0.001)

def theta(leg, hz, nx, ny):
    PI = math.pi
    
    # create unit normal vector
    nmag = math.sqrt(nx**2 + ny**2 + 1)  # magnitude of the normal vector
    nx /= nmag
    ny /= nmag
    nz = 1 / nmag
    
    # variables for calculating angle
    angle = 0.0
    x, y, z = 0.0, 0.0, 0.0
    d = 2.0 # distance from the center of the base to any of its corners
    e = 3.125 # distance from the center of the platform to any of its corners
    f = 1.75 # length of link #1
    g = 4.0 #length of link #2
    
    # calculates angle A, B, or C
    if leg == 'A':  # Leg A
        y = d + (e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
        z = hz + e * ny
        mag = math.sqrt(y**2 + z**2)
        angle = math.acos(y / mag) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
    elif leg == 'B':  # Leg B
        x = (math.sqrt(3) / 2) * (e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - d)
        y = x / math.sqrt(3)
        z = hz - (e / 2) * (math.sqrt(3) * nx + ny)
        mag = math.sqrt(x**2 + y**2 + z**2)
        angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
    elif leg == 'C':  # Leg C
        x = (math.sqrt(3) / 2) * (d - e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
        y = -x / math.sqrt(3)
        z = hz + (e / 2) * (math.sqrt(3) * nx - ny)
        mag = math.sqrt(x**2 + y**2 + z**2)
        angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
    
    return angle * (180 / PI)  # convert angle to degrees and return

