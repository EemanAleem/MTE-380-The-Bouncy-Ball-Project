import math
import serial
import cProfile
from smbus2 import SMBus
import time
from time import sleep
import cv2
import numpy as np
import struct

#arduino = SerialObject("COM3")
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1

# GLOBAL ************************************************************
# PID:
error = [0.0, 0.0]
errorPrev = [0.0, 0.0]
integr = [0.0, 0.0]
deriv = [0.0, 0.0]
out = [0.0, 0.0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
pos = [0, 0, 0]
prevT = 0

#real-time coordinates
x = 0
y = 0

# Checks whether the ball is present on the platform or not
detected = 0

# What point on the platform do we want the ball to remain at?
setpointX = 0
setpointY = 0

# Period of oscillation calculations for Ziegler-Nichols tuning
last_cross_time_x = None
last_cross_time_y = None
period_x = None
period_y = None

# CONSTANTS ***********************************************************
PI = math.pi
ANG_TO_STEP = 6400 / 360
ANG_ORIG = 204.0
X_OFFSET = 240  # Replace with actual X offset value
Y_OFFSET = 240  # Replace with actual Y offset value
KP = 2.1E-4 #2.1E-4   Replace with actual proportional gain
KI = 0 #2E-6  # Replace with actual integral gain
KD = 0 #4.8E-5  # Replace with actual derivative gain

A = 0  # Index for stepper A
B = 1  # Index for stepper B
C = 2  # Index for stepper C

sleep(5)

# Define a function to detect a yellow ball
def detect_yellow_ball():
    global x, y, detected # Declare said variables as global variables
  
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        frame = cv2.resize(frame, (480, 480))

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of yellow color in HSV
        lower_yellow = np.array([27, 160, 100])
        upper_yellow = np.array([33, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            detected = 1
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if  radius > 10:  # Only consider large enough objects
                # Draw a circle around the yellow ball
                # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a dot in the center of the yellow ball
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                
# The following line displays the xy position of the ball
#                 print(f"(Detected = {detected}, {int(x)}, {int(y)})")
        else:
            detected = 0

        # Display the resulting frame
        cv2.imshow('frame', frame)
    
        sleep(0.03) 
        PID(setpointX, setpointY)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pos[0] = 400
            pos[1] = 400
            pos[2] = 400
            SendData()
            break

    # Release the capture when everything is done
    cap.release()
    cv2.destroyAllWindows()

def SendData():
    pos[0] = list(struct.pack('>h', pos[0]))
    for j in range (0,2):
# The following comment tests if the numbers are properly split
#         print("pos[", 0, "][", j, "]: ", pos[0][j])
        bus.write_byte(addr, pos[0][j])
        sleep(0.002)
    
    pos[1] = list(struct.pack('>h', pos[1]))
    for j in range (0,2):
# The following comment tests if the numbers are properly split
#         print("pos[", 1, "][", j, "]: ", pos[1][j])
        bus.write_byte(addr, pos[1][j])
        sleep(0.002)
    
    pos[2] = list(struct.pack('>h', pos[2]))
    for j in range (0,2):
# The following comment tests if the numbers are properly split
#         print("pos[", 2, "][", j, "]: ", pos[2][j])
        bus.write_byte(addr, pos[2][j])
        sleep(0.002)

    # Delay 20ms between a full set transmissions
    sleep(0.02)

    
def PID(setpointX, setpointY):
    global error, errorPrev, integr, deriv, out, speed, speedPrev, pos, prevT
    global last_cross_time_x, last_cross_time_y, period_x, period_y

    if detected == 1:
        
        # Calculate PID values for X and Y
        for i in range(2):
            currT = time.time()
            deltaT = currT - prevT
            prevT = currT
            errorPrev[i] = error[i]
            error[i] = (x - X_OFFSET - setpointX) if i == 0 else (Y_OFFSET - y - setpointY)
            integr[i] += (error[i] + errorPrev[i]) * deltaT
            deriv[i] = (error[i] - errorPrev[i]) / deltaT
            deriv[i] = 0 if (deriv[i] != deriv[i] or abs(deriv[i]) == float('inf')) else deriv[i]
            out[i] = KP * error[i] + KI * integr[i] + KD * deriv[i]
            out[i] = max(-0.25, min(0.25, out[i]))

# The following block is used in finding the time period for the Ziegler Nichols Tuning Method            
#             if error[i]<0<errorPrev[i] or errorPrev[i]<0<error[i]:
#                 current_time = time.time()
#                 if i == 0:
#                     if last_cross_time_x is not None:
#                         period_x = current_time - last_cross_time_x
#                         print(f"Period X: {period_x} seconds")
#                     last_cross_time_x = current_time
#                 else:
#                     if last_cross_time_y is not None:
#                         period_y = current_time - last_cross_time_y
#                         print(f"Period Y: {period_y} seconds")
#                     last_cross_time_y = current_time
            
        pos[0] = round((ANG_ORIG - theta(A,4.5,-out[0],-out[1])) * ANG_TO_STEP)
        pos[1] = round((ANG_ORIG - theta(B,4.5,-out[0],-out[1]))* ANG_TO_STEP)
        pos[2] = round((ANG_ORIG - theta(C,4.5,-out[0],-out[1])) * ANG_TO_STEP)
     
    else:
        # Delay and re-check for ball detection
        time.sleep(0.01)  # 10 millis delay
        
        pos[0] = round((ANG_ORIG - theta(A,4.5,0,0)) * ANG_TO_STEP)
        pos[1] = round((ANG_ORIG - theta(B,4.5,0,0)) * ANG_TO_STEP)
        pos[2] = round((ANG_ORIG - theta(C,4.5,0,0)) * ANG_TO_STEP)
# The following block is used for viewing the stepper positions
#     print(f"pos[0] = {pos[0]}")
#     print(f"pos[1] = {pos[1]}")
#     print(f"pos[2] = {pos[2]}")
    SendData()
   

def theta(leg, hz, nx, ny):
    PI = math.pi
    
    # create unit normal vector
    nmag = math.sqrt(nx**2 + ny**2 + 1)  # magnitude of the normal vector
    nx /= nmag
    ny /= nmag
    nz = 1 / nmag
    
    # variables for calculating angle
    angle = 0.0
    xPOS, yPOS, zPOS = 0.0, 0.0, 0.0
    d = 2.0 # distance from the center of the base to any of its corners
    e = 3.125 # distance from the center of the platform to any of its corners
    f = 1.75 # length of link #1
    g = 4.0 #length of link #2
    
    # calculates angle A, B, or C
    if leg == A:  # Leg A
        yPOS = d + (e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
        zPOS = hz + e * ny
        mag = math.sqrt(yPOS**2 + zPOS**2)
        angle = math.acos(yPOS / mag) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
    elif leg == B:  # Leg B
        xPOS = (math.sqrt(3) / 2) * (e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - d)
        yPOS = xPOS / math.sqrt(3)
        zPOS = hz - (e / 2) * (math.sqrt(3) * nx + ny)
        mag = math.sqrt(xPOS**2 + yPOS**2 + zPOS**2)
        angle = math.acos((math.sqrt(3) * xPOS + yPOS) / (-2 * mag)) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
    elif leg == C:  # Leg C
        xPOS = (math.sqrt(3) / 2) * (d - e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
        yPOS = -xPOS / math.sqrt(3)
        zPOS = hz + (e / 2) * (math.sqrt(3) * nx - ny)
        mag = math.sqrt(xPOS**2 + yPOS**2 + zPOS**2)
        angle = math.acos((math.sqrt(3) * xPOS - yPOS) / (2 * mag)) + math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))
        
# The following block is used for viewing the angle outputs
#     print(f"Leg {leg}:")
#     print(f"  x = {xPOS}")
#     print(f"  y = {yPOS}")
#     print(f"  z = {zPOS}")
#     print(f"  angle = {angle}")
    return angle * (180 / PI)  # convert angle to degrees and return

if __name__ == '__main__':
    detect_yellow_ball()
