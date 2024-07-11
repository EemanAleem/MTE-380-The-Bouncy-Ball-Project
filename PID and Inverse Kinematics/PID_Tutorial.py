import math
import time
import cv2 as cv
import numpy as np
from time import sleep
from smbus2 import SMBus

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1. Use this bus.

K_PID = [1,1,1]
k = 1
alpha = 1

distance_from_motor_x = 0 # Horizontal distance from ball to motor in cm
pixels_per_cm = 0 #how many pixels cover 1 cm

Goal = 0 # Desired Y value

# Again here is the function to detect a yellow ball
def detect_yellow_ball():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        frame = cv.resize(frame, (480, 480))

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        mask = cv.inRange(hsv, lower_yellow, upper_yellow)

        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10: 
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

        cv.imshow('frame', frame)
        
        # Calculate the angle to move the motor
        angle = compute(compute, Goal, int(y))
        # Send the angle to the arduino
        bus.write_byte(addr, angle)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

def __init__(self, K_PID, k, alpha):
    self.kp = K_PID[0]  # Proportional gain
    self.ki = K_PID[1]  # Integral gain
    self.kd = K_PID[2]  # Derivative gain
    self.k = k  # Coefficient determining the magnitude of output
    self.alpha = alpha  # Coefficient for the low-pass filter
    self.last_output_y = 0  # Last output of PID in the y-direction
    self.last_error_y = 0  # Last error in the y-direction
    self.integral_y = 0  # Integral value in the y-direction
    self.last_time = None  # Last recorded time
    self.count = 0  # Counter
    self.F = 0  # Placeholder variable

def compute(self, Goal, Current_value):
    current_time = time.perf_counter()
    if self.last_time is None:
        self.last_time = current_time
        return 0 # Return zero values if last_time is None
    # Calculate errors
    error_y = Goal - Current_value
    # Calculate integral values
    self.integral_y += error_y * (current_time - self.last_time)
    # Calculate derivative values
    derivative_y = (error_y - self.last_error_y) / (current_time - self.last_time)
    # Calculate PID outputs
    output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
    # Apply low-pass filter
    output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
    # Calculate angle to move motor
    angle = atan2(- output_y, distance_from_motor_x * pixels_per_cm)

    # Update variables for next iteration
    self.last_error_y = error_y
    self.last_output_y = output_y
    self.last_time = current_time

    return angle

# Call the function to detect the yellow ball
detect_yellow_ball()
