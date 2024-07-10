import math
import time
    
class PID:
    def __init__(self, K_PID, k, alpha):
        self.kp = K_PID[0]  # Proportional gain
        self.ki = K_PID[1]  # Integral gain
        self.kd = K_PID[2]  # Derivative gain
        self.k = k  # Coefficient determining the magnitude of output
        self.alpha = alpha  # Coefficient for the low-pass filter
        self.last_output_x = 0  # Last output of PID in the x-direction
        self.last_output_y = 0  # Last output of PID in the y-direction
        self.last_error_x = 0  # Last error in the x-direction
        self.integral_x = 0  # Integral value in the x-direction
        self.last_error_y = 0  # Last error in the y-direction
        self.integral_y = 0  # Integral value in the y-direction
        self.last_time = None  # Last recorded time
        self.count = 0  # Counter
        self.F = 0  # Placeholder variable

    def compute(self, Goal, Current_value):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0  # Return zero values if last_time is None
        # Calculate errors
        error_x = Goal[0] - Current_value[0]
        error_y = Goal[1] - Current_value[1]
        # Calculate integral values
        self.integral_x += error_x * (current_time - self.last_time)
        self.integral_y += error_y * (current_time - self.last_time)
        # Calculate derivative values
        derivative_x = (error_x - self.last_error_x) / (current_time - self.last_time)
        derivative_y = (error_y - self.last_error_y) / (current_time - self.last_time)
        # Calculate PID outputs
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        # Apply low-pass filter
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        # Calculate theta and phi
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        phi = self.k * math.sqrt(output_x**2 + output_y**2)

        # Update variables for next iteration
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time

        return theta, phi  # Return calculated theta and phi
