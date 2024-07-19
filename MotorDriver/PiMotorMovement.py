import RPi.GPIO as GPIO
import time

# Define GPIO pins for each motor
motor1_pins = (step_pin_1, dir_pin_1, enable_pin_1)  # Replace with actual GPIO pin numbers
motor2_pins = (step_pin_2, dir_pin_2, enable_pin_2)  # Replace with actual GPIO pin numbers
motor3_pins = (step_pin_3, dir_pin_3, enable_pin_3)  # Replace with actual GPIO pin numbers

# Motor parameters
steps_per_rev = 200  # Number of steps per revolution of your motors

# Function to initialize GPIO pins
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor1_pins, GPIO.OUT)
    GPIO.setup(motor2_pins, GPIO.OUT)
    GPIO.setup(motor3_pins, GPIO.OUT)

# Function to move motor by specified steps, direction, speed, and acceleration
def move_motor(motor_pins, steps, direction, speed, acceleration):
    step_pin, dir_pin, enable_pin = motor_pins
    
    GPIO.output(enable_pin, GPIO.LOW)  # Enable the motor driver
    
    # Set direction
    GPIO.output(dir_pin, direction)
    
    # Calculate initial delay based on speed
    initial_delay = 1.0 / (speed * steps_per_rev)
    
    # Accelerate smoothly
    current_delay = initial_delay
    for _ in range(steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(current_delay / 2.0)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(current_delay / 2.0)
        current_delay = max(initial_delay, current_delay - acceleration * initial_delay)
    
    GPIO.output(enable_pin, GPIO.HIGH)  # Disable the motor driver

# Main function to test motors
def main():
    setup_gpio()
    
    try:
        # Move motor 1
        move_motor(motor1_pins, 400, GPIO.HIGH, speed=100, acceleration=1)  # Example: 400 steps clockwise
    
        # Move motor 2
        move_motor(motor2_pins, 600, GPIO.LOW, speed=200, acceleration=0.5)   # Example: 600 steps counterclockwise
    
        # Move motor 3
        move_motor(motor3_pins, 800, GPIO.HIGH, speed=150, acceleration=0.8)  # Example: 800 steps clockwise
    
    except KeyboardInterrupt:
        pass
    
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
