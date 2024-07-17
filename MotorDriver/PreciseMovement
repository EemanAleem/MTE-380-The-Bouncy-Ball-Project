#include <AccelStepper.h>

// Define constants for each motor
const int MAX_STEPPERS = 3;
const int STEP_PINS[MAX_STEPPERS] = {3, 6, 9};   // Step pins for motors 1, 2, 3
const int DIR_PINS[MAX_STEPPERS] = {2, 5, 8};    // Direction pins for motors 1, 2, 3

// Create instances of AccelStepper for each motor
AccelStepper steppers[MAX_STEPPERS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2])
};

// Array to store received parameters for each motor
long receivedSteps[MAX_STEPPERS] = {0}; // Initialize with 0 steps
long receivedSpeed[MAX_STEPPERS] = {0}; // Initialize with 0 speed
long receivedAcceleration[MAX_STEPPERS] = {0}; // Initialize with 0 acceleration

// Variables for managing serial input and commands
char receivedCommand;
bool newData = false;
bool runAllowed = false;

void setup() {
  Serial.begin(9600);

  // Set maximum speed and acceleration for each stepper motor
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(1200); // Speed = Steps / second
    steppers[i].setAcceleration(800); // Acceleration = Steps /(second)^2
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position: ");
    Serial.println(steppers[i].currentPosition());
  }
}

void loop() {
  checkSerial(); // Check for commands from serial monitor
  runMotors(); // Execute movements of the motors
}

void runMotors() {
  if (runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].enableOutputs(); // Enable outputs for all motors
      steppers[i].run(); // Step each motor
    }
  } else {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs(); // Disable outputs for all motors
    }
  }
}

void checkSerial() {
  if (Serial.available() > 0) {
    receivedCommand = Serial.read();
    newData = true;

    if (newData) {
      switch (receivedCommand) {
        case 'P': // Move motors positively (relative)
          for (int i = 0; i < MAX_STEPPERS; i++) {
            receivedSteps[i] = Serial.parseFloat();
            receivedSpeed[i] = Serial.parseFloat();
            steppers[i].setMaxSpeed(receivedSpeed[i]);
            steppers[i].move(receivedSteps[i]);
          }
          runAllowed = true;
          break;

        case 'N': // Move motors negatively (relative)
          for (int i = 0; i < MAX_STEPPERS; i++) {
            receivedSteps[i] = -Serial.parseFloat();
            receivedSpeed[i] = Serial.parseFloat();
            steppers[i].setMaxSpeed(receivedSpeed[i]);
            steppers[i].move(receivedSteps[i]);
          }
          runAllowed = true;
          break;

        case 'R': // Move motors to absolute positions (positive)
          for (int i = 0; i < MAX_STEPPERS; i++) {
            receivedSteps[i] = Serial.parseFloat();
            receivedSpeed[i] = Serial.parseFloat();
            steppers[i].setMaxSpeed(receivedSpeed[i]);
            steppers[i].moveTo(receivedSteps[i]);
          }
          runAllowed = true;
          break;

        case 'r': // Move motors to absolute positions (negative)
          for (int i = 0; i < MAX_STEPPERS; i++) {
            receivedSteps[i] = -Serial.parseFloat();
            receivedSpeed[i] = Serial.parseFloat();
            steppers[i].setMaxSpeed(receivedSpeed[i]);
            steppers[i].moveTo(receivedSteps[i]);
          }
          runAllowed = true;
          break;

        case 'S': // Stop all motors
          for (int i = 0; i < MAX_STEPPERS; i++) {
            steppers[i].stop();
          }
          runAllowed = false;
          break;

        case 'A': // Set acceleration for all motors
          for (int i = 0; i < MAX_STEPPERS; i++) {
            receivedAcceleration[i] = Serial.parseFloat();
            steppers[i].setAcceleration(receivedAcceleration[i]);
          }
          runAllowed = false;
          break;

        case 'L': // Print current positions of all motors
          for (int i = 0; i < MAX_STEPPERS; i++) {
            Serial.print("Motor ");
            Serial.print(i + 1);
            Serial.print(" current position: ");
            Serial.println(steppers[i].currentPosition());
          }
          runAllowed = false;
          break;

        case 'H': // Home all motors
          for (int i = 0; i < MAX_STEPPERS; i++) {
            steppers[i].setMaxSpeed(400); // Set homing speed
            steppers[i].moveTo(0); // Move to position 0
          }
          runAllowed = true;
          break;

        case 'U': // Update current position as new home (set 0)
          for (int i = 0; i < MAX_STEPPERS; i++) {
            steppers[i].setCurrentPosition(0);
          }
          runAllowed = false;
          break;

        case 'C': // Print commands
          printCommands();
          runAllowed = false;
          break;

        default:
          break;
      }
      newData = false; // Reset newData flag
    }
  }
}

void printCommands() {
  Serial.println(" 'C' : Prints all the commands and their functions.");
  Serial.println(" 'P' : Rotates all motors in positive (CW) direction, relative.");
  Serial.println(" 'N' : Rotates all motors in negative (CCW) direction, relative.");
  Serial.println(" 'R' : Rotates all motors to absolute positive position (+).");
  Serial.println(" 'r' : Rotates all motors to absolute negative position (-).");
  Serial.println(" 'S' : Stops all motors immediately.");
  Serial.println(" 'A' : Sets acceleration value for all motors.");
  Serial.println(" 'L' : Prints current position/location of all motors.");
  Serial.println(" 'H' : Homing sequence for all motors (move to 0).");
  Serial.println(" 'U' : Updates current position as new 0 position for all motors.");
}
