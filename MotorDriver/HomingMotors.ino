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
long receivedSteps[MAX_STEPPERS] = {0, 0, 0}; // Initialize with 0 steps
long receivedSpeed[MAX_STEPPERS] = {0, 0, 0}; // Initialize with 0 speed
long receivedAcceleration[MAX_STEPPERS] = {0, 0, 0}; // Initialize with 0 acceleration
// Variables for managing serial input and commands
bool newData = false;
bool runAllowed = false;
int directionMultiplier = -1; // = 1: positive direction, = -1: negative direction

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
  home();
}

void loop() {
  checkSerial(); // Check for commands from serial monitor
  runMotors(); // Execute movements of the motors
}

void home() {
  runAllowed = true;
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = 300;
    receivedSpeed[i] = 800;
    receivedAcceleration[i] = 100;
  }

  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].move(directionMultiplier * receivedSteps[i]);

    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" - Received Steps: ");
    Serial.print(receivedSteps[i]);
    Serial.print(", Speed: ");
    Serial.print(receivedSpeed[i]);
    Serial.print(", Acceleration: ");
    Serial.println(receivedAcceleration[i]);
  }

  runMotors();
  delay(6000);

  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setCurrentPosition(0);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position after homing: ");
    Serial.println(steppers[i].currentPosition());
  }

  runAllowed = false;
}

void runMotors() {
  if (runAllowed) {
    while (steppers[0].currentPosition()!=-300) {
      Serial.println("Current position: ");
      Serial.println(steppers[0].currentPosition());
      for (int i = 0; i < MAX_STEPPERS; i++) {
        steppers[i].enableOutputs(); // Enable outputs for all motors
        steppers[i].run(); // Step each motor
      }
    }
  }
  if (!runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs(); // Disable outputs for all motors
      Serial.println("Cannot Run");
    }
  }
}

void checkSerial() {
  //recieves char followed by steps,speed,accel for each stepper (ex "p200 200 200 200 200 200 200 200 200)
  if (Serial.available() > 0) {
    newData = true;

    if (newData) {
      for (int i = 0; i < MAX_STEPPERS; i++) {
        receivedSteps[i] = Serial.parseFloat();
        receivedSpeed[i] = Serial.parseFloat();
        receivedAcceleration[i] = Serial.parseFloat();

        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(" - Received Steps: ");
        Serial.print(receivedSteps[i]);
        Serial.print(", Speed: ");
        Serial.print(receivedSpeed[i]);
        Serial.print(", Acceleration: ");
        Serial.println(receivedAcceleration[i]);

        steppers[i].setAcceleration(receivedAcceleration[i]);
        steppers[i].setMaxSpeed(receivedSpeed[i]);
        if (receivedSteps[i] < 0)
          directionMultiplier = 1;
        else
          directionMultiplier = -1;
          
        steppers[i].moveTo(directionMultiplier * receivedSteps[i]);
      }
      runAllowed = true;
    }
  }
}

// bool anyMotorRunning() {
//   for (int i = 0; i < MAX_STEPPERS; i++) {
    // if (steppers[i].distanceToGo() != 0) {
//       return true;
//     }
//   }
//   return false;
// }
