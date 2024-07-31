#include <AccelStepper.h>

// Define motors config
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
long receivedSteps[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 steps
long receivedSpeed[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 speed
long receivedAcceleration[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 acceleration

// Variables for managing serial input and commands
bool newData = false;
bool runAllowed = false;

void setup() {
  Serial.begin(9600);

  // Set maximum speed and acceleration for each stepper motor
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000); // Speed = Steps / second
    steppers[i].setAcceleration(800); // Acceleration = Steps /(second)^2
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position: ");
    Serial.println(steppers[i].currentPosition());
  }
  home();
  Serial.println("Out of home");
}

void loop() {
  checkSerial(); // Check for commands from serial monitor
  runMotors(); // Execute movements of the motors
}

void home() {
  runAllowed = true;
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = 200;
    receivedSpeed[i] = 800;
    receivedAcceleration[i] = 100;
  }
  Serial.println("One");
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].moveTo(-1 * receivedSteps[i]);

    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" - Received Steps: ");
    Serial.print(receivedSteps[i]);
    Serial.print(", Speed: ");
    Serial.print(receivedSpeed[i]);
    Serial.print(", Acceleration: ");
    Serial.println(receivedAcceleration[i]);
  }
  Serial.println("Two");
  runMotors();
  Serial.println("Three");
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setCurrentPosition(0);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position after homing: ");
    Serial.println(steppers[i].currentPosition());
  }
  Serial.println("Four");
  runAllowed = false;
}

void runMotors() {
  if (runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++)
     steppers[i].enableOutputs(); // Enable outputs for all motors
    while (steppers[0].currentPosition() != -1*receivedSteps[0] || steppers[1].currentPosition() != -1*receivedSteps[1] || steppers[2].currentPosition() != -1*receivedSteps[2]) {
      for (int i = 0; i < MAX_STEPPERS; i++) 
        if (steppers[i].currentPosition() != -1*receivedSteps[i])
          steppers[i].run(); // Step each motor
      if (Serial.available() > 0) return;
    }
    Serial.println("Exited while loop");
    runAllowed=false;
  }
  if (!runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs(); // Disable outputs for all motors
    }
  }
}

void checkSerial() {
  //recieves char followed by steps,speed,accel for each stepper (ex "200 200 200 200 200 200 200 200 200)
  if (Serial.available() > 0) {
    newData = true;
    runAllowed = true;
    float tempSteps, tempSpeed, tempAccel;

    for (int i = 0; i < MAX_STEPPERS; i++) {
      tempSteps = Serial.parseInt();
      tempSpeed = Serial.parseInt();
      tempAccel = Serial.parseInt();

      if (tempSteps != 0 && tempSpeed != 0 && tempAccel != 0) {
        receivedSteps[i] = tempSteps;
        receivedSpeed[i] = tempSpeed;
        receivedAcceleration[i] = tempAccel;
      }
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
          
      steppers[i].moveTo(-1 * receivedSteps[i]);
    }
  }
  while (Serial.available() > 0 && Serial.parseInt() == 0 && Serial.parseInt() == 0) {
    Serial.parseInt(); // Discard any extra floats
  }
}
