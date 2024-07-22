#include <Wire.h>
#include <AccelStepper.h>
#include <cvzone.h>

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
volatile long receivedSteps[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 steps
volatile long receivedSpeed[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 speed
volatile long receivedAcceleration[MAX_STEPPERS] = {1, 0, 0}; // Initialize with 0 acceleration

// Variables for managing serial input and commands
volatile bool runAllowed = false;

// The received integer
volatile int receivedValue = 0;

const int ledPin = 13; 

void setup() {
  // Initialize I2C communication as slave
  Wire.begin(0x8); // Address of the Arduino

  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Set maximum speed and acceleration for each stepper motor
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000); // Speed = Steps / second
    steppers[i].setAcceleration(800); // Acceleration = Steps /(second)^2
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
  }
  home();
  // Register function to run when data is received
  Wire.onReceive(receiveEvent);
}

void loop() {
  if (runAllowed) {
    runMotors();
  }
  delay(100);
}

// Function to parse integer from I2C data
int parse_Integer() {
  if (Wire.available() >= 2) { // We expect 2 bytes for the integer
    byte byte1 = Wire.read();
    byte byte2 = Wire.read();
    
    // Combine the two bytes to form the integer
    receivedValue = (byte1 << 8) | byte2;
    return receivedValue;
  }
  return 0;
}

// Function to run when data is received
void receiveEvent(int howMany) {
  digitalWrite(ledPin, '1');
  for (int i = 0; i < MAX_STEPPERS; i++) {
      receivedSteps[i] = parse_Integer();
      receivedSpeed[i] = parse_Integer();
      receivedAcceleration[i] = parse_Integer();
  }
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].move(receivedSteps[i]);
  }
  runAllowed = true;
}

void home() {
  runAllowed = true;
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = 200;
    receivedSpeed[i] = 800;
    receivedAcceleration[i] = 100;
  }
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].move(-1 * receivedSteps[i]);
  }
  runMotors();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setCurrentPosition(0);
  }
  runAllowed = false;
}

void runMotors() {
  if (runAllowed) {
    while (steppers[0].currentPosition() != -1*receivedSteps[0] && steppers[1].currentPosition() != -1*receivedSteps[1] && steppers[2].currentPosition() != -1*receivedSteps[2]) {
      for (int i = 0; i < MAX_STEPPERS; i++) {
        steppers[i].enableOutputs(); // Enable outputs for all motors
        steppers[i].run(); // Step each motor
      }
      // if (Serial.available() > 0) return;
    }
    runAllowed = false;
  }
  if (!runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs(); // Disable outputs for all motors
    }
  }
}
