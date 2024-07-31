#include <Wire.h>
#include <AccelStepper.h>

// Define motors config
const int MAX_STEPPERS = 3;
const int STEP_PINS[MAX_STEPPERS] = {3, 6, 9};   // Step pins for motors 1, 2, 3
const int DIR_PINS[MAX_STEPPERS] = {4, 7, 10};    // Direction pins for motors 1, 2, 3
const int ks = 20; //speed amplifying constant

const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 9;
// The received integer
volatile int receivedValue = 0;

// Create instances of AccelStepper for each motor
AccelStepper steppers[MAX_STEPPERS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2])
};

// Array to store received parameters for each motor
int pos[MAX_STEPPERS] = {-373, -373, -373}; // Initialize with 0 steps
int speed[MAX_STEPPERS] = {0, 0, 0}; // Initialize with 0 speed
int speedPrev[MAX_STEPPERS] = {0,0,0};

byte endbit = 0;

unsigned long st1, et1, dur1, st2, et2, dur2;

// Function declarations
void home();
void setSpeed();
void moveMotors();
int parse_Integer();
void receiveEvent();

void setup() {
  st1 = micros();
  Wire.begin(0x8);
  Wire.setClock(100000);
  Serial.begin(9600);

  // Set maximum speed and acceleration for each stepper motor
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000); // Speed = Steps / second
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position: ");
    Serial.println(steppers[i].currentPosition());
  }
  
  home();
  Serial.println("Out of home"); 
  
  et1 = micros();
  dur1 = (et1 - st1);
  Serial.println(dur1);
  Wire.onReceive(receiveEvent);
}

void loop() {
  if (Wire.available())
    receiveEvent();
  setSpeed();
  moveMotors();
}

int parse_Integer() {
  byte temp = Wire.read();
  while (temp == 255) {temp = Wire.read();}
  byte integer1 = temp;

  delay(15);

  temp = Wire.read();
  while (temp == 255) {temp = Wire.read();}
  byte integer2 = temp;

  // endbit = Wire.read();
  // delay(500);
  receivedValue = (integer1 << 8) | integer2;

  Serial.print("int 1: ");
  Serial.println(integer1);
  Serial.print("int 2: ");
  Serial.println(integer2);
  Serial.print("RV: ");
  Serial.println(receivedValue);
  return receivedValue;
}

void receiveEvent() {
  // Serial.println(Wire.available());
  for (int i = 0; i < MAX_STEPPERS; i++) 
    pos[i] = parse_Integer();
}

void home() {
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(0);
    steppers[i].setMaxSpeed(3200);
    steppers[i].moveTo(pos[i]);
  }
  
  for (int i = 0; i < MAX_STEPPERS; i++) {
     steppers[i].enableOutputs(); // Enable outputs for all motors
    bool allAtPosition = false;
  }
  bool allAtPosition = false;
  while (!allAtPosition) {
    allAtPosition = true;
    for (int i = 0; i < MAX_STEPPERS; i++) 
      if (steppers[i].currentPosition() != pos[i]) 
        steppers[i].run(); // Step each motor
        allAtPosition = false;
  }
}

void setSpeed() {
 for (int i = 0; i < MAX_STEPPERS; i++) {
    speedPrev[i] = speed[i]; // Sets previous speed
    speed[i] = abs(steppers[i].currentPosition() - pos[i]) * ks; // Calculates the error in the current position and target position
    speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200); // Filters speed by preventing it from being over 200 away from last speed
    speed[i] = constrain(speed[i], 0, 1000);     
  } 
}

void moveMotors()
{
  for (int i = 0; i < MAX_STEPPERS; i++) {
    //sets calculated speed
    steppers[i].setMaxSpeed(speed[i]);
    //sets acceleration to be proportional to speed
    steppers[i].setAcceleration(speed[i] * 30);
    //sets target positions
    steppers[i].moveTo(pos[i]);
  }
    int timePrev = millis();
    while(millis() - timePrev < 20) {
      steppers[0].run();
      steppers[1].run();
      steppers[2].run();
    }
}
