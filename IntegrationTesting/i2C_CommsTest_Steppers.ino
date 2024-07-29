#include <Wire.h>
#include <AccelStepper.h>

const int MAX_STEPPERS = 3;
const int STEP_PINS[MAX_STEPPERS] = {3,6,9};
const int DIR_PINS[MAX_STEPPERS] = {2,5,8};

AccelStepper steppers[MAX_STEPPERS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
};

volatile long receivedSteps[MAX_STEPPERS] = {1,0,0};
volatile long receivedSpeed[MAX_STEPPERS] = {1,0,0};
volatile long receivedAcceleration[MAX_STEPPERS] = {1,0,0};

volatile bool runAllowed = false;
volatile int receivedValue = 0;
byte endbit = 0;

const int ledPin = 23;

// int BYTE1 = 1;
// int BYTE2 = 244;

void setup() {
  Wire.begin(0x8);
  // Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  for(int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000);
    steppers[i].setAcceleration(800);
    steppers[i].disableOutputs();
    steppers[i].setCurrentPosition(0);
  }

  // int RV = (BYTE1 << 8) | BYTE2;
  // Serial.println(RV);

  home();

  Wire.onReceive(receiveEvent);
}

void loop() {
  if (Wire.available()){
    receiveEvent();
  }
  delay(50);
}

int parse_Integer() {
  byte temp = Wire.read();
  while (temp == 255) {temp = Wire.read();}
  byte integer1 = temp;

  temp = Wire.read();
  while (temp == 255) {temp = Wire.read();}
  byte integer2 = temp;

  // endbit = Wire.read();
  // delay(500);
  receivedValue = (integer1 << 8) | integer2;

  // Serial.print("int 1: ");
  // Serial.println(integer1);
  // Serial.print("int 2: ");
  // Serial.println(integer2);
  // Serial.print("RV: ");
  // Serial.println(receivedValue);
  return receivedValue;
}

void receiveEvent() {
  // Serial.println(Wire.available());
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = parse_Integer();
    receivedSpeed[i] = parse_Integer();
    receivedAcceleration[i] = parse_Integer();
    // Serial.println(i);
    // Serial.print("Steps: ");
    // Serial.println(receivedSteps[i]);
    // Serial.print("Speeds: ");
    // Serial.println(receivedSpeed[i]);
    // Serial.print("Accels: ");
    // Serial.println(receivedAcceleration[i]);
  }

  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].moveTo((-1)*receivedSteps[i]);
  }
  runAllowed=true;
  runMotors();
}

void home() {
  runAllowed = true;
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = 200;
    receivedSpeed[i] = 1000;
    receivedAcceleration[i] = 100;
  }
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setMaxSpeed(receivedSpeed[i]);
    steppers[i].moveTo((-1) * receivedSteps[i]);
  }
  runMotors();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setCurrentPosition(0);
  }
  runAllowed=false;
}

void runMotors() {
  if (runAllowed) {
    while (steppers[0].currentPosition() != (-1)*receivedSteps[0] && 
    steppers[1].currentPosition() != (-1)*receivedSteps[1]  && 
    steppers[2].currentPosition() != (-1)*receivedSteps[2]) 
    {
      for (int i = 0; i < MAX_STEPPERS; i++) {
        steppers[i].enableOutputs();
        steppers[i].run();
      }
    }
    runAllowed = false;
  }
  if (!runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs();
    }
  }
}
