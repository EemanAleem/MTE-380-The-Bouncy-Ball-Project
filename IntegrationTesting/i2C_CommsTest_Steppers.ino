#include <Wire.h>
#include <AccelStepper.h>

const int MAX_STEPPERS = 3;
const int STEP_PINS[MAX_STEPPERS] = {3,6,9};
const int DIR_PINS[MAX_STEPPERS] = {4,7,10}; //{2,5,8};

const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 9;

AccelStepper steppers[MAX_STEPPERS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
};

volatile long receivedSteps[MAX_STEPPERS] = {1,0,0};
volatile signed long receivedSpeed[MAX_STEPPERS] = {1,0,0};
volatile long receivedAcceleration[MAX_STEPPERS] = {100,100,100};

volatile bool runAllowed = false;
volatile int receivedValue = 0;
byte endbit = 0;

unsigned long st1, et1, dur1, st2, et2, dur2;

void setup() {
  st1 = micros();
  Wire.begin(0x8);
  Wire.setClock(100000);
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);

  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000); // Speed = Steps / second
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
  }
  home();

  et1 = micros();
  dur1 = (et1 - st1);
  Serial.println(dur1);
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
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = parse_Integer();
    receivedSpeed[i] = parse_Integer();
    // receivedAcceleration[i] = parse_Integer();
    // Serial.println(i);
    Serial.print("Steps: ");
    Serial.println(receivedSteps[i]);
    Serial.print("Speeds: ");
    Serial.println(receivedSpeed[i]);
    // Serial.print("Accels: ");
    // Serial.println(receivedAcceleration[i]);
  }

  for (int i = 0; i < MAX_STEPPERS; i++) {
    // steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setSpeed(receivedSpeed[i]);
    steppers[i].moveTo((-1)*receivedSteps[i]);
  }
  runAllowed=true;
  runMotors();
}

void home() {
  runAllowed = true;
  for (int i = 0; i < MAX_STEPPERS; i++) {
    receivedSteps[i] = 200;
    receivedSpeed[i] = 3200;
    // receivedAcceleration[i] = 100;
  }
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setAcceleration(receivedAcceleration[i]);
    steppers[i].setSpeed(receivedSpeed[i]);
    steppers[i].moveTo(-1 * receivedSteps[i]);
  }
  runMotors();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setCurrentPosition(0);
  }
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
    }
    runAllowed=false;
  }
  if (!runAllowed) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].disableOutputs(); // Disable outputs for all motors
    }
  }
}
