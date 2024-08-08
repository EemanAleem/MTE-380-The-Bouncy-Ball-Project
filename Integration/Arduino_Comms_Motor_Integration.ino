#include <Wire.h>
#include <AccelStepper.h>

// i2c & LED test pins
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 47;

// I2C COMMS SETUP **********************************************************************
// rB is byte received from transmission; int1 and int2 are temporary values for rV;
// countByte is used to check whether we have received a first or second byte;
// rV is two bytes combined with a bitwise operation to get an integer in the range -32,768 to 32,767
volatile long int receivedByte, receivedValue;
volatile long int int1, int2;
volatile long long int countByte = 1, countValue = 1;
unsigned long st1, end1, dur1;
// volatile long int pos[3];

// MOTORS CONFIGURATION *****************************************************************
const int MAX_STEPPERS = 3;
const int STEP_PINS[MAX_STEPPERS] = {3, 6, 9};   // Step pins for motors 1, 2, 3
const int DIR_PINS[MAX_STEPPERS] = {4, 7, 10};    // Direction pins for motors 1, 2, 3
const int ks = 20; //speed amplifying constant
bool allAtPosition = false;

// Create instances of AccelStepper for each motor
AccelStepper steppers[MAX_STEPPERS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2])
};

// Array to store received parameters for each motor
// -373, -373, -373
volatile long int pos[MAX_STEPPERS] = {200, 200, 200}; // Initialize with 200 steps
volatile long int speed[MAX_STEPPERS] = {0, 0, 0}; // Initialize with 0 speed
volatile long int speedPrev[MAX_STEPPERS] = {0,0,0};

// FUNCTIONS *****************************************************************************
void home();
void setSpeed();
void moveMotors();
void receiveEvent();

void setup() {
  Wire.begin(0x8);
  Wire.setClock(400000);
  Serial.begin(9600);
  delay(3000);

  // Set maximum speed and acceleration for each stepper motor
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setMaxSpeed(100000); // Speed = Steps / second
    steppers[i].disableOutputs(); // Disable outputs initially
    steppers[i].setCurrentPosition(0); // Reset current position to 0
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" current position: ");
    Serial.println(steppers[i].currentPosition());
    Serial.print(pos[i]);
  }
  
  home();
  Serial.println("Out of home"); 
  
  Wire.onReceive(receiveEvent);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  // if (Wire.available())
  //   receiveEvent();
  // moveMotors();
  // Serial.println("In void loop"); 
  delay(10);
}

void receiveEvent() {
  // Serial.println("In receive event"); 
  receivedByte = Wire.read();

  // If the counter is odd, therefore a first byte.
  if (countByte%2 != 0)
    int1 = receivedByte;
  else {
    int2 = receivedByte;

    // Combine bytes to get a long int
    receivedValue = (int1 << 8) | int2;

    // Sort value received into the pos[3] array
    if ( (countValue - 1) % 3 == 0 ) {
      pos[0] = receivedValue;
      Serial.print("pos[0]: ");
      Serial.println(pos[0]);
    }
    else if ( (countValue - 2) % 3 == 0 ) {
      pos[1] = receivedValue;
      Serial.print("pos[1]: ");
      Serial.println(pos[1]);
    }
    else {
      pos[2] = receivedValue;
      Serial.print("pos[2]: ");
      Serial.println(pos[2]);
      setSpeed();
    }
    countValue++;
  }
  countByte++;

  // delay(20);
}

void home() {
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].enableOutputs(); // Enable outputs for all motors
    steppers[i].setAcceleration(500);
    steppers[i].setMaxSpeed(1000);
    steppers[i].moveTo(-1*pos[i]);
  }

  allAtPosition = false;
  while (!allAtPosition) {
    allAtPosition = true;
    
    for (int i = 0; i < MAX_STEPPERS; i++) {
      if (steppers[i].currentPosition() != -1*pos[i]) {
        // Serial.println(steppers[i].currentPosition());
        steppers[i].run(); // Step each motor
        allAtPosition = false;
      }
    }
  }

  for (int i = 0; i < MAX_STEPPERS; i++)
    steppers[i].setCurrentPosition(0);
}

void moveMotors()
{
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].enableOutputs();
    steppers[i].setMaxSpeed(speed[i]);
    steppers[i].setAcceleration(speed[i] * 30);
    steppers[i].moveTo(-1*pos[i]);
  }

  // int timePrev = millis();
  // while(millis() - timePrev < 20) {
  //   steppers[0].run();
  //   steppers[1].run();
  //   steppers[2].run();
  // }

  allAtPosition = false;
  while (!allAtPosition) {
    allAtPosition = true;
    
    for (int i = 0; i < MAX_STEPPERS; i++)
      if (steppers[i].currentPosition() != -1*pos[i]) {
        steppers[i].run(); // Step each motor
        allAtPosition = false;
      }

  }

}

void setSpeed() {
  st1 = micros();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    speedPrev[i] = speed[i]; // Sets previous speed
    speed[i] = abs(steppers[i].currentPosition() - pos[i]) * ks; // Calculates the error in the current position and target position
    speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200); // Filters speed by preventing it from being over 200 away from last speed
    speed[i] = constrain(speed[i], 0, 1000);     
  }
  end1 = micros();
  dur1 = end1-st1;
  Serial.println(dur1);
  Serial.println("Done setSpeed");
}
