#include <AccelStepper.h>

int directionMultiplier = 1; // = 1: positive direction, = -1: negative direction
const int MAX_STEPPERS = 3;

// Create instances of AccelStepper for each motor
AccelStepper stepper1(AccelStepper::DRIVER, 3, 2);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 5);
AccelStepper stepper3(AccelStepper::DRIVER, 9, 8);

// Array of pointers to AccelStepper objects
AccelStepper* steppers[MAX_STEPPERS];

long receivedSteps[MAX_STEPPERS] = {2400,2400,2400};//Number of steps
long receivedSpeed[MAX_STEPPERS] = {1200,1200,1200}; //Steps / second
long receivedAcceleration[MAX_STEPPERS] = {500,500,500}; //Steps / second^2

void setup() {
  Serial.begin(9600);

  // Assign pointers to each stepper object
  steppers[0] = &stepper1;
  steppers[1] = &stepper2;
  steppers[2] = &stepper3;

  // Set maximum speed and acceleration for each stepper motor
  stepper1.setMaxSpeed(3200); // Speed = Steps / second
  stepper1.setAcceleration(800); // Acceleration = Steps /(second)^2
  stepper2.setMaxSpeed(3200); // Speed = Steps / second
  stepper2.setAcceleration(800); // Acceleration = Steps /(second)^2
  stepper3.setMaxSpeed(3200); // Speed = Steps / second
  stepper3.setAcceleration(800); // Acceleration = Steps /(second)^2

  // Disable outputs for all steppers initially
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i]->disableOutputs(); //disable power
    steppers[i]->setCurrentPosition(0); //Reset current position. "new home"            
    Serial.print("The current position is updated to: "); //Print message
  }
}

void loop() {
  if (Serial.available() > 0) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
      //assume data is read as Angle_M1 Speed_M1 Accel_M1 Angle_M2 Speed_M2 Accel_M2 Angle_M3 Speed_M3 Accel_M3
      receivedSteps[i] = Serial.parseFloat() * 3200 / 360; //value for the steps
      receivedSpeed[i] = Serial.parseFloat();
      receivedAcceleration[i] = Serial.parseFloat();

      RotateAbsolute();
    }
  }
  else {
     for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i]->stop(); //stop motor
      steppers[i]->disableOutputs(); //disable power
     }
  }

}

void GoHome() {
  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (steppers[i]->currentPosition() == 0) {
      Serial.println("Motor " + String(i + 1) + " is already at home position.");
      steppers[i]->disableOutputs(); // Disable power if already at home
    } else {
      steppers[i]->setMaxSpeed(400); // Set speed manually to 400 steps/sec (1 rev/sec)
      steppers[i]->moveTo(0); // Move to absolute position 0
    }
  }
}

void RotateAbsolute() {

  for (int i = 0; i < MAX_STEPPERS;i++) {
    if (receivedSpeed[i] < 0)
      directionMultiplier = -1;
    else
      directionMultiplier = 1;
   steppers[i]->setMaxSpeed(receivedSpeed[i]); //set speed
   steppers[i]->setAcceleration(receivedAcceleration[i]); //set acceleration
   steppers[i]->moveTo(directionMultiplier * receivedSteps[i]); //set relative distance 
  }
}
