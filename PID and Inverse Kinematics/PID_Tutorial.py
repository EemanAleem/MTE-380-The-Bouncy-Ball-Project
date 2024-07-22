#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3; //pulsePin
const int enPin = 4;

int directionMultiplier = 1;
// PID constants
float kp = 1;
float kd = 0.025;
float ki = 0.0;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

long prevT = 0;
float errorPrev = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(6400); // Speed = Steps / second
  stepper.setAcceleration(800); // Acceleration = Steps /(second)^2
  stepper.disableOutputs(); // Disable outputs initially
  stepper.setCurrentPosition(0);
  stepper.enableOutputs();
}

void loop() {
  PID();
}

void PID() {

  int target = 360*sin(prevT/4e6) * 3200 / 360;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // PID calculation
  int error = stepper.currentPosition() - target;
  integral += integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);

  // control signal
  float output = kp*error + kd*derivative + ki*integral;

  if (output > 0)
    directionMultiplier = -1;
  else 
    directionMultiplier = 1;
  stepper.moveTo(directionMultiplier * output);
  stepper.run();
}
