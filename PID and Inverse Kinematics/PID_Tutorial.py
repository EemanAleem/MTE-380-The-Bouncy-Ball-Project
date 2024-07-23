#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3; //pulsePin
const int enPin = 4;

int directionMultiplier = 1;
// PID constants
float kp = 1;
float kd = 0;
float ki = 0;
int output = 0;
int outputPrev = 0;
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

long prevT = 0;
float errorPrev = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(128000); // Speed = Steps / second
  stepper.setAcceleration(12800); // Acceleration = Steps /(second)^2
  stepper.setSpeed(128000);
  stepper.disableOutputs(); // Disable outputs initially
  stepper.setCurrentPosition(0);
  stepper.enableOutputs();
}

void loop() {
  PID();
}

void PID() {

  int target = 360*sin(prevT/4e6) * 3200 / 360;
  Serial.print(target);
  Serial.print(",");
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // PID calculation
  int error = stepper.currentPosition() - target;
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;

  // control signal
  output = -(kp*error + kd*derivative + ki*integral);
  Serial.print(output);
  Serial.print(",");

  if (output < outputPrev)
    directionMultiplier = -1;
  else 
    directionMultiplier = 1;

  outputPrev = output;
  stepper.moveTo(output);
  stepper.setSpeed(directionMultiplier * 51200);
  while(stepper.currentPosition() != output) 
    stepper.runSpeed();
  
  Serial.print(stepper.currentPosition());
  Serial.println();
}
