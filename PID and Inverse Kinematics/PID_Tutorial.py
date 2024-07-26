#include <AccelStepper.h>

const int dirPin = 2; //direction Pin
const int stepPin = 3; //pulse Pin
const int enPin = 4; //enable Pin

int directionMultiplier = 1;
// PID constants (students can edit these to adjust accuracy)
float kp = 1; 
float kd = 0;
float ki = 0;
int output = 0; //output from PID algorithm
int outputPrev = 0; //previous output to help determine direction of motor
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); //create instance of stepper

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 2000.0; // Maximum integral limit
const float MIN_INTEGRAL = -2000.0; // Minimum integral limit

float integral = 0; //integral term

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(12800); // Speed = Steps / second
  stepper.setAcceleration(12800); // Acceleration = Steps /(second)^2
  stepper.setSpeed(12800);
  stepper.disableOutputs(); // Disable outputs initially
  stepper.setCurrentPosition(0); //zero current stepper position
  stepper.enableOutputs();
}

void loop() {
  PID();
}

void PID() {

  int target = sin(prevT/10e6) * 1600; // This is the target step we wish to achieve
  Serial.print(target); //print out the target
  Serial.print(",");

  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  // PID calculation
  int error = target - stepper.currentPosition();
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  // control signal
  output = kp*error + kd*derivative + ki*integral;
  Serial.print(output); //print output
  Serial.print(",");

 // determine direction
 /* if (output < outputPrev)
    directionMultiplier = -1;
  else 
    directionMultiplier = 1;*/

  outputPrev = output; //reset previous output
  stepper.move(output); //set target position to output
  stepper.setSpeed(output); //set speed (students can edit these to adjust accuracy)
  while(stepper.distanceToGo() != 0) //move the stepper to the position
    stepper.run();
  
  Serial.print(stepper.currentPosition()); //print stepper position
  Serial.println(); 
}
