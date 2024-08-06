#include <AccelStepper.h>

const int dirPin = 2; //direction Pin
const int stepPin = 3; //pulse Pin
const int enPin = 4; //enable Pin

int directionMultiplier = 1;
// PID constants (students can edit these to adjust accuracy)
float kp = 1; //*1
float kd = 1; //*2
float ki = 0.025; //*3
int output = 0; //output from PID algorithm

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); //create instance of stepper

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 2000.0; // Maximum integral limit
const float MIN_INTEGRAL = -2000.0; // Minimum integral limit

float integral = 0; //integral term

void setup() {
  Serial.begin(9600);
  stepper.disableOutputs(); // Disable outputs initially
  stepper.setCurrentPosition(0); //zero current stepper position
  stepper.enableOutputs();
}

void loop() {
  PID();
}

// The
void PID() {

  int target = sin(prevT/15e6) * 1600; // *4 This is the target step we wish to achieve
  Serial.print(target); //print out the target
  Serial.print(",");

  stepper.runSpeed();
  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  stepper.runSpeed();
  // PID calculation
  int error = target - stepper.currentPosition();
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  stepper.runSpeed();
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  stepper.runSpeed();
  // control signal
  output = kp*error + kd*derivative + ki*integral;
  Serial.print(output); //print output
  Serial.print(",");
 
  stepper.runSpeed();
  outputPrev = output; //reset previous output
  stepper.move(output); //set target position to output
  stepper.setSpeed(10 * output); //*5 sets speed (students can edit these to adjust accuracy)

  stepper.runSpeed();
  stepper.runSpeed();
  stepper.runSpeed();
  Serial.print(stepper.currentPosition()); //print stepper position
  Serial.println(); 
}
