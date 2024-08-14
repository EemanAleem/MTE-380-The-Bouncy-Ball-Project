#include <AccelStepper.h>

const int enPin = 2; //enable Pin
const int dirPin = 4; //direction Pin
const int stepPin = 3; //pulse Pin

int directionMultiplier = 1;
// PID constants (students can edit these to adjust accuracy)
float kp = 1; //*1
float kd = 0.025; //*2
float ki = 0; //*3
int output = 0; //output from PID algorithm

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); //create instance of stepper

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 100.0; // Maximum integral limit
const float MIN_INTEGRAL = -100.0; // Minimum integral limit

float integral = 0; //integral term

void setup() {
  Serial.begin(9600);
  stepper.disableOutputs(); // Disable outputs initially
  stepper.setMaxSpeed(10000);
  stepper.setCurrentPosition(0); //zero current stepper position
  stepper.enableOutputs();
}

void loop() {
  PID();
}

void PID() {

  float target = 135.0 * 1023 / 270; // *4 This is the target step we wish to achieve
  target = constrain(target, 0, 1023);
  Serial.print("target "); //print out the 
  Serial.print(target); 
  Serial.print(",");

  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  // PID calculation
  Serial.print("potenValue ");
  Serial.print(analogRead(A0)); //print out the target
  Serial.print(",");


  int error = target - analogRead(A0);
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  // control signal
  float output = kp*error + kd*derivative + ki*integral;
  Serial.print("output "); //print 
  Serial.print(output); //print output
  Serial.print(",");

  float stepperTarget = round(((((output * 270) / 1023) * 3200) / 360));
  stepperTarget = (constrain(stepperTarget, -2400, 2500));
  Serial.print("stepperTarget ");
  Serial.println(stepperTarget);
  stepper.move(stepperTarget);
 long currT2 = millis();
 while (analogRead(A0) < 1023 && analogRead(A0) > 0 && (millis() - currT2) < 50) {
    //  Serial.print((millis() - currT2));
    //  Serial.print(","); 
    //  Serial.print(stepper.currentPosition());
    //  Serial.println();
      stepper.setSpeed(2 * stepperTarget);
      stepper.runSpeed();
 }

  //Serial.println(); 
}
