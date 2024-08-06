#include <Wire.h>

const int ledPin = 13; 
 
void setup() {
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);

}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  while (Wire.available()) { // Loop until I2C connection unavailable
    int c = Wire.read(); // receive byte as an integer
    digitalWrite(ledPin, c); // turn on/off LED based on byte information
  }
}

void loop() {
  delay(100); // Keep waiting for data
}
