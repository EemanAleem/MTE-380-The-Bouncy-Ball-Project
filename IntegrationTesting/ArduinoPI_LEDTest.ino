#include <Wire.h>

const int ledPin = 13; 
 
void setup() {
  // Join I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  while (Wire.available()) { // Loop until I2C connection unavailable
    char y = Wire.read(); // receive byte as a character
    digitalWrite(ledPin, c); // turn on/off LED based on byte information
  }
}

void loop() {
  delay(100); // Keep waiting for data
}
