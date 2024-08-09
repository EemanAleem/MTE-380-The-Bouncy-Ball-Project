#include <Wire.h>

const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 47; 
 
void setup() {
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  Serial.begin(9600);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  int c = Wire.read(); // receive byte as an integer
  digitalWrite(ledPin, c); // turn on/off LED based on byte information
  Serial.println(c);
}

void loop() {
  delay(100); // Keep waiting for data
}
