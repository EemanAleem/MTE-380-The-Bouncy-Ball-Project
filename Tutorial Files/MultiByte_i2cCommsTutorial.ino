#include <Wire.h>

// I2C lines
const int SDA_Pin = 20;
const int SCL_Pin = 21;

// Data parsing in receiveEvent: ******************************************************************
// receivedByte is the byte received from transmission; byte1 and byte2 are temporary values for receivedValue;
// countByte is used to check whether we have received a first or second byte;
// receivedValue is two bytes combined with a bitwise operation to get an integer in the range 0 to 65,534.
volatile long int receivedByte, receivedValue;
volatile int byte1, byte2;
volatile long int steps[3];
volatile unsigned long long int countByte = 1, countValue = 1;
 
void setup() {
  Wire.begin(0x8); // Arduino joins I2C bus as slave with address 8
  Wire.setClock(400000);  // Set clock speed to 400 kHz, increases speed of transmissions
  Serial.begin(9600); //Sets up serial monitor to display received bytes and values
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  receivedByte = Wire.read();

  // If the byte counter is odd, it's the first byte
  if (countByte%2 != 0)
    byte1 = receivedByte;
  // If the byte counter is even, it's the second byte
  else {
    byte2 = receivedByte;

    // Display received bytes
    Serial.print("byte 1: ");
    Serial.println(byte1);
    Serial.print("byte 2: ");
    Serial.println(byte2);

    // byte1 << 8 shifts the bits of byte1 to the left by 8 positions, making it the MSB. 
    // This effectively multiplies byte1 by 256 and places it in the high byte of a 16-bit integer.
    // The bitwise OR operation combines the MSB byte1 with the now LSB byte2 to form the final 16-bit value.
    // General format: variable = (MSB << 8) | LSB;
    receivedValue = (byte1 << 8) | byte2;

    // If the value counter follows the sequence in the condition below, it's the step[0] value so save it there.
    // The other conditions follow the same logic as the first. 
    // This is needed to accurately determine what transmission is slotted for what array value.
    if ( (countValue - 1) % 3 == 0 ) {
      steps[0] = receivedValue;
      Serial.print("steps[0]: ");
      Serial.println(steps[0]);
    }
    else if ( (countValue - 2) % 3 == 0 ) {
      steps[1] = receivedValue;
      Serial.print("steps[1]: ");
      Serial.println(steps[1]);
    }
    else {
      steps[2] = receivedValue;
      Serial.print("steps[2]: ");
      Serial.println(steps[2]);
    }
    // Increment the Value counter.
    countValue++;
  }
  // Increment the Byte counter
  countByte++;
  // Add a delay between transmissions. *1
  delay(20);
}

void loop() {
  delay(100); // Or add a delay between transmissions here *2
}
