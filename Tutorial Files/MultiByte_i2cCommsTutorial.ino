#include <Wire.h>

// I2C lines
const int SDA_Pin = 20;
const int SCL_Pin = 21;

// Data parsing in receiveEvent: ******************************************************************
// receivedByte is byte received from transmission; byte1 and byte2 are temporary values for receivedValue;
// countByte is used to check whether we have received a first or second byte;
// receivedValue is two bytes combined with a bitwise operation to get an integer in the range -32,768 to 32,767
volatile long int receivedByte, receivedValue;
volatile long int byte1, byte2;
volatile long long int countByte = 1, countValue = 1;
volatile long int steps[3];
 
void setup() {
  Wire.begin(0x8); // Arduino joins I2C bus as slave with address 8
  Wire.setClock(400000);  // Set clock speed to 400 kHz
  Serial.begin(9600);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  receivedByte = Wire.read();

  // If the counter is odd, therefore a first byte.
  if (countByte%2 != 0)
    byte1 = receivedByte;
  else {
    byte2 = receivedByte;
    Serial.print("byte 1: ");
    Serial.println(byte1);
    Serial.print("byte 2: ");
    Serial.println(byte2);

    // byte1 << 8 shifts the bits of byte1 to the left by 8 positions. This effectively multiplies byte1 by 256 and places it in the high byte of a 16-bit integer.
    // The bitwise OR operation combines the shifted byte1 with byte2 to form the final 16-bit value. byte2 is placed in the lower 8 bits of the result.
    receivedValue = (byte1 << 8) | byte2;
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
    countValue++;
  }
  countByte++;
  delay(20);
}

void loop() {
  delay(100); // Keep waiting for data
}
