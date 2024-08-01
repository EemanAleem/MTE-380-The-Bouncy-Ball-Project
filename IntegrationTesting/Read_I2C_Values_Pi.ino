#include <Wire.h>

// I2C lines
const int SDA_Pin = 20;
const int SCL_Pin = 21;

// Data parsing in receiveEvent: ******************************************************************
// rB is byte received from transmission; int1 and int2 are temporary values for rV;
// count is used to check whether we have received a first or second byte;
// rV is two bytes combined with a bitwise operation to get an integer in the range -32,768 to 32,767
volatile long int receivedByte, receivedValue;
volatile long int int1, int2;
volatile int count = 1;
 
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
  if (count%2 != 0)
    int1 = receivedByte;
  else {
    int2 = receivedByte;
    // Serial.print("int1: ");
    // Serial.println(int1);
    // Serial.print("int2: ");
    // Serial.println(int2);

    // int1 << 8 shifts the bits of int1 to the left by 8 positions. This effectively multiplies int1 by 256 and places it in the high byte of a 16-bit integer.
    // The bitwise OR operation combines the shifted int1 with int2 to form the final 16-bit value. int2 is placed in the lower 8 bits of the result.
    receivedValue = (int1 << 8) | int2;
    // Serial.print("rV: ");
    Serial.println(receivedValue);
  }
  count++;
  delay(20);
}

void loop() {
  delay(100); // Keep waiting for data
}

