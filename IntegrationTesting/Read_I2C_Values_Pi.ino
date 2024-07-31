#include <Wire.h>

const int SDA_Pin = 20;
const int SCL_Pin = 21;
// byte c, f, temp;
volatile long int c,f,rV, temp, int1, int2;
volatile int count = 1;
 
void setup() {
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  Wire.setClock(400000);
  Serial.begin(9600);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  while (Wire.available()) {
    // Serial.print("count: ");
    // Serial.println(count);
    c = Wire.read();
    if (count%2 != 0)
      int1 = c;
    else {
      int2 = c;
      Serial.print("int1: ");
      Serial.println(int1);
      Serial.print("int2: ");
      Serial.println(int2);

      rV = (int1 << 8) | int2;
      Serial.print("rV: ");
      Serial.println(rV);
    }
    count++;
    delay(20);
  }
}

void loop() {
  delay(100); // Keep waiting for data
}

