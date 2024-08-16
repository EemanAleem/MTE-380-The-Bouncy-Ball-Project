# Integration

The Raspberry Pi file:
1. Scans the yellow ball's center coordinates in the x-y plane in reference to the balancing platform. This is done in the 'detect_yellow_ball' function.
2. Calculates the error between the ball's current position and its target position, done in the 'PID' function.
3. Performs PID calculations.
4. Inverse kinematics are implemented in the 'theta' function to gain the stepper motor leg positions.
5. The position data for each of the stepper motors is then sent via I2C to the Arduino Mega, byte by byte. This is done in the 'SendData' function.

The Arduino File:
1. Performs setup including setting up I2C comms (and itself as a client/slave) and homes the motors.
2. Once a data transmission is received by the Pi, receiveEvent is called. Once two bytes are received, byte merging / byte concatenation is performed to get the original two-byte integers.
3. Data is then stored in their respective indexes in the pos[3] array which stores the stepper motor positions.
4. Speed is calculated for each of the stepper motors.
5. The motors then move according to their received positions until data transmissions are received again.

The code is explained in further detail in their files' comments.
