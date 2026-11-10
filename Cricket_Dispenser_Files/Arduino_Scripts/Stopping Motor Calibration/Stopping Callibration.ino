#include <Servo.h> // library for controlling the servo

Servo motor; // creates the servo object

const int SERVO_PIN = 9; // servo control wire
const int STOP_PULSE = 1513; // This is the value to change. Re-upload each time value is changed. If the servo drifts, adjust by 1 microsecond.


void setup() {
  motor.attach(SERVO_PIN); // connects the servo to pin 9
  motor.writeMicroseconds(STOP_PULSE); // sends the pulse being tested
}

void loop() {
  // blank loop means setup sends the STOP_PULSE. No commands to loop through. 
}
