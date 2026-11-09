#include <Servo.h> // library for controlling the servo

Servo motor; // creates the servo object

const int SERVO_PIN = 9; // servo control wire
const int STOP_PULSE = 1513; // This is the value to change. Re-upload each time value is changed.

void setup() {
  motor.attach(SERVO_PIN); // connects the servo to pin 9
  motor.writeMicroseconds(STOP_PULSE); // sends the pulse being tested
}

void loop() {
  // blank loop means setup keeps sending the STOP_PULSE
}