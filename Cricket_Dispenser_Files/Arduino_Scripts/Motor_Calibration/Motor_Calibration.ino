/*
  Motor Calibration

  Finds the stop pulse for a Parallax Feedback 360° High-Speed Servo used in
  the 8-trial cricket dispenser. See the Parallax servo guide (linked in the
  protocols.io protocol) for details.

  Written by Grace Brown, Oct 2026.
  Code and documentation refactored by Emily Aery Jones
  with assistance from Claude Code, Nov 2026.

  This servo receives a control pulse every 20 ms. The width of the pulse
  sets the direction and speed of rotation. The width that holds the motor
  still, STOP_PULSE, is usually between 1480 and 1520 microseconds and varies
  between servos.

  To calibrate:
    1. Upload this sketch and watch the servo.
    2. If it drifts, change STOP_PULSE by 1 microsecond and upload again.
    3. Repeat until the servo is completely still.
    4. Copy the final value into STOP_PULSE in the dispensing sketches.
*/

#include <Servo.h>

const int SERVO_PIN  = 9;     // servo control (white wire)
const int STOP_PULSE = 1513;  // value being tested, in microseconds; adjust by 1 if the servo drifts

Servo servo;


void setup() {
  // Sets the stop pulse before attaching, so the first pulse sent is a stop.
  servo.writeMicroseconds(STOP_PULSE);
  servo.attach(SERVO_PIN);
}


void loop() {
  // Nothing to do: once attached, the Servo library keeps sending STOP_PULSE
  // every 20 ms in the background until the Arduino is reset or powered off.
}
