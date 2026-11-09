/*
Written by Grace Brown, Aug 2026
For use with the Parallax Feedback 360° High-Speed Servo
and an 8-trial cricket dispenser.

Moves to eight targets spaced 45 degrees apart, with a specified
ITI between trials. Stops after the eighth attempted trial.

Reads pulse-width modulation (PWM) feedback to calculate angular
position. Each target is calculated from the initial home angle.

Commands a stop when the measured position is within 3 degrees
of the target. Reports a timeout if this threshold is not reached
within 5 seconds. The settled endpoint may differ from the position
at which the stop was commanded.

Code works in a series of steps: 
1. Measure the feedback pulses. 
2. Calculate the duty cycle. 
3. Convert duty cycle to theta, between 0 and 359 degs. 
4. Compare theta to previous theta so Arduino detects a zero crossing. 
5. Update turns ++ and calculate the continuous position. 
6. Compare updated position with target angle. Repeat readings until within 
angle tolerance. 
7. Send a STOP_PULSE, settle for 20 microsecs, record trial success/failure. 
8. Increase trial counter until TRIAL_MAX is reached.

Sends a digital high/low TTL through A0 to connected
recording equipment. 
*/

#include <Servo.h>
Servo myServo;

// assigns the servo control, feedback, and timing output pins
const int SERVO_PIN = 9; // white control wire
const int FEEDBACK_PIN = 2; // yellow feedback wire
const int TTL_PIN = A0; // timing output to recording equipment

// defines the stop pulse and response to angular error
const int SERVO_CENTER = 1513; // replace with the calibrated stop value
const float KP = 1.5; // proportional gain used in this implementation, increased from Parralax guide due to motor moving too slowly under load.
const int CONTROL_OFFSET = 30; // adds an adjustment outside the stop region, value borrowed from Parralx guide. 
const int MAX_OUTPUT = 200; // limits the proportional adjustment, value borrowed from Parralax guide. 
const int SERVO_MIN = 1280; // lowest allowed control pulse, in microseconds
const int SERVO_MAX = 1720; // highest allowed control pulse, in microseconds

// converts feedback duty cycle into degrees; see the Parallax servo guide for more deets 
const int DC_MIN = 29; // minimum duty cycle in thousandths: 2.9%, borrowed from Parallax guide 
const int DC_MAX = 971; // maximum duty cycle in thousandths: 97.1%, borrowed from Parallax guide 
const int UNITS_FC = 360; // angular units in one full circle

// defines the target spacing and movement limits
const long ROTATION_ANGLE = -45; // advances targets clockwise
const int ANGLE_TOLERANCE = 3; // target reached when measured error is within 3 degrees
const unsigned long MOVE_TIMEOUT = 5000; // movement timeout of 5 seconds
const unsigned long CONTROL_INTERVAL = 20; // waits 20 ms between control updates

// counts 8 attempted trials and sets the pause between them
int trial_counter = 0; // trials are numbered 0 through 7
const int TRIAL_MAX = 8;
const unsigned long ITI = 60000UL; // 1 minute, in milliseconds

// tracks angle within a rotation and crossings through zero
long servo_angle = 0; // initial placeholder; updated from feedback
int theta_previous = 0; // previous angle used to detect crossings through zero
long turns = 0; // full-rotation count, which can be positive or negative
bool first_feedback = true; // true until the first valid reading is received
long home_angle = 0; // stores the initial measured angle as a fixed reference

void setup() {
  // starts serial communication and configures the pins
  Serial.begin(9600);
  pinMode(TTL_PIN, OUTPUT);
  digitalWrite(TTL_PIN, LOW);
  pinMode(FEEDBACK_PIN, INPUT);

  // prepares the stop pulse before starting output through pin 9
  myServo.writeMicroseconds(SERVO_CENTER);
  myServo.attach(SERVO_PIN);
  delay(500); // allows 500 milliseconds for the servo to settle

  // waits for valid feedback, retrying after a 20 ms pause
  while (!updatePosition()) {
    delay(20);
  }

  // saves the starting reference; the measured angle is not reset to zero
  home_angle = servo_angle;

  Serial.print("Home angle: ");
  Serial.println(home_angle);
  Serial.println(
    "Trial,Time,StartAngle,TargetAngle,EndAngle,Error,MoveTime,Success"
  );
}

void loop() {
  // stops attempting movements after trial 7
  // loop still repeats, but returns immediately
  if (trial_counter >= TRIAL_MAX) return;

  // obtains a valid position before starting the next movement
  while (!updatePosition()) {
    delay(20);
  }

  long start_angle = servo_angle; // records the position before movement

  // calculates every target from home rather than the previous endpoint
  long target_angle =
    home_angle + ROTATION_ANGLE * (trial_counter + 1);

  // marks and times the movement attempt
  digitalWrite(TTL_PIN, HIGH);
  unsigned long move_start = millis();

  // attempts movement and stores whether the stopping tolerance was reached
  bool success = moveToAngle(target_angle);

  unsigned long move_time = millis() - move_start; // elapsed attempt time in ms
  digitalWrite(TTL_PIN, LOW); // ends the timing signal after the attempt

  // allows settling and checks whether a fresh endpoint reading is available
  delay(100);
  bool endpoint_valid = updatePosition();

  // prints the first four reporting columns
  Serial.print(trial_counter);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(start_angle);
  Serial.print(",");
  Serial.print(target_angle);
  Serial.print(",");

  // prints endpoint and error only if the final feedback reading suceeded
  if (endpoint_valid) {
    long end_angle = servo_angle;//updates the end angle to be the servo angle 
    long final_error = target_angle - end_angle;//shows discrepancy between intended angle and target angle

    Serial.print(end_angle);
    Serial.print(",");
    Serial.print(final_error);
  }
  else {
    Serial.print("NA,NA"); // marks "NA" (missing) values if failed
  }

  // completes the print line with movement duration and success
  Serial.print(",");
  Serial.print(move_time);
  Serial.print(",");
  Serial.println(success);

  // reports feedback failure separately from movement timeout
  if (!endpoint_valid) {//if endpoint is not valid
    Serial.print("WARNING: endpoint feedback unavailable on trial ");//print this warning
    Serial.println(trial_counter);// print the current trial number
  }

  if (!success) {
    Serial.print("WARNING: movement timeout on trial ");
    Serial.println(trial_counter);
  }

  // counts the attempt even if it failed
  trial_counter++;

  // waits only if another trial remains; the servo retains its stop command
  if (trial_counter < TRIAL_MAX) {
    delay(ITI);
  }
}

bool moveToAngle(long target_angle) {
  unsigned long start_time = millis(); // starts the timeout timer

  // repeatedly reads position and adjusts the motor until success or timeout
  while (millis() - start_time < MOVE_TIMEOUT) {

    // stops while feedback is unavailable, then retries
    if (!updatePosition()) {
      myServo.writeMicroseconds(SERVO_CENTER);
      delay(CONTROL_INTERVAL);
      continue;
    }

    long error_angle = target_angle - servo_angle;

    // stops and reports success when measured error enters the allowed range
    if (abs(error_angle) <= ANGLE_TOLERANCE) {
      myServo.writeMicroseconds(SERVO_CENTER);// "motor: stop!"
      return true;//ends moveToAngle function, returns success = true  
    }

    // scales the adjustment with angular error, then caps its size, comes from Parallax guide 
    int output = error_angle * KP;
    output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

    // adds an offset in the required direction to get outside the stop region
    int offset = 0;
    if (error_angle > 0) {
      offset = CONTROL_OFFSET;
    }
    else if (error_angle < 0) {
      offset = -CONTROL_OFFSET;
    }

    // builds the final pulse and keeps it within the allowed limits
    int servo_command = SERVO_CENTER + output + offset;
    servo_command = constrain(servo_command, SERVO_MIN, SERVO_MAX);

    myServo.writeMicroseconds(servo_command);
    delay(CONTROL_INTERVAL);
  }

  // stops and reports failure when the movement times out
  myServo.writeMicroseconds(SERVO_CENTER);
  return false;
}
//next block checks to see if feedback wire's emitted signal is usable prior to calculating an angle
bool updatePosition() {//true if position is updated, false if feedback signal fails 
  unsigned long t_high;// stores length of time t_high as a long int
  unsigned long t_low;//stroes length of time t_low as a long int
  unsigned long t_cycle;//stores combined high and low duration
  bool valid_cycle = false;//begin as false because no reading has passed check 

  // starts at attempt 0, add 1 attempt each attempt up to 20
  for (int attempt = 0; attempt < 20; attempt++) {
    t_high = pulseIn(FEEDBACK_PIN, HIGH, 3000);// appends the feedback pin to record a high pulse, 3000 microsec timeout
    t_low = pulseIn(FEEDBACK_PIN, LOW, 3000);// appends feedback pin to record a low pusle, 3000 microecs timeout 

    // if either t_high or t_low zero, continue to next attempt, up to 20
    if (t_high == 0 || t_low == 0) continue;

    t_cycle = t_high + t_low;// estimates full cycle duration

    // expected cycle is about 1100 microsecs, accepts readings between 1k and 1.2k
    if (t_cycle > 1000 && t_cycle < 1200) {
      valid_cycle = true;
      break;//break the "retry" loop because cycle was found to be valid 
    }
  }

  // leaves the stored position unchanged if feedback could not be validated
  if (!valid_cycle) return false;

  // calculates duty cycle in thousandths and converts it to angular position
  long duty_cycle = (1000L * t_high) / t_cycle;// calculates what fraction of the signla cycle is high, borrowed from Parallax guide 

  long calculated_theta =
    (UNITS_FC - 1) -
    ((duty_cycle - DC_MIN) * UNITS_FC) /
    (DC_MAX - DC_MIN + 1);// formula maps the feedback value onto angular units, borrowed from Parralax guide

  int theta = constrain(calculated_theta, 0, UNITS_FC - 1);// limits value: calculated theta to exist between 0 and 359

  // initializes tracking without treating the first reading as movement, for example if motor started at postion 330
  // prevent comparing with placeholder theta_previous =0 and count that as a -1 turn
  if (first_feedback) {
    theta_previous = theta;
    servo_angle = theta;
    first_feedback = false;
    return true;
  }

  // counts crossings between the high and low ends of the angle range
  if (theta < 90 && theta_previous > 270) {
    turns++;
  }// previous reading was a high angle value, a subsequent low reading does not mean the motor moved nearly 360 degrees. turn would be +1 
  else if (theta_previous < 90 && theta > 270) {
    turns--;
  }//previous reading was low angle value, subsequent high reading does not mean motor moved nearly 360 degrees. turn would be -1

  // combines full rotations and the current angle
  // this is mathematically equivalent to the original positive/negative branches
  servo_angle = turns * UNITS_FC + theta;//for example, servo_angle= -1 * 360 + 358 = -2 degrees
  theta_previous = theta;// updates the step 

  return true;
}