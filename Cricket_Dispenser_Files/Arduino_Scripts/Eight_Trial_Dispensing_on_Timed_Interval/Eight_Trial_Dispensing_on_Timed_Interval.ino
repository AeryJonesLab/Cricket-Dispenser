/*
  Eight-Trial Dispensing on a Timed Interval

  Rotates an 8-trial cricket dispenser to eight targets spaced 45 degrees
  apart, with a fixed inter-trial interval (ITI) between trials. Stops after
  the eighth attempted trial; reset the Arduino to run again.

  Hardware: Parallax Feedback 360° High-Speed Servo and an Arduino Uno.
  See Dispenser_Pinout.txt for wiring and the Parallax servo guide (linked
  in the protocols.io protocol) for details on the feedback signal.

  Before running this code, update the STOP_PULSE value to the value
  that was determined from running Motor_Calibration.ino

  Written by Grace Brown, Aug 2026.
  Code and documentation refactored by Emily Aery Jones
  with assistance from Claude Code, Nov 2026.

  HOW IT WORKS
  ------------
  Position tracking: the servo's feedback wire carries a PWM signal whose
  duty cycle encodes the shaft angle (0-359 degrees). updatePosition()
  measures that signal, converts it to an angle, and counts zero crossings
  so that the position stays continuous across full rotations.

  Movement: every target is calculated from the home angle measured at
  startup, so positioning errors do not accumulate across trials.
  moveToAngle() uses proportional control and commands a stop once the
  measured position is within ANGLE_TOLERANCE_DEG of the target. If the
  target is not reached within MOVE_TIMEOUT_MS, the trial is reported as a
  timeout. The settled endpoint may differ slightly from the position at
  which the stop was commanded.

  Each trial:
    1. Read the current position and calculate the next target.
    2. Set the TTL output HIGH and move toward the target.
    3. Stop the servo, set the TTL output LOW, and wait SETTLE_MS.
    4. Read the settled position and print one CSV row.
    5. Wait ITI_MS while continuing to track position, then repeat.

  SERIAL OUTPUT (9600 baud)
  -------------------------
  One CSV row per trial, with these columns:
    Trial        trial number, 0-7
    Time         trial start (TTL output set HIGH), ms since startup
    StartAngle   position before the movement, degrees
    TargetAngle  commanded target, degrees
    EndAngle     settled position, degrees (NA if feedback failed)
    Error        TargetAngle - EndAngle, degrees (NA if feedback failed)
    MoveTime     duration of the movement attempt, ms
    Success      1 if the target tolerance was reached, 0 on timeout

  Status and warning lines begin with '#' so they can be skipped when the
  log is parsed (for example, pandas.read_csv(path, comment='#')).

  The TTL output on A0 is HIGH for the duration of each movement attempt.
*/

#include <Servo.h>

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
const int SERVO_PIN    = 9;   // servo control (white wire)
const int FEEDBACK_PIN = 2;   // servo feedback (yellow wire)
const int TTL_PIN      = A0;  // timing output to recording equipment

// ---------------------------------------------------------------------------
// Servo control; pulse widths in microseconds
// ---------------------------------------------------------------------------
const int   STOP_PULSE     = 1513;  // set to the value found with Motor_Calibration
const float KP             = 1.5;   // proportional gain; raised from the Parallax
                                    // value because the motor moved too slowly under load
const int   CONTROL_OFFSET = 30;    // moves commands outside the stop region (Parallax guide)
const int   MAX_OUTPUT     = 200;   // caps the proportional adjustment (Parallax guide)
const int   SERVO_MIN      = 1280;  // lowest allowed control pulse
const int   SERVO_MAX      = 1720;  // highest allowed control pulse

// ---------------------------------------------------------------------------
// Feedback signal (values from the Parallax guide)
// ---------------------------------------------------------------------------
const int DC_MIN            = 29;   // minimum duty cycle, thousandths (2.9%)
const int DC_MAX            = 971;  // maximum duty cycle, thousandths (97.1%)
const int UNITS_FULL_CIRCLE = 360;  // angular units in one full rotation

const unsigned long PULSE_TIMEOUT_US = 3000;  // longest wait for one feedback pulse
const unsigned long CYCLE_MIN_US     = 1000;  // the feedback cycle is ~1100 us (910 Hz);
const unsigned long CYCLE_MAX_US     = 1200;  //   readings outside this range are rejected
const int           FEEDBACK_RETRIES = 20;    // measurements tried before giving up

// A jump from above CROSSING_HIGH_DEG to below CROSSING_LOW_DEG (or the
// reverse) between readings is treated as crossing zero, not as a large move.
const int CROSSING_LOW_DEG  = 90;
const int CROSSING_HIGH_DEG = 270;

// ---------------------------------------------------------------------------
// Movement
// ---------------------------------------------------------------------------
const long          ROTATION_ANGLE_DEG  = -45;   // step between targets; negative = clockwise
const int           ANGLE_TOLERANCE_DEG = 3;     // target reached when |error| <= this
const unsigned long MOVE_TIMEOUT_MS     = 5000;  // movement attempt limit
const unsigned long CONTROL_INTERVAL_MS = 20;    // time between control updates
const unsigned long SETTLE_MS           = 100;   // pause after stopping, before the endpoint reading
const unsigned long STARTUP_SETTLE_MS   = 500;   // pause after attaching the servo

// ---------------------------------------------------------------------------
// Trials
// ---------------------------------------------------------------------------
const int           NUM_TRIALS = 8;
const unsigned long ITI_MS     = 60000UL;  // inter-trial interval: 1 minute

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Servo servo;

int  trial_counter  = 0;     // number of trials attempted so far
long home_angle     = 0;     // position at startup; all targets are relative to it
long servo_angle    = 0;     // continuous position, degrees (can exceed 0-359)
int  theta_previous = 0;     // previous within-rotation angle, used to detect zero crossings
long turns          = 0;     // net full rotations; positive or negative
bool first_feedback = true;  // true until the first valid reading


void setup() {
  Serial.begin(9600);
  pinMode(TTL_PIN, OUTPUT);
  digitalWrite(TTL_PIN, LOW);
  pinMode(FEEDBACK_PIN, INPUT);

  // Sets the stop pulse before attaching, so the first pulse sent is a stop.
  servo.writeMicroseconds(STOP_PULSE);
  servo.attach(SERVO_PIN);
  delay(STARTUP_SETTLE_MS);

  // Saves the starting position as the reference; it is not reset to zero.
  waitForFeedback();
  home_angle = servo_angle;

  Serial.print(F("# Home angle: "));
  Serial.println(home_angle);
  Serial.println(F("Trial,Time,StartAngle,TargetAngle,EndAngle,Error,MoveTime,Success"));
}


void loop() {
  // Idles once all trials have been attempted.
  if (trial_counter >= NUM_TRIALS) return;

  waitForFeedback();
  runTrial();

  if (trial_counter < NUM_TRIALS) {
    waitWhileTracking(ITI_MS);
  } else {
    Serial.println(F("# All trials attempted. Reset the Arduino to restart."));
  }
}


// Moves to the next target, reports the result, and advances the trial
// counter. A trial is counted even if the movement times out. Expects
// servo_angle to hold a current reading.
void runTrial() {
  long start_angle = servo_angle;

  // Calculates each target from home rather than from the previous endpoint.
  long target_angle = home_angle + ROTATION_ANGLE_DEG * (trial_counter + 1);

  // Records the trial start time together with the TTL HIGH, so the Time
  // column lines up with the TTL edge seen by the recording equipment.
  digitalWrite(TTL_PIN, HIGH);
  unsigned long trial_start = millis();

  bool success = moveToAngle(target_angle);

  unsigned long move_time = millis() - trial_start;
  digitalWrite(TTL_PIN, LOW);

  // Lets the servo settle, then takes a fresh endpoint reading.
  delay(SETTLE_MS);
  bool endpoint_valid = updatePosition();

  printTrialResult(trial_start, start_angle, target_angle, endpoint_valid,
                   move_time, success);

  trial_counter++;
}


// Drives the servo toward target_angle. Returns true once the measured error
// is within ANGLE_TOLERANCE_DEG, or false if MOVE_TIMEOUT_MS elapses first.
// The servo is commanded to stop in both cases.
bool moveToAngle(long target_angle) {
  unsigned long start_time = millis();

  while (millis() - start_time < MOVE_TIMEOUT_MS) {
    // Holds still while feedback is unavailable, then retries.
    if (!updatePosition()) {
      servo.writeMicroseconds(STOP_PULSE);
      delay(CONTROL_INTERVAL_MS);
      continue;
    }

    long error_angle = target_angle - servo_angle;

    if (abs(error_angle) <= ANGLE_TOLERANCE_DEG) {
      servo.writeMicroseconds(STOP_PULSE);
      return true;
    }

    // Proportional term, capped in size (Parallax guide).
    int output = (int)(error_angle * KP);
    output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

    // Adds a fixed offset in the direction of travel to get outside the
    // stop region. error_angle cannot be 0 here, because the tolerance
    // check above has already returned.
    int offset = (error_angle > 0) ? CONTROL_OFFSET : -CONTROL_OFFSET;

    int servo_command = constrain(STOP_PULSE + output + offset, SERVO_MIN, SERVO_MAX);
    servo.writeMicroseconds(servo_command);
    delay(CONTROL_INTERVAL_MS);
  }

  servo.writeMicroseconds(STOP_PULSE);
  return false;
}


// Reads the feedback signal and updates servo_angle. Returns true on success.
// Returns false, leaving servo_angle unchanged, if no valid feedback cycle is
// measured within FEEDBACK_RETRIES attempts.
bool updatePosition() {
  unsigned long t_high = 0;  // HIGH pulse duration, microseconds
  unsigned long t_low = 0;   // LOW pulse duration, microseconds
  unsigned long t_cycle = 0; // t_high + t_low
  bool valid_cycle = false;

  for (int attempt = 0; attempt < FEEDBACK_RETRIES; attempt++) {
    t_high = pulseIn(FEEDBACK_PIN, HIGH, PULSE_TIMEOUT_US);
    t_low = pulseIn(FEEDBACK_PIN, LOW, PULSE_TIMEOUT_US);

    // pulseIn() returns 0 on timeout.
    if (t_high == 0 || t_low == 0) continue;

    t_cycle = t_high + t_low;
    if (t_cycle > CYCLE_MIN_US && t_cycle < CYCLE_MAX_US) {
      valid_cycle = true;
      break;
    }
  }

  if (!valid_cycle) return false;

  // Converts the duty cycle (in thousandths) to an angle from 0 to 359
  // degrees, using the formula from the Parallax guide.
  long duty_cycle = (1000L * t_high) / t_cycle;
  long calculated_theta = (UNITS_FULL_CIRCLE - 1)
                          - ((duty_cycle - DC_MIN) * UNITS_FULL_CIRCLE)
                          / (DC_MAX - DC_MIN + 1);
  int theta = constrain(calculated_theta, 0, UNITS_FULL_CIRCLE - 1);

  // Uses the first reading as the starting point. Without this, a servo that
  // started at, e.g., 330 degrees would be compared with the placeholder
  // theta_previous = 0 and miscounted as a -1 turn.
  if (first_feedback) {
    theta_previous = theta;
    servo_angle = theta;
    first_feedback = false;
    return true;
  }

  // Counts zero crossings. A change from, e.g., 355 to 3 degrees is a
  // +1 turn, not a 352-degree move in the opposite direction.
  if (theta < CROSSING_LOW_DEG && theta_previous > CROSSING_HIGH_DEG) {
    turns++;
  } else if (theta_previous < CROSSING_LOW_DEG && theta > CROSSING_HIGH_DEG) {
    turns--;
  }

  // Combines full rotations with the within-rotation angle,
  // e.g., -1 * 360 + 358 = -2 degrees.
  servo_angle = turns * UNITS_FULL_CIRCLE + theta;
  theta_previous = theta;

  return true;
}


// Blocks until updatePosition() succeeds.
void waitForFeedback() {
  while (!updatePosition()) {
    delay(CONTROL_INTERVAL_MS);
  }
}


// Waits for duration_ms while continuing to read feedback, so that any drift
// across zero during long pauses is still counted in `turns`.
void waitWhileTracking(unsigned long duration_ms) {
  unsigned long start_time = millis();
  while (millis() - start_time < duration_ms) {
    updatePosition();
    delay(CONTROL_INTERVAL_MS);
  }
}


// Prints one CSV row for the trial, followed by any warnings.
void printTrialResult(unsigned long trial_start, long start_angle, long target_angle,
                      bool endpoint_valid, unsigned long move_time, bool success) {
  Serial.print(trial_counter);
  Serial.print(',');
  Serial.print(trial_start);
  Serial.print(',');
  Serial.print(start_angle);
  Serial.print(',');
  Serial.print(target_angle);
  Serial.print(',');

  if (endpoint_valid) {
    long end_angle = servo_angle;
    long final_error = target_angle - end_angle;
    Serial.print(end_angle);
    Serial.print(',');
    Serial.print(final_error);
  } else {
    Serial.print(F("NA,NA"));
  }

  Serial.print(',');
  Serial.print(move_time);
  Serial.print(',');
  Serial.println(success);

  // Reports feedback failure separately from movement timeout.
  if (!endpoint_valid) {
    Serial.print(F("# WARNING: endpoint feedback unavailable on trial "));
    Serial.println(trial_counter);
  }
  if (!success) {
    Serial.print(F("# WARNING: movement timeout on trial "));
    Serial.println(trial_counter);
  }
}
