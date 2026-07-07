/*
 * Cricket dispenser
 * Turns the servo motor 45 degrees every 10min
 * to dispense 1 cricket from an 8-cricket carosel
 */

// required library for servo
#include <Servo.h>;
Servo myServo;

// servo constants
const int SERVO_PIN = 9;
const int START_SERVO = 1280;
const int STOP_SERVO = 1500;
const int ROTATION_TIME = 120; // 45 degrees

// trial counters
int trial_counter = 0;
const int TRIAL_MAX = 8;
const int ITI = 10 * 60 * 100; // 10min ITI (in microseconds)

// TTL constants
const int TTL_PIN = A0;
const int START_TTL = 1;
const int END_TTL = 0;

void setup() {
  // initialize the data stream
  Serial.begin(9600);

  // initialize servo
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(STOP_SERVO);

  // initialize TTLs
  pinMode(TTL_PIN,OUTPUT);

  // display column names for output
  char buffer[50];
  sprintf(buffer, "Trial,Time");
  Serial.println(buffer);
}

void loop() {
  // run until all 8 crickets dispensed
  if (trial_counter < TRIAL_MAX) {

    // output TTL pulse
    digitalWrite(TTL_PIN,START_TTL);

    // print trial time & number to screen
    char buffer[50];
    sprintf(buffer, "%d,%lu",trial_counter,millis());
    Serial.println(buffer);

    // rotate 45 degrees
    myServo.writeMicroseconds(START_SERVO);
    delay(ROTATION_TIME); 
    myServo.writeMicroseconds(STOP_SERVO);

    // end TTL
    digitalWrite(TTL_PIN,END_TTL);
    
    // wait ITI before next trial
    delay(ITI - ROTATION_TIME); 

    trial_counter++;
    }
}
