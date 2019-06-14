#include <CPPM-RX.h>
#include "dshot.h"

//define pins
#define GREEN 23
#define RED 22
#define ACCEL_SCL 19
#define ACCEL_SDA 18
#define ACCEL_NEW 17
#define ACCEL_FLIP 16
#define ESC_DSHOT 15
#define RX_CPPM 14

//define states
#define STATE_STARTUP 0
#define STATE_IDLE 1
#define STATE_CALIBRATE 2
#define STATE_CALIBRATE_EXIT 3
#define STATE_MELTY 4
#define STATE_MAX_SPIN 5
byte state = STATE_STARTUP;
byte prevState = STATE_STARTUP;

//receiver variables
bool sticksNew = false;
bool switchesNew = false;
byte recThrot = 0;
byte recRudd = 0;
byte recElev = 0;
byte recAiler = 0;
byte recGear = 0;
byte recFlap = 0;

//accelerometer variables
bool accelNew = false;
int16_t zAccel; //radial acceleration - used to measure rotation speed
int8_t flipped = 1; //1 for rightside-up, -1 for upside down

//calibration variables
bool calibrateRunning = false;
byte throtMin = 5;
byte throtMax = 50;
uint16_t lightOffset = 50; //angle between lights and "front", which is 90 deg offset from the motor axle
int16_t accelCalA = 0;
int16_t accelCalB = 0;
int16_t accelCalC = 0;

//melty variables
uint16_t movementDirection = 0;
uint16_t movementSpeed = 0;

unsigned long blinkTimer = millis();

void setup() {
  //setup receiver interrupt
  StartCPPM(RX_CPPM);

  //setup accelerometer
  accelSetup();

  //setup LEDs
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);

  //setup ESC DShot out
  pinMode(ESC_DSHOT, OUTPUT);
  digitalWrite(ESC_DSHOT, LOW);
  setupDshotDMA();

  //setup watchdog
  watchdogSetup();

  //read EEPROM stored calibration data
  readCalibration();
}

void loop() {
  feedWatchdog();

  //check for new receiver inputs
  if (AilerNew()) readReceiver();

  //check for switch changes and resulting state changes
  if (switchesNew) stateChange();

  switch (state) {
    case STATE_STARTUP:
      //fast blink green status light to warn of improper startup switches or lost signal
      if (millis() - blinkTimer > 200) {
        blinkTimer = millis();
        digitalWrite(GREEN, !digitalRead(GREEN));
      }
      //send motor stop command
      setMotor(0);
      break;

    case STATE_IDLE:
      //green LED on solid to show idle state
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, HIGH);
      //Send motor stop command
      setMotor(0);
      break;

    case STATE_CALIBRATE:
      //red LED solid to show calibrate mode
      runCalibrate();
      break;

    case STATE_CALIBRATE_EXIT:
      //exit calibration with throttle high to save new calibration data
      calibrateRunning = false;
      if (recThrot < 50) break;
      writeCalibration();
      break;

    case STATE_MELTY:
      //both red and green LED solid while not spinning to show melty mode
      runMelty();
      break;

    case STATE_MAX_SPIN:
      //red LED constant on to show max spin mode
      digitalWrite(RED, HIGH);
      //send motor commands
      setMotor(flipped * 100);
      break;

    default:
      break;
  }
}

void stateChange() {
  //check switches to determine program state to transition to
  switchesNew = false;
  prevState = state;
  //safe state if transmitter connection is lost
  if (RX_Fail()) state = STATE_STARTUP;
  //idle if both switches off
  else if (recFlap <= 50 && recGear <= 50) state = STATE_IDLE;
  //calibrate if gear switch is on but flap switch is off
  else if (recFlap <= 50 && recGear > 50) state = STATE_CALIBRATE;
  //melty if flap switch is on and throttle is low
  else if (recFlap > 50 && recGear <= 50 && recThrot < 10) state = STATE_MELTY;
  //max spin if both switches on
  else if (recFlap > 50 && recGear > 50) state = STATE_MAX_SPIN;

  //check state transitions
  //startup can only exit to idle when both switches off
  if (prevState == STATE_STARTUP && state != STATE_IDLE) state = STATE_STARTUP;
  //calibrate state can only exit to idle, goes through exit state to save calibration data
  else if (prevState == STATE_CALIBRATE && state != STATE_CALIBRATE) {
    if (state == STATE_IDLE) state = STATE_CALIBRATE_EXIT;
    else state = STATE_CALIBRATE;
  }
  //Max spin mode cannot be entered at low throttle to prevent accidents
  else if (prevState == STATE_MELTY && state == STATE_MAX_SPIN && recThrot < 33) state = STATE_MELTY;
  //max spin mode can only exit to melty mode or idle
  else if (prevState == STATE_MAX_SPIN && state != STATE_MAX_SPIN) {
    if (state == STATE_MELTY || state == STATE_IDLE);
    else state = STATE_MAX_SPIN;
  }
}
