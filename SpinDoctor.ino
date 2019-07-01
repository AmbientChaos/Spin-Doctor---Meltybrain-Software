#include <CPPM-RX.h>

//define pins
#define GREEN 23
#define RED 22
#define ACCEL_SCL 19
#define ACCEL_SDA 18
#define ACCEL_NEW 17
#define RX_CPPM 16
#define ESC_1 9 //FTM0_CH2
#define ESC_2 10 //FTM0_CH3

//define states
#define STATE_STARTUP 0
#define STATE_IDLE 1
#define STATE_CALIBRATE 2
#define STATE_CALIBRATE_EXIT 3
#define STATE_DRIVE 4
#define STATE_MELTY 5
#define STATE_MAX_SPIN 6
byte state = STATE_STARTUP;
byte prevState = STATE_STARTUP;

//receiver variables
bool sticksNew = false;
uint16_t recThrot = 0;
uint16_t recRudd = 500;
uint16_t recElev = 500;
uint16_t recAiler = 500;
uint16_t recGear = 0;
uint16_t recFlap = 0;

//accelerometer variables
bool accelNew = true;
const byte arraySize = 2; //2 for triangular integration
unsigned long accelTime[arraySize] = {0}; //time between accelerometer measurements
uint16_t degreePeriod[arraySize] = {0}; //period in microseconds per degree
uint16_t accelAngle[arraySize] = {0}; //current angle calculated from accelerometer
int16_t zAccel; //centrifugal force, +/-2047 scale for +/-400G
int8_t flipped = 1; //1 for rightside-up, -1 for upside down

//calibration variables
bool calibrateRunning = false;
uint16_t throtMin = 100;
uint16_t throtMax = 500;
uint16_t lightOffset = 50; //angle between lights and "front", which is 90 deg offset from the motor axle
int16_t accelCalA = 0;
int16_t accelCalB = 0;
int16_t accelCalC = 0;

//melty variables
uint16_t throtCurrent = 0;
uint16_t movementDirection = 0;
uint16_t movementSpeed = 0;

unsigned long blinkTimer = millis();

//prototypes for functions in different tabs
void dshotOut(uint16_t value, uint8_t motor = 1);
void setMotor(int16_t value, uint8_t motor = 1);

void setup() {
  //setup receiver interrupt
  StartCPPM(RX_CPPM);

  //setup accelerometer
  accelSetup();

  //setup LEDs
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);

  //setup ESC DShot out
  pinMode(ESC_1, OUTPUT);
  digitalWrite(ESC_1, LOW);
  pinMode(ESC_2, OUTPUT);
  digitalWrite(ESC_2, LOW);
  setupDshotDMA();
  dshotOut(0, 1); //arm motor
  dshotOut(0, 2); //arm motor

  //setup watchdog
  watchdogSetup();

  //read EEPROM stored calibration data
  readCalibration();
}

void loop() {
  feedWatchdog();

  //check for new receiver inputs
  if (Aux1New()){
    readReceiver();
    //check for state changes
    stateChange();
  }

  //get any new accelerometer data
  if (accelNew) runAccel();

  switch (state) {
    case STATE_STARTUP:
      //fast blink green status light to warn of improper startup switches or lost signal
      if (millis() - blinkTimer > 200) {
        blinkTimer = millis();
        digitalWrite(GREEN, !digitalRead(GREEN));
        digitalWrite(RED, LOW);
      }
      //send motor stop command
      setMotor(0, 1);
      setMotor(0, 2);
      break;

    case STATE_IDLE:
      //green LED on solid to show idle state
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, HIGH);
      //Send motor stop command
      setMotor(0, 1);
      setMotor(0, 2);
      break;

    case STATE_CALIBRATE:
      //red LED solid to show calibrate mode
      runCalibrate();
      break;

    case STATE_CALIBRATE_EXIT:
      //exit calibration with throttle high to save new calibration data
      calibrateRunning = false;
      if (recThrot < 500) break;
      writeCalibration();
      break;

    case STATE_DRIVE:
      //both red and green LED solid while not spinning to show melty mode
      getSticks();
      uint16_t radDirection = movementDirection * 71 / 4068;
      setMotor(flipped * (movementSpeed * 2 * (sin(radDirection) + cos(radDirection)) / 5), 1);
      setMotor(flipped * (movementSpeed * 2 * (sin(radDirection) - cos(radDirection)) / 5), 1);
      break;

    case STATE_MELTY:
      //both red and green LED solid while not spinning to show melty mode
      runMelty();
      break;

    case STATE_MAX_SPIN:
      //red LED constant on to show max spin mode
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      //send motor commands
      setMotor(flipped * 1000, 1);
      setMotor(flipped * 1000, 2);
      break;

    default:
      break;
  }
}

void stateChange() {
  //check switches to determine program state to transition to
  prevState = state;
  //safe state if transmitter connection is lost
  if (signalLost()) state = STATE_STARTUP;
  //idle if both switches off
  else if (recFlap <= 500 && recGear <= 500) state = STATE_IDLE;
  //calibrate if gear switch is on but flap switch is off
  else if (recFlap <= 500 && recGear > 500) state = STATE_CALIBRATE;
  //drive if flap switch is on and throttle is down
  else if (recFlap > 500 && recGear <= 500 && recThrot < 100) state = STATE_DRIVE;
  //melty if flap switch is on and throttle is up
  else if (recFlap > 500 && recGear <= 500 && recThrot >= 100) state = STATE_MELTY;
  //max spin if both switches on
  else if (recFlap > 500 && recGear > 500) state = STATE_MAX_SPIN;

  //check state transitions
  //startup can only exit to idle when both switches off
  if (prevState == STATE_STARTUP && state != STATE_IDLE) state = STATE_STARTUP;
  //calibrate state can only exit to idle, goes through exit state to save calibration data
  else if (prevState == STATE_CALIBRATE && state != STATE_CALIBRATE) {
    if (state == STATE_IDLE) state = STATE_CALIBRATE_EXIT;
    else state = STATE_CALIBRATE;
  }
  //Max spin mode cannot be entered at low throttle to prevent accidents
  else if (prevState == STATE_MELTY && state == STATE_MAX_SPIN && recThrot < 250) state = STATE_MELTY;
  //max spin mode can only exit to melty mode or idle
  else if (prevState == STATE_MAX_SPIN && state != STATE_MAX_SPIN) {
    if (state == STATE_MELTY || state == STATE_IDLE);
    else state = STATE_MAX_SPIN;
  }
}
