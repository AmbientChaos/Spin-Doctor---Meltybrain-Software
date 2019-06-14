#include <EEPROM.h>
#include <LinearRegression.h>

//calibration states
#define STATE_CAL_SELECT 0
#define STATE_CAL_THROT 1
#define STATE_CAL_ACCEL 2
#define STATE_CAL_LIGHT 3

//calibration variables
byte calState = STATE_CAL_SELECT;
byte selectedState = STATE_CAL_THROT;
uint16_t yMin = 0 - 1;
double calTempA = 0;
double calTempB = 0;
double calTempR = 0;
double accelCalR = 0;
unsigned long calibrateTimer = millis();
unsigned long debounceTimer = millis();
unsigned long accelTimer = micros();

//3 stage calibration
  //start in menu to select which calibration to run
    //red light on
    //blink green light in 2 second cycle until selection is made
    //number of blinks in cycle corresponds to which calibration to run
    //rudder right/left to select next/prev calibration
    //elevator high to start calibration
  //run selected calibration
    //1 - throttle calibration routine
      //5 second red blink warning before starting
        //elevator low to cancel
      //slow blink green light until complete
      //start at 5% throttle
      //use rudder right to increase throttle until spin is achieved
      //record min throttle
      //slowly increases throttle until accelerometer saturates
      //record max throttle
    //2 - accel calibration routine
      //5 second red blink warning before starting
        //elevator low to cancel
      //20 steps from min throttle to max throttle
        //at each step flash light at an x microsecond period
        //use rudder left/right to decrease/increase period until light appears stationary
        //use elevator low to move to next step
        //record aaccel reading and period at end of each step
      //linearized exopnential regression -> y = a * exp(b * x) + c
        //run regression with increasing c until R squared decreases
        //record calibration parameters
    //3 -  light offset
      //5 second red blink warning before starting
        //elevator low to cancel
      //accelerate to melty throttle
      //elevator moves forward/back to show movement direction
      //green light indicates position
      //rudder adjusts light offset
      //aileron adjusts heading
      //throttle high to save calibration

void runCalibrate() {  
  //check calibrate state to determine what to do
  if(prevState != STATE_CALIBRATE) calState = STATE_CAL_SELECT;
  switch(calState) {
    case STATE_CAL_SELECT:
      calibrateSelect();
    break;

    case STATE_CAL_THROT:
      calibrateThrot();
    break;

    case STATE_CAL_ACCEL:
      calibrateAccel();
    break;

    case STATE_CAL_LIGHT:
      calibrateLight();
    break;
  }
}

void calibrateSelect(){
  calibrateRunning = false;
  digitalWrite(RED, HIGH);
  calSelectBlink();
  //rudder right to go to next cal state
  if(recRudd > 80){
    if(millis() - debounceTimer > 750) {
      debounceTimer = millis();
      selectedState++;
      if(selectedState > 3) selectedState =1;
    }
  }
  //rudder left to go to prev cal state
  else if(recRudd < 20){
    if(millis() - debounceTimer > 750) {
      debounceTimer = millis();
      selectedState--;
      if(selectedState < 1) selectedState =3;
    }
  }
  //elevator up to select current cal state
  else if(recElev > 80) {
    if (millis() - debounceTimer > 750) {
      debounceTimer = millis();
      calState = selectedState;
      digitalWrite(GREEN, LOW);
      calibrateTimer = millis();
    }
  }
}
  
void calibrateThrot(){
  static uint16_t calThrot = 5;
  static bool minSet = false;
  static uint16_t throt;
  //1 - throttle calibration routine
    //5 second red blink warning before starting
      //elevator low to cancel
    //slow blink green light until complete
    //start at 5% throttle
    //use rudder right to increase throttle until spin is achieved
    //elevator low to record min throttle
    //slowly increases throttle until accelerometer saturates
    //record max throttle

  //5 second red blinking warning before starting movement
  calWarningBlink(); 
  if(!calibrateRunning) return;
  //blink green light while running
  if(millis() - blinkTimer > 1000) {
    blinkTimer = millis();
    digitalWrite(GREEN, !digitalRead(GREEN));
  }
  //manual setting of minimum throttle
  if(!minSet){ 
    //rudder right to increase minimum throttle
    if(recRudd > 80){
      if(millis() - debounceTimer > 750) {
        debounceTimer = millis();
        throtMin++;
      }
    }
    //rudder left to decrease minimum throttle
    else if(recRudd < 20){
      if(millis() - debounceTimer > 750) {
        debounceTimer = millis();
        throtMin--;
      }
    }
    //elevator down to save minimum throttle
    else if(recElev < 20){
      calibrateTimer = millis();
      minSet = true;
    }
    //set throttle at current minimum throttle setting
    throt = throtMin;
  }
  //Automatic setting of max throttle
  else {
    //increase throttle by 1% in 1 second intervals
    if(millis() - calibrateTimer > 1000) {
      //record max throttle when accelerometer reaches maximum
      if(zAccel >= 2045){
        throtMax = throt - 1;
        calState = STATE_CAL_SELECT;
        minSet = false;
      }
      calibrateTimer = millis();
      throt++;
    }
  }
  //run motor at current throttle setting
  setMotor(throt);
}

void calibrateAccel(){
  static int16_t calAccel[20] = {0};
  //initialize array with estimates of periods at testing throttles
  static int16_t throtStep = ((throtMax - throtMin) / 19);
  static uint32_t calPeriod[20] = {110000 / ((0 * throtStep + throtMin) * 27),
                                   110000 / ((1 * throtStep + throtMin) * 27),
                                   110000 / ((2 * throtStep + throtMin) * 27),
                                   110000 / ((3 * throtStep + throtMin) * 27),
                                   110000 / ((4 * throtStep + throtMin) * 27),
                                   110000 / ((5 * throtStep + throtMin) * 27),
                                   110000 / ((6 * throtStep + throtMin) * 27),
                                   110000 / ((7 * throtStep + throtMin) * 27),
                                   110000 / ((8 * throtStep + throtMin) * 27),
                                   110000 / ((9 * throtStep + throtMin) * 27),
                                   110000 / ((10 * throtStep + throtMin) * 27),
                                   110000 / ((11 * throtStep + throtMin) * 27),
                                   110000 / ((12 * throtStep + throtMin) * 27),
                                   110000 / ((13 * throtStep + throtMin) * 27),
                                   110000 / ((14 * throtStep + throtMin) * 27),
                                   110000 / ((15 * throtStep + throtMin) * 27),
                                   110000 / ((16 * throtStep + throtMin) * 27),
                                   110000 / ((17 * throtStep + throtMin) * 27),
                                   110000 / ((18 * throtStep + throtMin) * 27),
                                   110000 / ((19 * throtStep + throtMin) * 27)};
  static bool calFinished = false;
  static uint16_t throt;
  //2 - accel calibration routine
    //5 second red blink warning before starting
      //elevator low to cancel
    //20 steps from min throttle to max throttle
      //at each step flash light at an x microsecond period
      //use rudder left/right to decrease/increase period until light appears stationary
      //use elevator low to move to next step
      //record average of 10 accel readings and period at end of each step
    //linearized exopnential regression -> y = a * exp(b * x) + c
      //run regression with increasing c until R squared decreases
    //record calibration parameters
    
  //5 second red blinking warning before starting movement
  calWarningBlink(); 
  if(!calibrateRunning) return;
  if(!calFinished) { //collect accelerometer/period data for regression
    static int i = 0;
    throt = i * throtStep + throtMin;

    //blink light based on period
    if((micros() - accelTimer) > (calPeriod[i] * 360)){
      digitalWrite(GREEN, HIGH);
    }
    else if((micros() - accelTimer) > ((calPeriod[i] + 10) * 360)){
      digitalWrite(GREEN, LOW);
      accelTimer = micros();
    }

    //check for inputs to set period
    //rudder right to decrease blink speed (increase period)
    //should make light move more in direction of rotation
    if(recRudd > 80){
      if(millis() - debounceTimer > 250) {
        debounceTimer = millis();
        calPeriod[i]++;
      }
    }
    //rudder right to increase blink speed (decrease period)
    //should make light move more against direction of rotation
    else if(recRudd < 20){
      if(millis() - debounceTimer > 250) {
        debounceTimer = millis();
        calPeriod[i]--;
      }
    }
    //elevator down to move to the next measurement speed
    //moves to calculation after last measurement
    else if(recElev < 20){
      if(millis() - debounceTimer > 750) {
        debounceTimer = millis();
        calAccel[i] = zAccel;
        i++;
        if(i >= 20){
          calFinished = true;
          i = 0;
          throt = 0;
        }
      }
    }
  }
  //finished collecting accelerometer data, calculate constants
  else { 
    //brute force iteration of regressions to find best value for C
    for(int i = 1; i <= 1000; i++) { 
      //lease squares regression on collected period and accel data to derive relationship
      //solves for variables of (linearized) equation Y = A * exp(X / B) + C
      regression(calAccel, calPeriod, i);
      //check to see if the current iteration is a better fit than the previous
      if(calTempR < accelCalR) break;
      if(calTempR > accelCalR) {
        accelCalA = exp(calTempA);
        accelCalB = 1/calTempB;
        accelCalC = yMin - i;
        accelCalR = calTempR;
      }
    }
    calState = STATE_CAL_SELECT;
    calFinished = false;
  }
  setMotor(throt);
}

void calibrateLight(){
  static bool moving = false;
  static bool pause = true;
  static bool forward = true;
  //3 -  light offset
    //5 second red blink warning before starting
      //elevator low to cancel
    //accelerate to melty throttle
    //green light indicates position
    //automatically move forward/back for 1/2 sec
      //throttle low to activate/deactivate movement
    //rudder adjusts light offset
    //aileron adjusts heading
    //throttle high to save calibration
    
  //5 second red blinking warning before starting movement
  calWarningBlink(); 
  if(!calibrateRunning) return;

  //elevator low to start/stop movement
  if(recElev < 20) {
    if(millis() - debounceTimer > 750){
      debounceTimer = millis();
      moving = !moving;
    }
  }

  //rudder left to decrease offset
  else if(recRudd < 20) {
    if(millis() - debounceTimer > 200){
      debounceTimer = millis();
      lightOffset = (lightOffset - 1) % 360;
    }
  }

  //rudder right to increase offset
  else if(recRudd > 80) {
    if(millis() - debounceTimer > 200){
      debounceTimer = millis();
      lightOffset = (lightOffset + 1) % 360;
    }
  }

  //aileron left to rotate heading left
  else if(recAiler < 20) {
    if(millis() - debounceTimer > 10){
      debounceTimer = millis();
      for (int i = 0; i < arraySize; i++) {
        accelAngle[i] = (accelAngle[i] - 1) % 360;
      }
    }
  }

  //aileron right to rotate heading right
  else if(recAiler > 80) {
    if(millis() - debounceTimer > 10){
      debounceTimer = millis();
      for (int i = 0; i < arraySize; i++) {
        accelAngle[i] = (accelAngle[i] + 1) % 360;
      }
    }
  }

  //throttle high to save calibration and exit
  else if(recThrot > 80) {
    if(millis() - debounceTimer > 750){
      debounceTimer = millis();
      calState = STATE_CAL_SELECT;
      moving = false;
      setMotor(0);
      return;
    }
  }

  if(moving){
    //pause 1 sec
    if(pause){
      movementSpeed = 0;
      if(millis() - calibrateTimer > 1000) {
        calibrateTimer = millis();
        pause = false;
      }
    }
    else {
      movementSpeed = 100;
      //move forward 1 sec then pause
      if(forward){
        movementDirection = 0;
        if(millis() - calibrateTimer > 1000) {
          calibrateTimer = millis();
          pause = true;
          forward = false;
        }
      }
      //move backward 1 sec then pause
      else {
        movementDirection = 180;
        if(millis() - calibrateTimer > 1000) {
          calibrateTimer = millis();
          pause = true;
          forward = true;
        }
      }
    }
  }
  getAngle();
  meltLights();
  meltMove();
}

void calSelectBlink(){
  static byte blinkCount = 0;
  static bool blinked = false;

  //blink number of times for calibration selection, then 2 seconds off
  digitalWrite(RED, HIGH);
  if(digitalRead(GREEN) == LOW){
    if(blinkCount == 0){
      if(millis() - blinkTimer > 2000){
        blinkCount++;
      }
    }
    else if(blinkCount > 0){
      if(!blinked){  
        digitalWrite(GREEN, HIGH);
        blinkTimer = millis();
      }
      else if(blinked){
        if(millis() - blinkTimer > 250){
          blinkCount++;
          blinked = false;
        }
        if(blinkCount > selectedState) {
          blinkCount = 0;
        }
      }
    }
  }
  else if(digitalRead(GREEN) == HIGH){
    if(millis() - blinkTimer > 250){
      digitalWrite(GREEN, LOW);
      blinkTimer = millis();
      blinked = true;
    }
  }
}

void calWarningBlink(){
  if(!calibrateRunning){
    if(millis() - calibrateTimer > 5000){ //ready to start running calibration?
      calibrateTimer = millis();
      calibrateRunning = true;
      digitalWrite(RED, LOW);
    }
    //blink red status light to warn of calibration starting
    if(millis() - blinkTimer > 200) {
      blinkTimer = millis();
      digitalWrite(RED, !digitalRead(RED));
    }
    //elevator low to cancel calibration
    if(recElev < 20) {
      calState = STATE_CAL_SELECT;
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
    }
  }
}

void regression(uint16_t x[], uint32_t y[], uint16_t shift){
  int16_t xMin = 32767;
  int16_t xMax = -32768;
  yMin = 0 - 1;

  //find x range for regression algorithm
  for(int i = 0; i < 20; i++) {
    if(x[i] < xMin) xMin = x[i];
    if(x[i] > xMax) xMax = x[i];
    if(y[i] < yMin) yMin = y[i];
  }
  //least squares linear regression of x(accel) and y(period) data
  LinearRegression reg = LinearRegression(xMin,xMax);
  for(int i = 0; i < 20; i++) {
    reg.learn(x[i], y[i] - yMin - max(shift,1));
  }
  double values[3];
  reg.getValues(values);
  calTempB = values[0];
  calTempA = values[1];
  calTempR = reg.correlation();
  calTempR = calTempR * calTempR;
}

void readCalibration(){
  //check that data is valid and read in saved calibration data from EEPROM
  if(EEPROM.read(0) == 42) {
    throtMin = EEPROM.read(1);
    throtMax = EEPROM.read(2);
    lightOffset = EEPROM.read(3) | EEPROM.read(4) << 8;
    accelCalA = EEPROM.read(5) | EEPROM.read(6) << 8;
    accelCalB = EEPROM.read(7) | EEPROM.read(8) << 8;
    accelCalC = EEPROM.read(9) | EEPROM.read(10) << 8;
  }
}

void writeCalibration(){
  //write validation byte and calibration data to EEPROM
  EEPROM.update(0, 42);
  EEPROM.update(1, throtMin);
  EEPROM.update(2, throtMax);
  EEPROM.update(3, highByte(lightOffset));
  EEPROM.update(4, lowByte(lightOffset));
  EEPROM.update(5, highByte(accelCalA));
  EEPROM.update(6, lowByte(accelCalA));
  EEPROM.update(7, highByte(accelCalB));
  EEPROM.update(8, lowByte(accelCalB));
  EEPROM.update(9, highByte(accelCalC));
  EEPROM.update(10, lowByte(accelCalC));
}
