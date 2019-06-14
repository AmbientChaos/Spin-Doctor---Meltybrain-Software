#include <SparkFun_LIS331.h>
#include <Wire.h>

const byte arraySize = 2; //2 for triangular integration
unsigned long accelTime[arraySize] = {0}; //time between accelerometer measurements
uint16_t degreePeriod[arraySize] = {0}; //period in microseconds per degree
uint16_t accelAngle[arraySize] = {0}; //current angle calculated from accelerometer
LIS331 accel;

void accelISR(){
  static int16_t xAccel; //vertical acceleration - detect flips - using interrupt pin instead
  static int16_t yAccel; //tangential acceleration - not used
  for(int i=1; i>arraySize; i++) { //shift old data down
      accelTime[i] = accelTime[i-1];
      degreePeriod[i] = degreePeriod[i-1];
    }
  noInterrupts();
  accelTime[0] = micros(); //add new data
  accel.readAxes(xAccel, yAccel, zAccel, false, false, true); //get z accel data
  if(digitalRead(ACCEL_FLIP) == LOW) flipped = 1; //record orientation
  else if(digitalRead(ACCEL_FLIP) == HIGH) flipped = -1;
  interrupts();
  //calculate rotation speed(period) from accelerometer
  degreePeriod[0] = (accelCalA * exp((double)(zAccel / accelCalB)) + accelCalC); 
  accelNew = true;
}

void accelSetup(){
  Wire.begin();
  
  //set the registers manually because I don't like the LIS331 library not exposing all of the options
  Wire.beginTransmission(0x19);
  Wire.write(CTRL_REG1);
  Wire.write(0x3D); //normal mode, 1000Hz, X and Z axes enabled
  Wire.write(CTRL_REG4);
  Wire.write(0xB0); //Block Data Update on, full scale to 400g
  Wire.endTransmission();
  
  //set the rest of the options with the LIS331 library
  accel.setI2CAddr(0x19);
  accel.begin(LIS331::USE_I2C);
  accel.intSrcConfig(LIS331::DRDY, 1); //interrupt pin 1 out to show new data
  accel.latchInterrupt(false, 1);
  pinMode(ACCEL_NEW,INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_NEW), accelISR, RISING);
  accel.intSrcConfig(LIS331::INT_SRC, 2); //interrupt pin 2 out to show flipped
  accel.setIntDuration(500, 2); //0.5 sec before triggering at 1000Hz
  accel.setIntThreshold(0x40, 2); //threshold set for a reading of below 0g, 64 on a scale of 0 to 127
  accel.latchInterrupt(false, 2);
  accel.enableInterrupt(LIS331::X_AXIS, LIS331::TRIG_ON_LOW, 2, true);
  pinMode(ACCEL_FLIP, INPUT);
}

//I wanted an accelerometer read function that didn't read unused axes
void LIS331::readAxes(int16_t &x, int16_t &y, int16_t &z, bool getX, bool getY, bool getZ)
{
  uint8_t data[2]; // create a buffer for our incoming data
  if(getX) {
    LIS331_read(OUT_X_L, &data[0], 1);
    LIS331_read(OUT_X_H, &data[1], 1);
    x = data[0] | data[1] << 8;
    x = x >> 4;
  }
  if(getY) {
    LIS331_read(OUT_Y_L, &data[0], 1);
    LIS331_read(OUT_Y_H, &data[1], 1);
    y = data[0] | data[1] << 8;
    y = y >> 4;
  }
  if(getZ) {
    LIS331_read(OUT_Z_L, &data[0], 1);
    LIS331_read(OUT_Z_H, &data[1], 1);
    z = data[0] | data[1] << 8;
    z = z >> 4;
  }
}
