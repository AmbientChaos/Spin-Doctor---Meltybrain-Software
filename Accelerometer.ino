//Teensy 3.X specific I2C Wire library
#include <i2c_t3.h>

//default address for the accelerometer
#define accelAddress 0x19

//accelerometer register addresses
#define CTRL_REG1        0x20
#define CTRL_REG2        0x21
#define CTRL_REG3        0x22
#define CTRL_REG4        0x23
#define CTRL_REG5        0x24
#define HP_FILTER_RESET  0x25
#define REFERENCE        0x26
#define STATUS_REG       0x27
#define OUT_X_L          0x28
#define OUT_X_H          0x29
#define OUT_Y_L          0x2A
#define OUT_Y_H          0x2B
#define OUT_Z_L          0x2C
#define OUT_Z_H          0x2D
#define INT1_CFG         0x30
#define INT1_SRC         0x31
#define INT1_THS         0x32
#define INT1_DUR         0x33
#define INT2_CFG         0x34
#define INT2_SRC         0x35
#define INT2_THS         0x36
#define INT2_DUR         0x37

//Accelerometer output offset correction values
#define X_OFFSET         19
#define Y_OFFSET         15
#define Z_OFFSET         -21



unsigned long accelNewTime = 0;

void readAccel(byte regAddress, byte* data, uint8_t bytes){
  //read bytes from accelerometer register
  Wire.beginTransmission(accelAddress);
  Wire.write(regAddress | 0x80);//
  Wire.sendTransmission();
  Wire.sendRequest(accelAddress, bytes);
  while(Wire.available()) *(data++) = Wire.readByte();
}

void writeAccel(byte regAddress, byte data){
  //write byte to accelerometer register
  Wire.beginTransmission(accelAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void getAccel(int16_t& xAccel, int16_t& yAccel, int16_t& zAccel){
  //read axis output values from accelerometer
  
  uint8_t data[6];
  static uint32_t xSum = 0;
  static uint8_t xCount = 0;
  
  //input buffers for smoothing filters
  static int16_t xHist[100], zHist[5];

  //read bytes from accelerometer
  readAccel(OUT_X_L, data, 6);

  //100 point square averaging window on X axis to smooth orientation data
  xSum -= xHist[xCount];
  xHist[xCount] = ((int16_t)(data[4] | data[5] << 8) >> 4) + X_OFFSET;
  xSum += xHist[xCount];
  xAccel = xSum/100; 
  xCount++;
  if(xCount >= 100) xCount = 0;

  //5 point triangle averaging window to smooth Z axis data
  for(int i = 4; i > 0; i--) zHist[i] = zHist[i-1];
  zHist[0] = ((int16_t)(data[4] | data[5] << 8) >> 4) + Z_OFFSET;
  zAccel = (zHist[0] + 2 * zHist[1] + 3 * zHist[2] + 2 * zHist[3] + zHist[4])/9; 
  
  //unfiltered axis data
  /*xAccel = ((int16_t)(data[0] | data[1] << 8) >> 4) + X_OFFSET;
  yAccel = ((int16_t)(data[2] | data[3] << 8) >> 4) + Y_OFFSET;
  zAccel = ((int16_t)(data[4] | data[5] << 8) >> 4) + Z_OFFSET;*/
}

void accelISR(){
  //record time of new measurement and set flag to read data
  accelNewTime = micros();
  accelNew = true;
}

void accelSetup(){
  //I2C settings and write to accelerometer registers
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1800000, I2C_OP_MODE_IMM);
  writeAccel(CTRL_REG1, B00111101); //normal mode, 1000Hz, X and Z enabled
  writeAccel(CTRL_REG2, B00110000); //high pass filter enabled
  writeAccel(CTRL_REG3, B00000010); //Data ready output pin enabled
  writeAccel(CTRL_REG4, B10110000); //BDU enabled, 400G range
  pinMode(ACCEL_NEW,INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_NEW), accelISR, RISING);
}

void runAccel(){
  //collect new accelerometer data and determine rotation speed and orientation
  static int16_t xAccel, yAccel;
  accelNew = false;
  
  //shift old data down
  for(int i=arraySize - 1; i>0; i--) { //shift old data down
      accelTime[i] = accelTime[i-1];
      degreePeriod[i] = degreePeriod[i-1];
  }
  
  //get new data
  getAccel(xAccel, yAccel, zAccel);
  accelTime[0] = accelNewTime;

  //calculate rotation speed(period) from accelerometer
  degreePeriod[0] = (accelCalA * exp((double)(zAccel / accelCalB)) + accelCalC); 

  //determine current orientation
  if(xAccel >= 0) flipped = 1;
  else flipped = -1;
}
