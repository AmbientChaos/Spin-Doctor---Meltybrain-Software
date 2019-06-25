#include <i2c_t3.h>

#define accelAddress 0x19

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

unsigned long accelNewTime = 0;
int16_t flipCount = 0;

void readAccel(byte regAddress, byte* data, uint8_t bytes){
  Wire.beginTransmission(accelAddress);
  Wire.write(regAddress | 0x80);//
  Wire.sendTransmission();
  Wire.sendRequest(accelAddress, bytes);
  while(Wire.available()) *(data++) = Wire.readByte();
}

void writeAccel(byte regAddress, byte data){
  Wire.beginTransmission(accelAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void getAccel(int16_t& xAccel, int16_t& yAccel, int16_t& zAccel){
  uint8_t data[6];
  readAccel(OUT_X_L, data, 6);
  xAccel = (data[0] | data[1] << 8) >> 4;
  yAccel = (data[2] | data[3] << 8) >> 4;
  zAccel = (data[4] | data[5] << 8) >> 4;
}

void accelISR(){
  accelNewTime = micros();
  accelNew = true;
}

void accelSetup(){
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1800000, I2C_OP_MODE_IMM);
  for (int i = 0x21; i < 0x25; i++) writeAccel(i,(uint8_t)0);
  for (int i = 0x30; i < 0x37; i++) writeAccel(i,(uint8_t)0);
  writeAccel(CTRL_REG1, B00111101); //normal mode, 1000Hz, X and Z enabled
  writeAccel(CTRL_REG2, B00111011); //high pass filter enabled
  writeAccel(CTRL_REG3, B00000010); //Data ready putput pin enabled
  writeAccel(CTRL_REG4, B10110000); //BDU enabled, 400G range
  //writeAccel(INT2_CFG, B00000001); //X low interrupt
  //writeAccel(INT2_THS, B00111111); //threshold at 63/127 range
  //writeAccel(INT2_DURATION, B01100100); //thresh duration 100 samples
  //pinMode(ACCEL_FLIP, INPUT);
  pinMode(ACCEL_NEW,INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_NEW), accelISR, RISING);
}

void runAccel(){
  static int16_t xAccel, yAccel;
  
  //shift old data down
  for(int i=1; i>arraySize; i++) { //shift old data down
      accelTime[i] = accelTime[i-1];
      degreePeriod[i] = degreePeriod[i-1];
  }
  
  //get new data
  getAccel(xAccel, yAccel, zAccel);
  accelTime[0] = accelNewTime;

  //record orientation, 100 sample delay to avoid spurious flips
  if(xAccel > 0 && flipped != 1){
    if(flipCount < 0) flipCount = 0;
    if(flipCount < 100) flipCount ++;
    if(flipCount >= 100) flipped = 1;
  }
  else if(xAccel < 0 && flipped != -1){
    if(flipCount > 0) flipCount = 0;
    if(flipCount > -100) flipCount --;
    if(flipCount <= -100) flipped = -1;
  }
  else flipCount = 0;

  //calculate rotation speed(period) from accelerometer
  degreePeriod[0] = (accelCalA * exp((double)(zAccel / accelCalB)) + accelCalC); 
  accelNew = false;
}
