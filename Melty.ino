byte throtCurrent = 0;
uint16_t currentAngle = 0;
unsigned long lastMotorSend = micros();

void runMelty() {
  getSticks();

  //run melty if throttle is high enough
  if (throtCurrent >= throtMin) {
    getAngle();
    meltLights();
    meltMove();
  }
  //throttle too low for translation, shut off motor
  else { 
    setMotor(0);
    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, HIGH);
  }
}

void getSticks() {
  //calculate melty inputs from receiver
  if (sticksNew) { 
    sticksNew = false;
    movementSpeed = max(100, (int)hypot(recAiler - 50, recElev - 50));
    movementDirection = (atan2(recAiler * flipped, recElev) * 4068) / 71; //deg = rad * 4068 / 71
    throtCurrent = recThrot * throtMax / 100;
    trimAngle();
  }
}

void getAngle() { 
  //triangular integration calculations borrowed from Halo
  //calculate angle from new accel data
  if (accelNew) { 
    accelNew = false;
    //shift old data down
    for (int i = 1; i < arraySize; i++) { 
      accelAngle[i] = accelAngle[i - 1];
    }
    //give up if accel too low
    if (zAccel < 400) { 
      accelAngle[0] = 0;
      currentAngle = accelAngle[0];
    }
    //triangular integration from new data
    else {
      uint16_t deltaT = accelTime[0] - accelTime[1];
      accelAngle[0] = (accelAngle[1] + (deltaT / degreePeriod[0] + deltaT / degreePeriod[1]) / 2) % 360;
      currentAngle = accelAngle[0];
    }
  }
  //predict the angle between accel readings by extrapolating from old data
  else { 
    if (zAccel >= 400) {
      uint16_t newTime = micros();
      uint16_t periodPredicted = degreePeriod[1] + (newTime - accelTime[1]) 
          * (degreePeriod[0] - degreePeriod[1]) / (accelTime[0] - accelTime[1]);
      //predict the current robot heading by triangular integration up to the extrapolated point
      uint16_t deltaT = newTime - accelTime[0];
      currentAngle = (accelAngle[0] + (deltaT / periodPredicted + deltaT / degreePeriod[0]) / 2) % 360;
    }
  }
}

void trimAngle() {
  //use rudder to rotate heading direction
  int16_t angleTrim;
  angleTrim = (recRudd - 50) / 50;
  for (int i = 0; i < arraySize; i++) {
    accelAngle[i] = (accelAngle[i] + angleTrim) % 360;
  }
}

void meltLights() {
  //turn on green light if it's position is "forward"
  if ((currentAngle + lightOffset) % 360 <= 10 
      || (currentAngle + lightOffset) % 360 >= 350) {
    digitalWrite(GREEN, HIGH);
  }
  else {
    digitalWrite(GREEN, LOW);
  }
  //turn on red ligth if its position is in the stick direction
  if ((currentAngle + lightOffset) % 360 <= (movementDirection + 10) % 360 
      && (currentAngle + lightOffset) % 360 >= (movementDirection - 10) % 360) {
    digitalWrite(RED, HIGH);
  }
  else {
    digitalWrite(RED, LOW);
  }
}

void meltMove() {
  static int16_t diff;
  //translate if stick is moved enough
  if (movementSpeed > 15) {
    diff = 180 - abs(abs(movementDirection - currentAngle) - 180);
    //speed up motor if moving toward movement direction
    if (diff < 90) {
      setMotor(flipped * (throtCurrent + 20));
    }
    //slow down motor if moving away from movement direction
    else {
      setMotor(flipped * (max(throtCurrent - 20, throtMin)));
    }
  }
  //spin in place if no stick movement
  else {
    setMotor(flipped * throtCurrent);
  }
}

void setMotor(uint16_t throttle) {
  //send DShot command based on 0-100% throttle input
  //limit max update rate to 4kHz
  if (lastMotorSend - micros() >= 250) { 
    if (throttle == 0) dshotThrottle(0);
    else dshotThrottle(throttle * 999 / 100 + 1047);
    lastMotorSend = micros();
  }
}
