uint16_t currentAngle = 0;
unsigned long lastMotorSend1 = micros();
unsigned long lastMotorSend2 = micros();

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
    setMotor(0, 1);
    setMotor(0, 2);
    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, HIGH);
  }
}

void getSticks() {
  //calculate melty inputs from receiver
  if (sticksNew) { 
    sticksNew = false;
    movementSpeed = min(500, (int)hypot(recAiler - 500, recElev - 500));
    movementDirection = (atan2((recAiler - 500) * flipped, recElev - 500) * 4068) / 71; //deg = rad * 4068 / 71
    throtCurrent = recThrot * throtMax / 1000;
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
    if (zAccel < 100) { 
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
    if (zAccel >= 100) {
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
  angleTrim = (recRudd - 500) / 200;
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
  if (movementSpeed > 50) {
    if ((currentAngle + lightOffset) % 360 <= (movementDirection + 10) % 360 
        && (currentAngle + lightOffset) % 360 >= (movementDirection - 10) % 360) {
      digitalWrite(RED, HIGH);
    }
    else {
      digitalWrite(RED, LOW);
    }
  }
}

void meltMove() {
  static int16_t diff;
  //translate if stick is moved enough
  if (movementSpeed > 50) {
    diff = 180 - abs(abs(movementDirection - currentAngle) - 180);
    //speed up motor if moving toward movement direction
    if (diff < 90) {
      setMotor(flipped * (throtCurrent + 200), 1);
      setMotor(flipped * (throtCurrent - 200), 2);
    }
    //slow down motor if moving away from movement direction
    else {
      setMotor(flipped * (throtCurrent - 200), 1);
      setMotor(flipped * (throtCurrent + 200), 2);
    }
  }
  //spin in place if no stick movement
  else {
    setMotor(flipped * throtCurrent, 1);
    setMotor(flipped * throtCurrent, 2);
  }
}

void setMotor(int16_t value, uint8_t motor = 1) {
  //send DShot command based on -1000 to 1000 throttle input
  //limit max update rate to 4kHz, min 25us between motors
  if (((motor == 1) && (micros() - lastMotorSend1 >= 250) &&(micros() - lastMotorSend2 >= 25)) 
    ||((motor == 2) && (micros() - lastMotorSend2 >= 250) &&(micros() - lastMotorSend1 >= 25))) { 
    if(value == 0) dshotOut(value, motor);
    else {
      if(value > 0) dshotOut(value * 1 + 1047, motor);
      else if(value < 0) dshotOut(value * -1 + 47, motor);
    }
    if(motor == 1) lastMotorSend1 = micros();
    else if(motor == 2) lastMotorSend2 = micros();
  }
}
