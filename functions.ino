void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
  if (!vertical) {
       digitalWrite(BUZZER, HIGH);
       delay(70);
       digitalWrite(BUZZER, LOW);
    }  
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);

  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);

  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // add mpu6050 offset values
  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536;   
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536; 
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              // angle from acc. values * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  //SerialBT.print("AngleX_: "); SerialBT.print(robot_angleX); SerialBT.print(" AngleY_: "); SerialBT.println(robot_angleY); 
  angleX = robot_angleX - offsetX;
  angleY = robot_angleY - offsetY;
  //SerialBT.print("AngleX: "); SerialBT.print(angleX); SerialBT.print(" AngleY: "); SerialBT.println(angleY); 
  
  if (abs(angleX - offsetX2) < 2 && abs(angleY - offsetY2) < 0.6) {
    balancing_point = 2;
    beep();
    vertical = true;
  } else if (abs(angleX - offsetX3) < 2 && abs(angleY - offsetY3) < 0.6) {
    balancing_point = 3;
    beep();
    vertical = true;
  } else if (abs(angleX - offsetX4) < 0.6 && abs(angleY - offsetY4) < 2) {
    balancing_point = 4;
    beep();
    vertical = true;
  } else if (abs(angleX) < 0.4 && abs(angleY) < 0.4) {
    balancing_point = 1;
    beep();
    vertical = true;
  } 
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);  
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = -pwm_X;  
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR1, HIGH);
  }
  pwmSet(PWM1_CH, sp > 255 ? 255 : 255 - sp);
}

void Motor2_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR2, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR2, HIGH);
  }
  pwmSet(PWM2_CH, sp > 255 ? 255 : 255 - sp);
}

void Motor3_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR3, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR3, HIGH);
  }
  pwmSet(PWM3_CH, sp > 255 ? 255 : 255 - sp);
}

int Tuning() {
  if (!SerialBT.available())  return 0;
  //delay(1);
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  //SerialBT.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+')    K2 += 0.05;
      if (cmd == '-')    K2 -= 0.05;
      printValues();
      break;
    case 's':
      if (cmd == '+')    K3 += 0.005;
      if (cmd == '-')    K3 -= 0.005;
      printValues();
      break;  
   }
}

void printValues() {
  SerialBT.print("K1: "); SerialBT.print(K1);
  SerialBT.print(" K2: "); SerialBT.print(K2);
  SerialBT.print(" K3: "); SerialBT.println(K3,4);
}
