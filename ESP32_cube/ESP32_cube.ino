#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube-blue"); //Bluetooth device name
  EEPROM.begin(EEPROM_SIZE);
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    else calibrated = false;
    
  delay(2000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
}

void loop() {

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    Tuning();  // derinimui
    angle_calc();
    if (balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 5) vertical = false;
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 5) vertical = false;
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 5) vertical = false;
    }
    
    if (abs(angleX) < 8 || abs(angleY) < 8) {  // fast restore angle
      Gyro_amount = 0.996; 
    } else 
      Gyro_amount = 0.1;

    if (vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroX = GyX / 131.0; // Convert to deg/s

      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_Y, -255, 255);
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      
      if (balancing_point == 1) {
        XY_to_threeWay(-pwm_X, -pwm_Y);
      } else if (balancing_point == 2) {
        Motor1_control(pwm_Y);
      } else if (balancing_point == 3) {
        Motor2_control(-pwm_Y);
      } else if (balancing_point == 4) {
        Motor3_control(pwm_X);
      }
    } else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / 207); 
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing points...");
    }
    previousT_2 = currentT;
  }
}

