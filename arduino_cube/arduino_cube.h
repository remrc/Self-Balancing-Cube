#define PWM_3         3
#define DIR_3         4

#define PWM_1         10
#define DIR_1         2

#define PWM_2         9
#define DIR_2         7

#define BRAKE         8
#define BUZZER        12
#define VBAT          A7

#define MPU6050 0x68          // Device address
#define ACCEL_CONFIG 0x1C     // Accelerometer configuration address
#define GYRO_CONFIG 0x1B      // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

float Gyro_amount = 0.996;  

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

int balancing_point = 0;

float pGain = 150;
float iGain = 14.00; 
float sGain = 0.035;
int loop_time = 10;

struct OffsetsObj {
  int ID1;
  float X1;
  float Y1;
  int ID2;
  float X2;
  float Y2;
  int ID3;
  float X3;
  float Y3;
  int ID4;
  float X4;
  float Y4;
};

OffsetsObj offsets;

float alpha = 0.6;

int16_t  AcX, AcY, AcZ, GyY, GyZ, gyroX, gyroY, gyroZ, gyroYfilt, gyroZfilt;

int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;      
int32_t motor_speed_pwmX; 
int32_t motor_speed_pwmY;   

int bat_divider = 57;

long currentT, previousT_1, previousT_2 = 0;

