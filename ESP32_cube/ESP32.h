#define BUZZER      27
#define VBAT        34

#define BRAKE       26

#define DIR1        4
#define PWM1        32
#define PWM1_CH     1

#define DIR2        15
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define PWM3        18
#define PWM3_CH     2

#define TIMER_BIT  8
#define BASE_FREQ  20000

#define MPU6050 0x68              // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG  0x1B         // Gyro configuration address

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

float Gyro_amount = 0.1;  

bool vertical = false;
int balancing_point = 0;

float K1 = 160;
float K2 = 10.50; 
float K3 = 0.03;
int loop_time = 10;

float offsetX = -0.99, offsetY = -3.43;
float offsetX2 = -31.24, offsetY2 = -19.05;
float offsetX3 = 30.40, offsetY3 = -19.21;
float offsetX4 = 0.17, offsetY4 = 35.9;
float alpha = 0.74;  

int16_t  AcX, AcY, AcZ, GyX, GyY, GyZ, gyroX, gyroY, gyroZ, gyroYfilt, gyroZfilt;

int16_t  AcX_offset = -940;  
int16_t  AcY_offset = -200;      
int16_t  AcZ_offset = 1800;
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;      
int32_t motor_speed_X;  
int32_t motor_speed_Y;   

long currentT, previousT_1, previousT_2 = 0; 

