#include <Wire.h>

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

extern double offsetX, offsetY, offsetZ;
extern double gyro_angle_x, gyro_angle_y, gyro_angle_z;
extern float angleX, angleY, angleZ;
extern float interval, preInterval;
extern float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
extern float gx, gy, gz, dpsX, dpsY, dpsZ;

//加速度、ジャイロから角度を計算
void calcRotation();

//I2c書き込み
void writeMPU6050(byte reg, byte data);

//i2C読み込み
byte readMPU6050(byte reg);

void mpu6050_setup() ;
void mpu6050_loop() ;

