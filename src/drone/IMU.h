#ifndef __IMU_H__
#define __IMU_H__

//Different types of sensors/libraries defined here
#define NO_IMU -1
#define IMU_MPU6050 0
#define IMU_MPU6050_DMP 1 //200 Hz max

//Select which IMU setup to use
#define IMU_TYPE IMU_MPU6050


//Import libraries
#if IMU_TYPE == IMU_MPU6050
  #include <Wire.h>
  #include <MPU6050_kriswiner.h> //https://github.com/kriswiner/MPU6050
  #include <SimpleKalmanFilter.h>
#elif IMU_TYPE == IMU_MPU6050_DMP
  #include <Wire.h>
  #include <MPU6050_6Axis_MotionApps612.h>
#elif IMU_TYPE == NO_IMU
  #include <MPU6050_kriswiner.h>
  #include <SimpleKalmanFilter.h>
#endif

extern const int lightPin;
extern float loopTime();


/* Settings */
const float angleOffset[3] = {8, 1.1, 0}; //IMU angle offset {roll, pitch and yaw}
/* Settings */


class IMU {
  public:
    int init();
    void updateAngle();
    
    float currentAngle[3] = {0, 0, 0}; //Current angle of roll, pitch and yaw (in degrees)
    float rRate[3] = {0, 0, 0};        //Rotation rate (degrees per millisecond) of roll, pitch and yaw

  private:
    //Define variables specific to setups
    #if IMU_TYPE == IMU_MPU6050
      MPU6050lib mpu;
      const int intPin = 41;
      float aRes; //Resolution of the accelerometer
      float gRes; //Resolution of the accelerometer
      int16_t accelData[3];      //Accelerometer sensor output
      int16_t gyroData[3];       //Gyroscope sensor output
    #endif
    #if IMU_TYPE == IMU_MPU6050 or IMU_TYPE == NO_IMU
      float accelVal[3];         //Accelerometer value in g's
      float gyroVal[3];          //Gyroscope value in degrees per seconds
      SimpleKalmanFilter rollKalman{.5, 1, 0.5};
      SimpleKalmanFilter pitchKalman{.5, 1, 0.5};
      float q[4] = {1, 0, 0, 0}; //Quaternion container
      //Parameters for MadgwickQuaternionUpdate calculations
      const float beta = sqrt(.05) * PI * (5.0 / 180.0);
      const float zeta = sqrt(.75) * PI * (2.0 / 180.0);

      void MadgwickQuaternionUpdate(float *accel, float *gyro);
      void eulerToQuat(float roll, float pitch, float yaw);
    #elif IMU_TYPE == IMU_MPU6050_DMP
      //MPU control/status vars
      MPU6050 mpu;
      uint8_t devStatus;      //Return status after each device operation (0 = success, !0 = error)
      uint8_t fifoBuffer[64]; //FIFO storage buffer
      //MPU orientation/motion vars
      Quaternion q;                     //Quaternion container
      VectorFloat gravity;              //Gravity vector
      float ypr[3];                     //Array containing yaw, pitch and roll
      const float MPUmult = 180 / M_PI; //Constant used to convert from MPU6050 data to degrees
      int16_t gyroData[3];              //Gyroscope sensor output
    #endif
};
#endif
