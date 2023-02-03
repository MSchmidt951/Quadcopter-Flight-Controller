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

//Import files
#include "Logger.h"

extern const int lightPin;
extern float loopTime();


/**
 * @class IMU
 * @brief Handes the inertial measurement unit(s) and other sensors of the device
 */
class IMU {
  public:
    /** Initialise the sensors.
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @returns Status of IMU. 0 for no error
     */
    int init(Logger &logger);
    /** Reads the sensors and updates the current angle and other measurements */
    void updateAngle();
    
    ///Current angle of roll, pitch and yaw (in degrees)
    float currentAngle[3] = {0, 0, 0};
    ///Rotation rate (degrees per millisecond) of roll, pitch and yaw
    float rRate[3] = {0, 0, 0};

    /* Settings */
    ///IMU angle offset {roll, pitch and yaw}. Can be set via SD card
    float angleOffset[3] = {8, 1.1, 0};
    /* Settings */

  private:
    //Define variables specific to setups
    #if IMU_TYPE == IMU_MPU6050 or IMU_TYPE == NO_IMU
      ///Accelerometer value in Gs
      float accelVal[3];
      ///Gyroscope value in degrees per seconds
      float gyroVal[3];
      ///Kalman filter for the roll axis
      SimpleKalmanFilter rollKalman{.5, 1, 0.5};
      ///Kalman filter for the pitch axis
      SimpleKalmanFilter pitchKalman{.5, 1, 0.5};
      ///Quaternion container
      float q[4] = {1, 0, 0, 0};
      ///beta parameter for MadgwickQuaternionUpdate calculations
      const float beta = sqrt(.05) * PI * (5.0 / 180.0);
      ///beta parameter for MadgwickQuaternionUpdate calculations
      const float zeta = sqrt(.75) * PI * (2.0 / 180.0);

      /** Calculates the current angle (in quaternions) using the accelerometer and gyroscope
       *  
       *  @param[in] accel accelerometer data (Gs)
       *  @param[in] gyro gyroscope data (degrees/second)
       */
      void MadgwickQuaternionUpdate(float *accel, float *gyro);
      /** Converts roll, pitch and yaw to quaternions */
      void eulerToQuat(float roll, float pitch, float yaw);

      #if IMU_TYPE == IMU_MPU6050
        ///MPU6050 object
        MPU6050lib mpu;
        ///Interrupt pin
        const int intPin = 39;
        ///Resolution of the accelerometer
        float aRes;
        ///Resolution of the accelerometer
        float gRes;
        ///Accelerometer sensor output
        int16_t accelData[3];
        ///Gyroscope sensor output
        int16_t gyroData[3];
      #endif
    #elif IMU_TYPE == IMU_MPU6050_DMP
      //MPU control/status vars
      ///MPU6050 object
      MPU6050 mpu;
      ///Return status after each device operation (0 = success, !0 = error)
      uint8_t devStatus;
      ///FIFO storage buffer
      uint8_t fifoBuffer[64];
      //MPU orientation/motion vars
      ///Quaternion container
      Quaternion q;
      ///Gravity vector
      VectorFloat gravity;
      ///Array containing yaw, pitch and roll
      float ypr[3];
      ///Constant used to convert from MPU6050 data to degrees
      const float MPUmult = 180 / M_PI;
      ///Gyroscope sensor output
      int16_t gyroData[3];
    #endif
};
#endif
