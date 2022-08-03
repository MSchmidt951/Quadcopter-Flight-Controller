#ifndef __IMU_H__
#define __IMU_H__

//Different types of sensors/libraries defined here
#define IMU_MPU6050 0
#define IMU_MPU6050_DMP 1 //200 Hz max

//Select which IMU setup to use
#define IMU_TYPE IMU_MPU6050_DMP


//Import libraries
#if IMU_TYPE == IMU_MPU6050_DMP
  #include <Wire.h>
  #include <MPU6050_6Axis_MotionApps612.h>
#endif

extern const int lightPin;
extern float getLoopTime(int);


/* Settings */
const float angleOffset[3] = {8, 1.1, 0}; //IMU angle offset {roll, pitch and yaw}
const int rRateCount = 20;                //How many loops are used to calculate the rotation rate, minimum 2
/* Settings */


class IMU {
  public:
    int init();
    void updateAngle();
    float currentAngle[3] = {0, 0, 0}; //Current angle of roll, pitch and yaw (in degrees)
    float rRate[3] = {0, 0, 0};        //Rotation rate (degrees per millisecond) of roll, pitch and yaw

  private:
    float rotTime = 1;  //Time (in milliseconds) over which the rotation rate is calculated
    int SMAcounter = 0; //Index of rpSMA which was last used
    float rpSMA[50][3]; //Simple moving average of roll and pitch

    //Define variables specific to setups
    #if IMU_TYPE == IMU_MPU6050_DMP
      //MPU control/status vars
      MPU6050 mpu;
      uint8_t devStatus;      //Return status after each device operation (0 = success, !0 = error)
      uint8_t fifoBuffer[64]; //FIFO storage buffer
      //MPU orientation/motion vars
      Quaternion q;                     //Quaternion container
      VectorFloat gravity;              //Gravity vector
      float ypr[3];                     //Array containing yaw, pitch and roll
      const float MPUmult = 180 / M_PI; //Constant used to convert from MPU6050 data to degrees
    #endif
};
#endif
