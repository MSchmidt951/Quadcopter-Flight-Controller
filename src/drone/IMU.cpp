#if IMU_TYPE == NONE
  #include "Arduino.h"
#endif
#include "IMU.h"

int IMU::init() {
  digitalWrite(lightPin, HIGH);
  #if IMU_TYPE == IMU_MPU6050
    Wire.begin();
    Wire.setClock(400000);
    
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    
    if (mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050) != 0x68) {
      return 1;
    }
    mpu.calibrateGyro();
    mpu.initMPU6050();
    
    aRes = mpu.getAres();
    gRes = mpu.getGres();
    
    //Get the current angle from an average of 10 readings
    for (int i=0; i<10; i++) {
      //Give time for the MPU6050 to update
      delay(1);
      //Get the accelerometer data
      mpu.readAccelData(accelData);
      for (int j=0; j<3; j++) {
        accelVal[j] = (float)accelData[j]*aRes;
      }
      
      //Calculate the angle from the accelerometer
      currentAngle[0] += atan2(accelVal[1], accelVal[2]);
      currentAngle[1] += atan(-accelVal[0] / sqrt(accelVal[1]*accelVal[1] + accelVal[2]*accelVal[2]));
      
      //Get the initial value for the kalman filter
      rollKalman.updateEstimate((-currentAngle[0] * 180/PI) / (i+1));
      pitchKalman.updateEstimate((currentAngle[1] * 180/PI) / (i+1));
    }
    //Get the average from the ten readings
    currentAngle[0] /= 10;
    currentAngle[1] /= 10;
    
    //Set the quaternion to the current angle
    eulerToQuat(currentAngle[0], currentAngle[1], PI);
  #elif IMU_TYPE == IMU_MPU6050_DMP
    Wire.begin();
    Wire.setClock(400000);
    
    //Set up MPU 6050
    mpu.initialize();
    int devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(-5054);
    mpu.setYAccelOffset(297);
    mpu.setZAccelOffset(1024);
    mpu.setXGyroOffset(122);
    mpu.setYGyroOffset(-101);
    mpu.setZGyroOffset(22);
    
    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      //Let the MPU run for a bit to allow for values to stabilise
      for (int i=0; i<1000; i++) {
        delay(4);
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
      }
    } else {
      return devStatus;
    }
  #endif

  digitalWrite(lightPin, LOW);
  return 0;
}

//Get the current angle and rotation rate from
void IMU::updateAngle() {
  #if IMU_TYPE == IMU_MPU6050
    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
      //Read data from MPU6050
      mpu.readAccelData(accelData);
      mpu.readGyroData(gyroData);

      //Calculate values into usable units
      for (int i=0; i<3; i++) {
        accelVal[i] = (float)accelData[i]*aRes;
        gyroVal[i] = (float)gyroData[i]*gRes;
      }
    }

    //Update quaternion values
    MadgwickQuaternionUpdate(accelVal, gyroVal);
    //Convert quaternion to roll, pitch and yaw
    currentAngle[0] = -atan2(2 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
    currentAngle[1] = -asin(2 * (q[1]*q[3] - q[0]*q[2]));
    currentAngle[2] = atan2(2 * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
    //Convert from radians to degrees
    for (int i=0; i<3; i++) {
      currentAngle[i] *= 180 / PI;
    }

    //Apply kalman filter to roll and pitch
    currentAngle[0] = rollKalman.updateEstimate(currentAngle[0]);
    currentAngle[1] = pitchKalman.updateEstimate(currentAngle[1]);

    //Get rotation rate
    for (int i=0; i<3; i++) {
      rRate[i] = gyroVal[i];
    }
  #elif IMU_TYPE == IMU_MPU6050_DMP
    //Get current angle and acceleration
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(gyroData, fifoBuffer);
    }
    currentAngle[0] = (-ypr[2] * MPUmult) + angleOffset[0]; //roll
    currentAngle[1] = (-ypr[1] * MPUmult) + angleOffset[1]; //pitch
    currentAngle[2] = ( ypr[0] * MPUmult) + angleOffset[2]; //yaw
    
    //Get rotation rate
    for (int i=0; i<3; i++) {
      rRate[i] = gyroData[i] * 2000.0/32768.0;;
    }
  #elif IMU_TYPE == NONE
    for (int i=0; i<3; i++) {
      rRate[i] = 0;
      currentAngle[i] = 0;
    }
  #endif
}

//Functions for specific setups
#if IMU_TYPE == IMU_MPU6050
  //Kris Winer's implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
  //I have changd the inputs to work with my program
  void IMU::MadgwickQuaternionUpdate(float *accel, float *gyro) {
    //Convert inputs
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    float gyrox = gyro[0] * PI / 180.0f;
    float gyroy = gyro[1] * PI / 180.0f;
    float gyroz = gyro[2] * PI / 180.0f;

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         //short name local variable for readability
    float norm;                                               //vector norm
    float f1, f2, f3;                                         //objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; //objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        //gyro bias error

    //Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    //Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; //handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    //Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    //Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    //Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    //Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    //Compute and remove gyroscope biases
    gbiasx += gerrx * loopTime()/1000 * zeta;
    gbiasy += gerry * loopTime()/1000 * zeta;
    gbiasz += gerrz * loopTime()/1000 * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;

    //Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    //Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * loopTime()/1000;
    q2 += (qDot2 -(beta * hatDot2)) * loopTime()/1000;
    q3 += (qDot3 -(beta * hatDot3)) * loopTime()/1000;
    q4 += (qDot4 -(beta * hatDot4)) * loopTime()/1000;

    //Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    //normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
  }

  void IMU::eulerToQuat(float roll, float pitch, float yaw) {
    q[0] = (sin(yaw/2) * cos(-pitch/2) * cos(roll/2)) - (cos(yaw/2) * sin(-pitch/2) * sin(roll/2));
    q[1] = (cos(yaw/2) * sin(-pitch/2) * cos(roll/2)) + (sin(yaw/2) * cos(-pitch/2) * sin(roll/2));
    q[2] = (cos(yaw/2) * cos(-pitch/2) * sin(roll/2)) - (sin(yaw/2) * sin(-pitch/2) * cos(roll/2));
    q[3] = (cos(yaw/2) * cos(-pitch/2) * cos(roll/2)) + (sin(yaw/2) * sin(-pitch/2) * sin(roll/2));
  
    //Normalise quaternion
    float norm = 1/sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;
  }
#endif
