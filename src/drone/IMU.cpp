#include "IMU.h"

int IMU::init() {
  #if IMU_TYPE == IMU_MPU6050_DMP
    Wire.begin();
    Wire.setClock(400000);
    
    //Set up MPU 6050
    digitalWrite(lightPin, HIGH);
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
    digitalWrite(lightPin, LOW);
    
    //Fill rpSMA
    for (int i=0; i<rRateCount; i++) {
      rpSMA[i][0] = (-ypr[2] * MPUmult) + angleOffset[0];
      rpSMA[i][1] = (-ypr[1] * MPUmult) + angleOffset[1];
      rpSMA[i][2] =   ypr[0] * MPUmult;
    }
  #endif
  return 0;
}

//Get the current angle and rotation rate from
void IMU::updateAngle() {
  #if IMU_TYPE == IMU_MPU6050_DMP
    //Get current angle and acceleration
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    currentAngle[0] = (-ypr[2] * MPUmult) + angleOffset[0]; //roll
    currentAngle[1] = (-ypr[1] * MPUmult) + angleOffset[1]; //pitch
    currentAngle[2] = ( ypr[0] * MPUmult) + angleOffset[2]; //yaw
    
    //Update SMA values
    SMAcounter++;
    SMAcounter %= rRateCount;
    for (int i=0; i<3; i++) {
      rpSMA[SMAcounter][i] = currentAngle[i];
    }
  
    //Calculate rotation rate
    rotTime = getLoopTime(-1);
    for (int i=0; i<3; i++) {
      rRate[i] = (rpSMA[SMAcounter][i] - rpSMA[(SMAcounter+1) % rRateCount][i]) / rotTime;
    }
  #endif
}
