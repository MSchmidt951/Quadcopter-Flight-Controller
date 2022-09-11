#include "PIDcontroller.h"

void PIDcontroller::init(Logger &logger) {
  //Load settings from the SD card
  logger.loadSetting("maxAngle", maxAngle);
  logger.loadSetting("Pgain", Pgain, 3);
  logger.loadSetting("Igain", Igain, 3);
  logger.loadSetting("Dgain", Dgain, 3);

  maxAngle = 127.0/maxAngle;
}

void PIDcontroller::calcPID(IMU imu) {
  //Calculate the change in motor power per axis
  for (int i=0; i<2; i++) {
    //Get difference between wanted and current angle
    rpDiff[i] = (-xyzr[i]/maxAngle) - imu.currentAngle[i];

    //Get proportional change
    PIDchange[0][i] = rpDiff[i] * Pgain[i]/1000.0;

    //Get integral change
    Isum[i] += rpDiff[i] * loopTime();
    PIDchange[1][i] = Isum[i] * Igain[i]/1000.0;

    //Get derivative change
    PIDchange[2][i] = imu.rRate[i] * -Dgain[i];
  }

  //Yaw control
  if (xyzr[3] == 0) {
    PIDchange[2][2] = imu.rRate[2] * Dgain[2]; //Stabilise yaw rotation
  } else {
    PIDchange[0][2] = xyzr[3] * Pgain[2]; //Joystick control
  }
}
