#include "PIDcontroller.h"

void PIDcontroller::calcPID(IMU imu) {
  //Calculate the change in motor power per axis
  for (int i=0; i<2; i++) {
    //Get difference between wanted and current angle
    rpDiff[i] = (-xyzr[i]/maxAngle) - imu.currentAngle[i];

    //Get proportional change
    PIDchange[0][i] = Pgain * rpDiff[i];

    //Get integral change
    Isum[i] += rpDiff[i] * getLoopTime(1);
    PIDchange[1][i] = Igain * Isum[i];

    //Get derivative change
    PIDchange[2][i] = Dgain * imu.rRate[i];
  }

  //Yaw control
  if (xyzr[3] == 0) {
    PIDchange[2][2] = imu.rRate[2] * yawGain; //Stabilise yaw rotation
  } else {
    PIDchange[0][2] = xyzr[3] * yawControl; //Joystick control
  }
}
