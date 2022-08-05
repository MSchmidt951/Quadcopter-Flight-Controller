#ifndef __PIDcontroller_H__
#define __PIDcontroller_H__

//Import files
#include "IMU.h"

extern int xyzr[4];


class PIDcontroller {
  public:
    void calcPID(IMU imu);
    
    float PIDchange[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}}; //The change from the P, I and D values that will be applied to the roll, pitch & yaw; PIDchange[P/I/D][roll/pitch/yaw]
    /* Settings */
    //User input
    const float maxAngle = 127.0/15.0;   //Maximum wanted bank angle available to select by the user
    const float yawControl = 0.0/127.0;  //How much the joystick affects yaw
    //Performance
    const float Pgain = 2.0/1000;   //Proportional gain, percentage difference per ESC at 10 degrees
    const float Igain = .0000/1000; //Integral gain, changes motor performance over time, devided by 1000 due to converting Isum to seconds
    const float Dgain = -.33;       //Differential gain, helps control the rotation speed
    const float yawGain = 0;        //Yaw differential gain
    /* Settings */

  private:
    float rpDiff[2] = {0,0}; //Difference in roll & pitch from the wanted angle
    float Isum[2];           //The sum of the difference in angles used to calculate the integral change
};
#endif
