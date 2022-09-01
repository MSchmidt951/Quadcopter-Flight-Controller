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
    //Performance
    const float Pgain[3] = {2.0, 2.0, 0};     //Proportional gain for roll, pitch & yaw, percentage difference per ESC at 10 degrees
    const float Igain[3] = {.0000, .0000, 0}; //Integral gain for roll, pitch & yaw, changes motor performance over time, devided by 1000 due to converting Isum to seconds
    const float Dgain[3] = {.33, .33, 0};     //Differential gain for roll, pitch & yaw, helps control the rotation speed
    /* Settings */

  private:
    float rpDiff[2] = {0,0}; //Difference in roll & pitch from the wanted angle
    float Isum[2];           //The sum of the difference in angles used to calculate the integral change
};
#endif
