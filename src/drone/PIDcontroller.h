#ifndef __PIDcontroller_H__
#define __PIDcontroller_H__

//Import files
#include "IMU.h"
#include "Logger.h"

extern int xyzr[4];

/** 
 * @class PIDcontroller
 * @brief Runs the PID calculations and stores the settings for it
 */
class PIDcontroller {
  public:
    /** Loads settings from the SD card.
     *  
     *  @param[in] logger Logger object to read the settings from
     */
    void init(Logger &logger);
    /** Calculate new PID values based on the IMU data.
     *  
     *  @param[in] imu IMU object to read the data from
     */
    void calcPID(IMU imu);
    
    ///The change from the P, I and D values that will be applied to the roll, pitch & yaw; PIDchange[P/I/D][roll/pitch/yaw]
    float PIDchange[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

    /* Settings */
    //User input
    ///Maximum wanted bank angle available to select by the user. Can be set via SD card
    float maxAngle = 15.0;
    //Performance
    ///Proportional gain for roll, pitch & yaw, percentage difference per ESC at 10 degrees. Can be set via SD card
    float Pgain[3] = {2.0, 2.0, 0};
    ///Integral gain for roll, pitch & yaw, changes motor performance over time. Can be set via SD card
    float Igain[3] = {.0000, .0000, 0};
    ///Differential gain for roll, pitch & yaw, helps control the rotation speed. Can be set via SD card
    float Dgain[3] = {.33, .33, 0};
    /* Settings */

  private:
    ///Difference in roll & pitch from the wanted angle
    float rpDiff[2] = {0,0};
    ///The sum of the difference in angles used to calculate the integral change
    float Isum[2];
};
#endif
