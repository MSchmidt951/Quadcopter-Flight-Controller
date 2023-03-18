#ifndef __PIDcontroller_H__
#define __PIDcontroller_H__

//Import files
#include "IMU.h"
#include "Logger.h"

class MotorController;

extern int xyzr[4];

/** 
 * @class PIDcontroller
 * @brief Runs the PID calculations and stores the settings for it
 */
class PIDcontroller {
  public:
    /** Initialises a PIDcontroller object. Settings are configured in settings.json
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @param[in] parent Name of the parent object
     *  @param[in] name Name of the PID, must be the same as one of the objects in settings.json
     *  @param[in] targetPtr Pointer to the target value
     *  @param[in] currentPtr Pointer to the current value
     *  @param[in] currentDiffPtr Pointer to the current change in value, if NULL then it is calculated
     */
    void init(Logger &logger, const char* parent, const char* name, float* targetPtr, float* currentPtr, float* currentDiffPtr);
    /** Calculates and applies PID change to a MotorController
     *  
     *  @param[in] controller MotorController object to apply the PID to
     */
    void calc(MotorController* controller);
    /** Applies an offset to the target, used by InputHandler
     *  
     *  @param[in] input Value to offset the target by
     */
    void addInput(float input);
    /** Retrieves the name of the object
     *  
     *  @returns the name of the object
     */
    const char* getName();

  private:
    ///The name of the PID controller, used for loading settings from settings.json
    const char* namePtr;
    ///Number of positive pins
    int positivePinCount = 0;
    ///Array containing the indexes of pins which are affected positively by the PID
    int* positivePins;
    ///Number of negative pins
    int negativePinCount = 0;
    ///Array containing the indexes of pins which are affected negatively by the PID
    int* negativePins;
    ///The proportional, integral and derivative gains
    float PIDGains[3];

    ///Target value
    float* target;
    ///Current value
    float* current;
    ///Current differentail change
    float* currentDiff;

    ///Offset applied to the target. Determined by inputs
    float inputOffset = 0;
    ///Difference between the current and target
    float currentErr = 0;
    ///Difference between the current and target of the previous loop
    float lastErr = 0;
    ///The sum of the difference between the current and target value used to calculate the integral change
    float iSum = 0;
};
#endif
