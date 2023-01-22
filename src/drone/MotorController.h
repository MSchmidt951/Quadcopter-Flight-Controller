#ifndef __MotorController_H__
#define __MotorController_H__

//ESC signal type defenitions
#define PWM 1 //1000 to 2000 μs pulse length
#define ONESHOT125 2 //125 to 250 μs pulse length

//Set ESC signal type
#define ESC_TYPE ONESHOT125


//Import libraries
#if ESC_TYPE == PWM
  #include <Servo.h>
#elif ESC_TYPE == ONESHOT125
  #include <Teensy_PWM.h>
#endif

//Import files
#include "Logger.h"

extern const int lightPin;
extern int xyzr[4];
extern float potPercent;


/** Rounds a float to the nearest integer */
int toInt(float f);

/** 
 * @class MotorController
 * @brief Controls calculations for motor percentages and communicating with ESCs
 */
class MotorController {
  public:
    /** Load settings from storage. Arm ESCs and test spin the motors
     *  
     *  @param[in] logger Logger object to read the settings from
     */
    void init(Logger &logger);
    /** Use the PID controls to add a change per motor on a certain axis.
     *  
     *  The axis is split in two (positive and negative).
     *  The positive side adds the change and the negative side subtracts the change.
     *  
     *  @param[in] PIDchange See PIDcontroller::PIDchange
     *  @param[in] axis Which axis to add change to (0: roll, 1: pitch, 2:yaw)
     *  @param[in] pA Index of motor A of the positive side
     *  @param[in] pB Index of motor B of the positive side
     *  @param[in] nA Index of motor A of the negative side
     *  @param[in] nB Index of motor B of the negative side
     */
    void addChange(float PIDchange[3][3], int axis, int pA, int pB, int nA, int nB);
    /** Calculate motor percentages and write to ESC */
    void write();
    /** Turns off all motors */
    void writeZero();
    
    ///Percentage of each motor. Order: {FL, FR, BL, BR}
    float motorPower[4];

    /* Settings */
    ///Base percentage difference per motor. Can be set via SD card
    float offset[4] = {0, 0, 0, 0};
    ///Default motor percentage. Can be set via SD card
    float defaultZ = .35;
    ///Percentage z difference for joystick down & up, respectively. Can be set via SD card
    float maxZdiff[2] = {.1, .18};
    ///Max percentage difference of potentiometer which acts like a trim for the base motor power. Can be set via SD card
    float potMaxDiff = .1;
    /* Settings */

  private:
    /** Write to a motor with a certain value
     * 
     *  @param[in] index Index of the motor in the motors array
     *  @param[in] value Speed of motor, 0 - 1000
     */
    void writeToMotor(int index, float value);

    ///Which pin controls which motor. Order: {FL, FR, BL, BR}
    const int motors[4] = {23, 22, 21, 20};
    ///Base motor power percentage at the start of each loop
    float initialPower;
    ///Percentage change of each motor. Usually from the PID controller
    float dynamicChange[4];
    
    #if ESC_TYPE == PWM
      ///Holds the PWM signal being sent to each motor
      Servo ESCsignal[4];
    #elif ESC_TYPE == ONESHOT125
      ///Holds the OneShot125 signal being sent to each motor
      Teensy_PWM* ESCsignal[4];
      //Frequency of the signal being sent to the ESC (Hz)
      float signalFreq = 3500.0f;
      //Maximum duty cycle allowed to be sent to the ESC, depends on signalFreq
      float maxDutyCycle = signalFreq/40.0f;
    #endif
};
#endif
