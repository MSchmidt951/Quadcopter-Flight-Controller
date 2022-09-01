#ifndef __MotorController_H__
#define __MotorController_H__

//ESC signal type defenitions
#define PWM 1 //1000 to 2000 μs pulse length

//Set ESC signal type
#define ESC_TYPE PWM


//Import libraries
#if ESC_TYPE == PWM
  #include <Servo.h>
#endif

extern const int lightPin;
extern int xyzr[4];
extern float potPercent;


int toInt(float f);

class MotorController {
  public:
    void init();
    void addChange(float PIDchange[3][3], int axis, int pA, int pB, int nA, int nB);
    void write();
    void writeZero();
    
    float motorPower[4]; //Percentage; FL, FR, BL, BR

    /* Settings */
    const float offset[4] = {0, 0, 0, 0}; //Base percentage difference per motor
    const float defaultZ = .35;           //Default motor percentage
    const float maxZdiff[2] = {.1, .18};  //Percentage z difference for joystick down & up, respectively
    const float potMaxDiff = .1;          //Max percentage difference of potentiometer which acts like a trim for the base motor power
    /* Settings */

  private:
    void writeToMotor(int index, int value); //Write to a motor with a certain value (0 - 1000)

    const int motors[4] = {23, 22, 21, 20}; //FL, FR, BL, BR
    float initialPower;                     //The base motor power percentage at the start of each loop
    float dynamicChange[4];                 //This is the percentage change for example from the PID controller
    
    #if ESC_TYPE == PWM
      Servo ESCsignal[4];
    #endif
};
#endif
