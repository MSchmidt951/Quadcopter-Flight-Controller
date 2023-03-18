#ifndef __MotorController_H__
#define __MotorController_H__

#define MAX_PIDS 6

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
#include "PIDcontroller.h"

class InputHandler;

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
    /** Initialises the motors by loading the necessary settings then arming them
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @param[in] motorName The name of the motor
     *  @param[in] test Whether to test the motors
     *  @returns True if init successful
     */
    bool init(Logger &logger, const char* motorName, bool test=false);
    /** Adds a PID instance to the class. The PID settings are configured in settings.json
     *  
     *  @param[in] name The name of the PID, must be the same as one of the objects in settings.json
     *  @param[in] target A pointer to the target value
     *  @param[in] current A pointer to the current value
     *  @param[in] currentDiff A pointer to the current change in value, if NULL then it is calculated
     */
    void addPID(const char* name, float* target, float* current, float* currentDiff=NULL);
    /** Adds a PID instance to the class with a constant target value. The PID settings are configured in settings.json
     *  
     *  @param[in] name The name of the PID, must be the same as one of the objects in settings.json
     *  @param[in] target The target value
     *  @param[in] current A pointer to the current value
     *  @param[in] currentDiff A pointer to the current change in value, if NULL then it is calculated
     */
    void addPID(const char* name, float target,  float* current, float* currentDiff=NULL);
    /** Retrieves the controls settings from settings.json. Call after all PIDs are added */
    void setupInputs();
    ////void logSettings();

    /** Adds a percentage of power to a certain motor
     *  
     *  @param[in] index An index in the array of motors
     *  @param[in] amount The amount to add, 0.1 would add 10% power
     */
    void addMotorPower(int index, float amount);
    /** Calculates and writes the percentages to the motors */
    void write();
    /** Turns off all motors */
    void writeZero();

    /** Get the number of PID controllers attached to the controller
     *  
     *  @returns Number of PID controllers attached to the controller
     */
    int getPIDcount();

    ///An array containing all of the PID controllers
    PIDcontroller* PIDs;

  private:
    /** Write to a motor with a certain value
     * 
     *  @param[in] index Index of the motor in the motors array
     *  @param[in] value Speed of motor, 0 - 1000
     */
    void writeToMotor(int index, float value);

    ///The number of motors the object has
    int motorCount = 0;
    ///Stores the pin numbers of the motors
    int* motors;
    #if ESC_TYPE == PWM
      ///Holds the PWM signal being sent to each motor
      Servo* motorSignal;
    #elif ESC_TYPE == ONESHOT125
      ///Holds the OneShot125 signal being sent to each motor
      Teensy_PWM** motorSignal;
      //Frequency of the signal being sent
      float signalFreq = 3500.0f;
      //Maximum duty cycle allowed to be sent, depends on signalFreq
      float maxDutyCycle = signalFreq/40.0f;
    #endif

    ///The default percentage of each of the motors
    float* defaultValues;
    ///Current percentage of each motor
    float* motorPower;
    ///The number of inputs the motor controller has
    int inputCount = 0;
    ///The array holding all of the inputs
    InputHandler* inputs;
    ///The number of PID controllers the motor controller has
    int PIDcount = 0;

    ///A reference to the logger
    Logger* logger;
    ///The name of the motor controller, used for loading settings from settings.json
    const char* name;
};


/** 
 * @class InputHandler
 * @brief Converts raw inputs and applies them to a motor controller or a PID controller
 */
class InputHandler {
  public:
    /** Initialises the motors by loading the necessary settings then arming them
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @param[in] controller Pointer to the parent motor cotroller
     *  @param[in] parent The name of the parent motor cotroller
     *  @param[in] name The name of the input handler, must be the same as one of the objects in settings.json
     */
    void init(Logger &l, MotorController* controller, const char* parent, const char* name);
    /** Processes the input and applies it to the motor controller or one of the PID controllers owned by the motor controller
     *  
     *  @param[in] controller Pointer to the motor cotroller
     */
    void processInput(MotorController* controller);

  private:
    ///The current value of the input
    float input;
    ///Value of the output when the input is at its minimum
    float minControl;
    ///Value of the output when the input is at its mid point
    float midControl;
    ///Value of the output when the input is at its maximum
    float maxControl;

    ///The number of output pins that the handler controls
    int outputPinCount = 0;
    ///Array containing the indexes of output pins that the handler controls
    int* outputPin;
    ///Name off the PID controller to control
    const char* PIDTargetStr;
    ///Reference to the PID controller to control
    PIDcontroller* PIDTarget;
};
#endif
