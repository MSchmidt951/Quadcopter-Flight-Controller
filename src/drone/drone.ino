#include "DroneRadio.h"
#include "IMU.h"
#include "Logger.h"
#include "MotorController.h"
#include "PIDcontroller.h"

/*** * * * DRONE SETTINGS * * * ***/
//IMU and sensor settings can be found in IMU.h
//Log and SD card settings can be found in Logger.h
//Motor settings can be found in MotorController.h
//PID settings can be found in PIDcontroller.h
/*** * * * DRONE SETTINGS * * * ***/

//Time vars
unsigned long startTime;            //Start time of flight (in milliseconds)
unsigned long loopTime[rRateCount]; //Timestamp of last few loops (in milliseconds)
int loopTimeCounter = 0;            //Index of loopTime which was last used
//Standby
int standbyStatus = 0; //0: not on standby, 1: starting standby, 2: on standby
unsigned long standbyStartTime;
unsigned long standbyOffset = 0;
bool standbyLights = true;
unsigned long lightChangeTime = 0;

//Input vars
int xyzr[4] = {0,0,0,0}; //Joystick inputs for x, y, z and rotation(yaw) (from -127 to 127)
float potPercent = 0;    //Percentage of the controllers potentiometer, used as a trim
bool light = false;
bool standbyButton = false;

//Rotation vars
IMU imu;
PIDcontroller pid;

//Hardware vars
const int lightPin = 5;
DroneRadio droneRadio;
MotorController ESC;
Logger logger;

//Functions

void blink(int d){
  digitalWrite(lightPin, HIGH);
  delay(d);
  digitalWrite(lightPin, LOW);
  delay(d);
}

float getLoopTime(int Step){ //Return the loop time in microseconds
  int index = (loopTimeCounter-Step) % rRateCount;
  if (index < 0){
    index = rRateCount+index;
  }
  return (loopTime[loopTimeCounter] - loopTime[index]) / 1000.0;
}

void standby() {
  if (standbyStatus == 1) {
    standbyStatus = 2;
    
    //Turn off all motors
    ESC.writeZero();
  
    //Log the drone going into standby
    logger.logString("\n--- on standby ---\n");
    logger.write(true);

    standbyStartTime = micros();
  }

  droneRadio.radioReceived = 0;
  //Blink lights
  if (millis()-lightChangeTime > 750) {
    lightChangeTime = millis();
    standbyLights = !standbyLights;
  }
  digitalWrite(lightPin, standbyLights);
}

void ABORT(){ //This is also used to turn off all the motors after landing
  ESC.writeZero();

  logger.closeFile();
  
  for (;;){
    ESC.writeZero();
    blink(111);
    blink(1234);
  }
}


void setup(){
  pinMode(lightPin, OUTPUT);

  //Set up SD card
  logger.init();
  //Log the settings
  logger.logString("User input\n");
  logger.logSetting("maxZdiff", ESC.maxZdiff, 2, 2, false);
  logger.logSetting("potMaxDiff", ESC.potMaxDiff);
  logger.logSetting("maxAngle", 127/pid.maxAngle);
  logger.logSetting("yawControl", pid.yawControl*127, 2);
  logger.logString("\nOffsets\n");
  logger.logSetting("motorOffset", ESC.offset, 4, 3, false);
  logger.logSetting("angleOffset", angleOffset, 3, 2);
  logger.logSetting("defaultZ", ESC.defaultZ);
  logger.logString("\nPerformance\n");
  logger.logSetting("rRateCount", rRateCount, false);
  logger.logSetting("Pgain", pid.Pgain*1000, 2);
  logger.logSetting("Igain", pid.Igain*1000, 4);
  logger.logSetting("Dgain", -pid.Dgain, 3);
  logger.logSetting("yawGain", pid.yawGain, 2);
  logger.logString("\nchangeLog,CHANGELOG GOES HERE\n");
  logger.logString("Time,Loop time,Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,Pr,Pp,Ir,Ip,Dr,Dp,FL,FR,BL,BR,radio,yaw");
  logger.write(true);

  //Set up motors
  ESC.init();

  //Set up inertial measurement unit
  if (imu.init()) {
    logger.logString("IMU error");
    ABORT();
  }
  
  //Set up communication
  droneRadio.init();

  //Startup lights
  delay(100);
  for (int i=0; i<3; i++){
    blink(150);
  }

  //Get time variables
  for (int i=rRateCount-1; i>=0; i--) {
    loopTime[i] = micros();
    delayMicroseconds(1);
  }
  //Start the clock
  startTime = micros();
}


void loop(){
  // Recieve input data
  droneRadio.getInput();

  //Check standby status
  if (standbyButton and standbyStatus == 0) {
    standbyStatus = 1;
  } else if (!standbyButton and standbyStatus == 2) {
    standbyOffset += micros() - standbyStartTime;
    standbyStatus = 0;
  }
  
  if (standbyStatus > 0) {
    standby();
  } else {
    //Check radio signal
    droneRadio.checkSignal();


    /* Get current angle */
    imu.updateAngle();
    
    //Get loop time
    loopTimeCounter++;
    loopTimeCounter %= rRateCount;
    loopTime[loopTimeCounter] = micros()-standbyOffset;
    //Make sure the loop is at least 2 milliseconds long
    while (getLoopTime(1) < 2) {
      delayMicroseconds(2);
      loopTime[loopTimeCounter] = micros()-standbyOffset;
    }


    /* Calculate motor speeds */
    pid.calcPID(imu);
    
    //Apply the calculated roll, pitch and yaw change
    ESC.addChange(pid.PIDchange, 0, 0,2, 1,3);
    ESC.addChange(pid.PIDchange, 1, 0,1, 2,3);
    ESC.addChange(pid.PIDchange, 2, 0,3, 1,2);


    /* Apply input to hardware */
    digitalWrite(lightPin, light);
    ESC.write();
  
    /* Log flight info */
    logger.logData((micros()-startTime-standbyOffset) / 1000, false);
    logger.logData(getLoopTime(1), 1);
    logger.logArray(xyzr, 4);
    logger.logData(potPercent*100);
    logger.logArray(imu.currentAngle, 2, 2);
    logger.logArray(pid.PIDchange[0], 2, 3);
    logger.logArray(pid.PIDchange[1], 2, 3);
    logger.logArray(pid.PIDchange[2], 2, 3);
    logger.logArray(ESC.motorPower, 4, 0);
    logger.logData(droneRadio.radioReceived);
    logger.logData(imu.currentAngle[2], 1);
    
    logger.write();
  }
}
