#include "DroneRadio.h"
#include "IMU.h"
#include "Logger.h"
#include "MotorController.h"
#include "PIDcontroller.h"

/*** * * * DRONE SETTINGS * * * ***/
const int loopRate = 2000; //Maxiumum loop rate (Hz)
const int maxLoopTime = 1000000/loopRate; //Maximum loop time (us)

//IMU and sensor settings can be found in IMU.h
//Log and SD card settings can be found in Logger.h
//Motor settings can be found in MotorController.h
//PID settings can be found in PIDcontroller.h
/*** * * * DRONE SETTINGS * * * ***/

//Time vars
unsigned long startTime;         //Start time of flight (in milliseconds)
unsigned long loopTimestamp;     //Timestamp of the current loop
unsigned long lastLoopTimestamp; //Timestamp of last loop
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

//Return the loop time in milliseconds
float loopTime(){
  return (loopTimestamp - lastLoopTimestamp) / 1000.0;
}
//Return the loop time in microseconds
unsigned long loopTimeMicro(){
  return loopTimestamp - lastLoopTimestamp;
}

void standby() {
  if (standbyStatus == 1) {
    standbyStatus = 2;
    
    //Turn off all motors
    ESC.writeZero();
  
    //Log the drone going into standby
    logger.logString("\n--- on standby ---\n");

    standbyStartTime = micros();
  }

  droneRadio.timer = 0;
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

  //Set up PID controller
  pid.init(logger);

  //Log the settings
  logger.logString("User input\n");
  logger.logSetting("maxZdiff", ESC.maxZdiff, 2, 2, false);
  logger.logSetting("potMaxDiff", ESC.potMaxDiff);
  logger.logSetting("maxAngle", 127/pid.maxAngle);
  logger.logString("\nOffsets\n");
  logger.logSetting("motorOffset", ESC.offset, 4, 3, false);
  logger.logSetting("angleOffset", angleOffset, 3, 2);
  logger.logSetting("defaultZ", ESC.defaultZ);
  logger.logString("\nPerformance\n");
  logger.logSetting("Loop rate", loopRate, false);
  logger.logSetting("Pgain", pid.Pgain, 3, 2);
  logger.logSetting("Igain", pid.Igain, 3, 4);
  logger.logSetting("Dgain", pid.Dgain, 3, 3);
  logger.logString("\nchangeLog,CHANGELOG GOES HERE\n");
  logger.logString("Time (μs),Loop time (μs),Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,Pr,Pp,Ir,Ip,Dr,Dp,radio,yaw");

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
  delay(10);
  for (int i=0; i<3; i++){
    blink(100);
  }

  //Start the clock
  startTime = micros();
  loopTimestamp = startTime;
  lastLoopTimestamp = startTime;
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
    droneRadio.checkSignal(loopTimeMicro(), loopTimestamp);


    /* Get current angle */
    imu.updateAngle();
    
    //Get loop time
    lastLoopTimestamp = loopTimestamp;
    loopTimestamp = micros()-standbyOffset;
    //Make sure the loop is executing no faster than the max loop time
    while (loopTimeMicro() < maxLoopTime) {
      delayMicroseconds(1);
      loopTimestamp = micros()-standbyOffset;
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
    if (logger.checkLog()) {
      logger.logTime(micros()-startTime-standbyOffset);
      for (int i=0; i<4; i++) {
        logger.logData(xyzr[i], typeID.uint8);
      }
      logger.logData((uint8_t)(potPercent*255), typeID.uint8);
      for (int i=0; i<2; i++) {
        logger.logData(imu.currentAngle[i], typeID.float32);
      }
      for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
          logger.logData(pid.PIDchange[i][j], typeID.float16k);
        }
      }
      logger.logData((uint16_t)(droneRadio.timer/1000), typeID.uint16);
      logger.logData(imu.currentAngle[2], typeID.float16);

      logger.write();
    }
  }
}
