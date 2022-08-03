#include "DroneRadio.h"
#include "IMU.h"
#include "Logger.h"
#include "MotorController.h"

/*** * * * DRONE SETTINGS * * * ***/
//Motor settings can be found in MotorController.h
//Log and SD card settings can be found in Logger.h
//User input
const float maxAngle = 127.0/15.0;   //Maximum wanted bank angle available to select by the user
const float yawControl = 0.0/127.0;  //How much the joystick affects yaw
//Offsets
const float rpOffset[2] = {6, 0};          //Base gyro angle offset
//Performance
const int rRateCount = 25;      //How many loops are used to calculate the rotation rate, minimum 2
const float Pgain = 2.4/1000;   //Proportional gain, percentage difference per ESC at 10 degrees
const float Igain = .0017/1000; //Integral gain, changes motor performance over time, devided by 1000 due to converting Isum to seconds
const float Dgain = -.33;       //Differential gain, helps control the rotation speed
const float yawGain = 0;        //Yaw differential gain
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
IMU imu(rpOffset[0], rpOffset[1]);
float rpDiff[2] = {0,0}; //Difference in roll & pitch from the wanted angle
float PIDchange[3][3];   //The change from the P, I and D values that will be applied to the roll, pitch & yaw; PIDchange[P/I/D][roll/pitch/yaw]
float Isum[2];           //The sum of the difference in angles used to calculate the integral change

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
  logger.logSetting("maxAngle", 127/maxAngle);
  logger.logSetting("yawControl", yawControl*127, 2);
  logger.logString("\nOffsets\n");
  logger.logSetting("motorOffset", ESC.offset, 4, 3, false);
  logger.logSetting("rpOffset", rpOffset, 2, 2);
  logger.logSetting("defaultZ", ESC.defaultZ);
  logger.logString("\nPerformance\n");
  logger.logSetting("rRateCount", rRateCount, false);
  logger.logSetting("Pgain", Pgain*1000, 2);
  logger.logSetting("Igain", Igain*1000, 4);
  logger.logSetting("Dgain", -Dgain, 3);
  logger.logSetting("yawGain", yawGain, 2);
  logger.logString("\nchangeLog,CHANGELOG GOES HERE\n");
  logger.logString("Time,Loop time,Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,Pr,Pp,Ir,Ip,Dr,Dp,FL,FR,BL,BR,radio,yaw");
  logger.write();

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
  for (int i=0; i<rRateCount; i++) {
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
    
    //Apply the calculated roll and pitch change
    ESC.addChange(PIDchange, 0, 0,2, 1,3);
    ESC.addChange(PIDchange, 1, 0,1, 2,3);

    //Yaw control
    if (xyzr[3] == 0) {
      PIDchange[2][2] = imu.rRate[2] * yawGain; //Stabilise yaw rotation
    } else {
      PIDchange[0][2] = xyzr[3] * yawControl; //Joystick control
    }
    ESC.addChange(PIDchange, 2, 0,3, 1,2);


    /* Apply input to hardware */
    digitalWrite(lightPin, light);
    ESC.write();
  
    /* Log flight info */
    logger.logData((micros()-startTime-standbyOffset) / 1000, false);
    logger.logData(getLoopTime(1), 1);
    logger.logArray(xyzr, 4);
    logger.logData(potPercent*100);
    logger.logArray(imu.currentAngle, 2, 2);
    logger.logArray(PIDchange[0], 2, 3);
    logger.logArray(PIDchange[1], 2, 3);
    logger.logArray(PIDchange[2], 2, 3);
    logger.logArray(ESC.motorPower, 4, 0);
    logger.logData(droneRadio.radioReceived);
    logger.logData(imu.currentAngle[2], 1);
    
    logger.write();
  }
}
