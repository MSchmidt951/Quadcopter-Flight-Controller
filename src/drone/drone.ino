#include <SdFat.h>

#include "DroneRadio.h"
#include "IMU.h"
#include "MotorController.h"

/*** * * * DRONE SETTINGS * * * ***/
//Motor settings can be found in MotorController.h
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
int timerIndex;
unsigned long loopTimings[20];
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
const int chipSelect = SS;
DroneRadio droneRadio;
MotorController ESC;

//Log vars
SdFs sd;
FsFile logFile;
const int logFileSize = 32 * 1024 * 1024; //32 million bytes of pre allocated data (used up in around 10 mins)
const int maxLogLoopCounter = 10;         //How often the log is written to the SD card
int logLoopCounter = 0;

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

String logArray(int *arr, int len){
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i]);
  }
  return arrayString;
}
String logArray(float *arr, int len, int decimals){
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i], decimals);
  }
  return arrayString;
}

void calcTime(){
  if (timerIndex < 20) {
    loopTimings[timerIndex] = micros();
    timerIndex++;
  }
}

//This function is to be used along with calcTime to calculate how long each part of the loop takes
void logLoopTimings(){
  //Process
  for(int i=0; i<19; i++) {
    if (loopTimings[i+1] != 0) {
      loopTimings[i] = loopTimings[i+1] - loopTimings[i];
    }
  }
  
  //Log
  logFile.println(logArray(loopTimings, timerIndex-1));
  
  //Reset the variables for the next loop
  timerIndex = 0;
  for(int i=1; i<20; i++) {
    loopTimings[i] = 0;
  }
  calcTime();
}


void checkSD(bool condition) {
  if (!condition) {
    for (;;) {
      blink(1500);
    }
  }
}

void standby() {
  if (standbyStatus == 1) {
    standbyStatus = 2;
    
    //Turn off all motors
    ESC.writeZero();
  
    //Log the drone going into standby
    logFile.println("\n--- on standby ---\n");
    logFile.flush();

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
  
  logFile.flush();
  logFile.truncate();
  logFile.close();
  
  for (;;){
    ESC.writeZero();
    blink(111);
    blink(1234);
  }
}


void setup(){
  pinMode(lightPin, OUTPUT);

  //Set up SD card
  checkSD(sd.begin(SdioConfig(FIFO_SDIO)));
  if (sd.exists("log.csv")) {
    sd.remove("log_old.csv");
    logFile.open("log.csv", O_WRITE);
    logFile.rename("log_old.csv");
    logFile.close();
  }
  logFile.open("log.csv", O_WRITE | O_CREAT | O_TRUNC);
  checkSD(logFile.preAllocate(logFileSize));
  //Log the settings
  logFile.println("User input");
  logFile.print("maxZdiff"); logFile.print(logArray(ESC.maxZdiff, 2, 2));
  logFile.print(",potMaxDiff,"); logFile.print(ESC.potMaxDiff);
  logFile.print(",maxAngle,"); logFile.print(127/maxAngle);
  logFile.print(",yawControl,"); logFile.print(yawControl*127, 2);
  logFile.println("\nOffsets");
  logFile.print("motorOffset"); logFile.print(logArray(ESC.offset, 4, 3));
  logFile.print(",rpOffset"); logFile.print(logArray(rpOffset, 2, 2));
  logFile.print(",defaultZ,"); logFile.print(ESC.defaultZ);
  logFile.println("\nPerformance");
  logFile.print("rRateCount:,"); logFile.print(rRateCount);
  logFile.print(",Pgain,"); logFile.print(Pgain*1000, 2);
  logFile.print(",Igain,"); logFile.print(Igain*1000, 4);
  logFile.print(",Dgain,"); logFile.print(-Dgain, 3);
  logFile.print(",yawGain,"); logFile.print(yawGain, 2);
  logFile.println("\nchangeLog,");
  logFile.println("Time,Loop time,Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,Pr,Pp,Ir,Ip,Dr,Dp,FL,FR,BL,BR,radio,yaw");
  logFile.flush();

  //Set up motors
  ESC.init();

  //Set up inertial measurement unit
  if (imu.init()) {
    logFile.print("IMU error");
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
  }
  //Start the clock
  startTime = micros();
}


void loop(){
  logLoopTimings();

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
    logFile.print((micros()-startTime-standbyOffset) / 1000);
    logFile.print("," + String(getLoopTime(1), 1));
    logFile.print(logArray(xyzr, 4));
    logFile.print("," + String(potPercent*100));
    logFile.print(logArray(imu.currentAngle, 2, 2));
    logFile.print(logArray(PIDchange[0], 2, 3));
    logFile.print(logArray(PIDchange[1], 2, 3));
    logFile.print(logArray(PIDchange[2], 2, 3));
    logFile.print(logArray(ESC.motorPower, 4, 0));
    logFile.print("," + String(droneRadio.radioReceived));
    logFile.print("," + String(imu.currentAngle[2], 1));
    
    logLoopCounter++;
    if (logLoopCounter == maxLogLoopCounter) {
      logFile.flush();
      logLoopCounter = 0;
    }
  }
}
