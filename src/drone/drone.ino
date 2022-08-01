#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <SdFat.h>

/*** * * * DRONE SETTINGS * * * ***/
//User input
const float maxZdiff[2] = {.1, .18}; //Percentage z difference for joystick down & up, respectively
const float potMaxDiff = .06;        //Max percentage difference of potentiometer
const float maxAngle = 127.0/15.0;   //Maximum wanted bank angle available to select by the user
const float yawControl = 0.0/127.0;  //How much the joystick affects yaw
//Offsets
const float motorOffset[4] = {0, 0, 0, 0}; //Base percentage difference per motor
const float rpOffset[2] = {6, 0};          //Base gyro angle offset
const float defaultZ = .36;                //Default motor percentage
//Performance
const int rRateCount = 25;      //How many loops are used to calculate the rotation rate, minimum 2
const float Pgain = 2.4/1000;   //Proportional gain, percentage difference per ESC at 10 degrees
const float Igain = .0017/1000; //Integral gain, changes motor performance over time, devided by 1000 due to converting Isum to seconds
const float Dgain = -.33;       //Differential gain, helps control the rotation speed
const float yawGain = 0;        //Yaw differential gain
/*** * * * DRONE SETTINGS * * * ***/

//Radio vars
RF24 radio(25, 10); //Sets CE and CSN pins of the radio
byte addresses[][6] = {"C", "D"};

//MPU control/status vars
MPU6050 mpu;
uint8_t devStatus;      //Return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; //FIFO storage buffer
//MPU orientation/motion vars
Quaternion q;                     //Quaternion container
VectorFloat gravity;              //Gravity vector
float ypr[3];                     //Array containing yaw, pitch and roll
const float MPUmult = 180 / M_PI; //Constant used to convert from MPU6050 data to degrees
//Time vars
unsigned long startTime;            //Start time of flight (in milliseconds)
unsigned long loopTime[rRateCount]; //Timestamp of last few loops (in milliseconds)
int loopTimeCounter = 0;            //Index of loopTime which was last used
//SMA
float rpSMA[rRateCount][3]; //Simple moving average of roll and pitch
int SMAcounter = 0;         //Index of rpSMA which was last used
//Standby
int standbyStatus = 0; //0: not on standby, 1: starting standby, 2: on standby
unsigned long standbyStartTime;
unsigned long standbyOffset = 0;
bool standbyLights = true;
unsigned long lightChangeTime = 0;

//Input vars
char Data[7];            //Input data from radio
int xyzr[4] = {0,0,0,0}; //Joystick inputs for x, y, z and rotation(yaw) (from -127 to 127)
float potPercent = 0;    //Percentage of the controllers potentiometer, used as a trim
bool light = false;

//Rotation vars
float currentAngle[3];   //Current angle of roll and pitch (in degrees)
float rpDiff[2] = {0,0}; //Difference in roll & pitch from the wanted angle
float PIDchange[3][2];   //The change from the P, I and D values that will be applied to the roll & pitch; PIDchange[P/I/D][roll/pitch]
float rotTime;           //Time (in milliseconds) over which the rotation rate is calculated
float rRate[3];          //Rotation rate (degrees per millisecond) of roll, pitch and yaw
float Isum[2];           //The sum of the difference in angles used to calculate the integral change
float yawChange;         //The percentage change in motor power to control yaw

//Hardware vars
const int lightPin = 5;
const int chipSelect = SS;
const int motors[4] = {23, 22, 21, 20}; //FL, FR, BL, BR
float initialPower;                     //The base motor power percentage at the start of each loop
float motorPower[4];                    //Percentage; FL, FR, BL, BR
Servo ESC[4];                           //0-180

//Log vars
SdFs sd;
FsFile logFile;
const int logFileSize = 32 * 1024 * 1024; //32 million bytes of pre allocated data (used up in around 10 mins)
int radioReceived = 0;                    //The number of loops done since the last radio signal
const int maxLogLoopCounter = 10;         //How often the log is written to the SD card
int logLoopCounter = 0;

//Functions

int toInt(float f){
  return int(f + .5);
}

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

void applyChange(int axis, int pA, int pB, int nA, int nB){
  float change = PIDchange[0][axis] + PIDchange[1][axis] + PIDchange[2][axis];

  if (abs(change) > .00277) {
    motorPower[pA] += change;
    motorPower[pB] += change;

    motorPower[nA] -= change - .00277;
    motorPower[nB] -= change - .00277;
  }
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

void checkSD(bool condition) {
  if (!condition) {
    for (int i=0; i<4; i++){
      ESC[i].attach(motors[i], 1000, 2000);
      delay(1);
      ESC[i].write(0);
    }
    
    for (;;) {
      blink(1500);
    }
  }
}

void standby() {
  if (standbyStatus == 1) {
    standbyStatus = 2;
    
    //Turn off all motors
    for (int i=0; i<4; i++) {
      ESC[i].write(0);
    }
  
    //Log the drone going into standby
    logFile.println("\n--- on standby ---\n");
    logFile.flush();

    standbyStartTime = micros();
  }

  radioReceived = 0;
  //Blink lights
  if (millis()-lightChangeTime > 750) {
    lightChangeTime = millis();
    standbyLights = !standbyLights;
  }
  digitalWrite(lightPin, standbyLights);
}

void ABORT(){ //This is also used to turn off all the motors after landing
  for (int i=0; i<4; i++){
    ESC[i].write(0);
  }
  
  logFile.flush();
  logFile.truncate();
  logFile.close();
  
  for (;;){
    for (int i=0; i<4; i++){
      ESC[i].write(0);
    }
    blink(111);
    blink(1234);
  }
}


void setup(){
  pinMode(lightPin, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);

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
  logFile.print("maxZdiff"); logFile.print(logArray(maxZdiff, 2, 2));
  logFile.print(",potMaxDiff,"); logFile.print(potMaxDiff);
  logFile.print(",maxAngle,"); logFile.print(127/maxAngle);
  logFile.print(",yawControl,"); logFile.print(maxAngle/127, 2);
  logFile.println("\nOffsets");
  logFile.print("motorOffset"); logFile.print(logArray(motorOffset, 4, 3));
  logFile.print(",rpOffset"); logFile.print(logArray(rpOffset, 2, 2));
  logFile.print(",defaultZ,"); logFile.print(defaultZ);
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
  digitalWrite(lightPin, HIGH);
  for (int i=0; i<4; i++){
    ESC[i].attach(motors[i], 1000, 2000);
    delay(1);
    ESC[i].write(0);
  }
  delay(4000);
  digitalWrite(lightPin, LOW);
  delay(1000);
  for (int i=0; i<4; i++){
    digitalWrite(lightPin, HIGH);
    ESC[i].write(10);
    delay(500);
    ESC[i].write(0);
    digitalWrite(lightPin, LOW);
    delay(200);
  }

  //Set up MPU 6050
  digitalWrite(lightPin, HIGH);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(-5054);
  mpu.setYAccelOffset(297);
  mpu.setZAccelOffset(1024);
  mpu.setXGyroOffset(122);
  mpu.setYGyroOffset(-101);
  mpu.setZGyroOffset(22);
  
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    //Let the MPU run for a bit to allow for values to stabilise
    for (int i=0; i<1000; i++) {
      delay(4);
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
    }
  } else {
    logFile.print("MPU6050 error: ");
    logFile.println(devStatus);
    ABORT();
  }
  digitalWrite(lightPin, LOW);
  
  //Set up communication
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(124);
  
  //Open pipe
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  
  //Ready drone
  for (int i=0; i<10; i++){
    radio.write(0b01010101, 1);
    delay(5);
  }
  radio.startListening();

  //Fill rpSMA
  for (int i=0; i<rRateCount; i++) {
    rpSMA[i][0] = (-ypr[2] * MPUmult) + rpOffset[0];
    rpSMA[i][1] = (-ypr[1] * MPUmult) + rpOffset[1];
    rpSMA[i][2] =   ypr[0] * MPUmult;
  }

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
  /* Recieve input data */
  radioReceived++;
  if (radio.available()) {
    //Lower the counter for loss of connection after receiving radio
    radioReceived = max(radioReceived-25, 0);
    //Get data from controller
    while (radio.available()) {
      radio.read(&Data, sizeof(char[7]));
    }
    
    //Abort button
    if (bitRead(Data[6], 0)) {
      ABORT();
    }
    
    //Get analog info from packet
    for (int i=0; i<4; i++) {
      xyzr[i] = Data[i] - 127;
      if (abs(xyzr[i]) < 5) { //Joystick deadzone
       xyzr[i] = 0;
      }
    }
    xyzr[1] = -xyzr[1]; //Correct pitch
    potPercent = Data[4]/255.0; //Put between 0-1
    //Get binary info from packet
    light = bitRead(Data[6], 2);

    if (bitRead(Data[6], 1) and standbyStatus == 0) {
      standbyStatus = 1;
    } else if (!bitRead(Data[6], 1) and standbyStatus == 2) {
      standbyOffset += micros() - standbyStartTime;
      standbyStatus = 0;
    }
  }
  
  if (standbyStatus > 0) {
    standby();
  } else {
    //Abort after connection lost for 450 cycles (nominally just under a second)
    if (radioReceived > 450) {
      ABORT();
    }


    /* Get current angle */
    //Get current angle and acceleration
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    currentAngle[0] = (-ypr[2] * MPUmult) + rpOffset[0]; //roll
    currentAngle[1] = (-ypr[1] * MPUmult) + rpOffset[1]; //pitch
    currentAngle[2] =   ypr[0] * MPUmult;                //yaw
    //Abort if the drone is flipping, is not needed but is a nice failsafe when testing new features
    if (abs(currentAngle[0]) > 120 or abs(currentAngle[1]) > 120) {
      ABORT();
    }
    
    //Update SMA values
    SMAcounter++;
    SMAcounter %= rRateCount;
    for (int i=0; i<3; i++) {
      rpSMA[SMAcounter][i] = currentAngle[i];
    }
    
    //Get loop time
    loopTimeCounter++;
    loopTimeCounter %= rRateCount;
    loopTime[loopTimeCounter] = micros()-standbyOffset;
    //Make sure the loop is at least 2 milliseconds long
    while (getLoopTime(1) < 2) {
      delayMicroseconds(2);
      loopTime[loopTimeCounter] = micros()-standbyOffset;
    }
    //Calculate rotation rate
    rotTime = getLoopTime(-1);
    for (int i=0; i<3; i++) {
      rRate[i] = (rpSMA[SMAcounter][i] - rpSMA[(SMAcounter+1) % rRateCount][i]) / rotTime;
    }


    /* Set motor speeds */
    //Set initial motor power
    if (xyzr[2] < 0) {
      initialPower = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[0] * xyzr[2]/127);
    } else {
      initialPower = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[1] * xyzr[2]/127);
    }
    for (int i=0; i<4; i++){
      motorPower[i] = initialPower;
    }
  
    //Calculate the change in motor power per axis
    for (int i=0; i<2; i++) {
      //Get difference between wanted and current angle
      rpDiff[i] = (-xyzr[i]/maxAngle) - currentAngle[i];
    
      //Get proportional change
      PIDchange[0][i] = Pgain * rpDiff[i];
    
      //Get integral change
      Isum[i] += rpDiff[i] * getLoopTime(1);
      PIDchange[1][i] = Igain * Isum[i];
      
      //Get derivative change
      PIDchange[2][i] = Dgain *  rRate[i];
    }
    
    //Apply the calculated roll and pitch change
    applyChange(0, 0,2, 1,3);
    applyChange(1, 0,1, 2,3);

    //Yaw control
    if (xyzr[3] == 0) {
      yawChange = rRate[2] * yawGain; //Stabilise yaw rotation
    } else {
      yawChange = xyzr[3] * yawControl; //Joystick control
    }
    //Apply yaw change
    motorPower[0] += yawChange;
    motorPower[3] += yawChange;
    motorPower[1] -= yawChange;
    motorPower[2] -= yawChange;


    /* Apply input to hardware */
    digitalWrite(lightPin, light);
    for (int i=0; i<4; i++){
      motorPower[i] += motorOffset[i]/100;
      motorPower[i] = min(max(0, motorPower[i]*1000), 1000); //Limit values to 0 - 1000
      ESC[i].writeMicroseconds(1000 + toInt(motorPower[i])); //Round motor power and apply it to the ESC
    }
  
    /* Log flight info */
    logFile.print((micros()-startTime-standbyOffset) / 1000);
    logFile.print("," + String(getLoopTime(1), 1));
    logFile.print(logArray(xyzr, 4));
    logFile.print("," + String(potPercent*100));
    logFile.print(logArray(currentAngle, 2, 2));
    logFile.print(logArray(PIDchange[0], 2, 3));
    logFile.print(logArray(PIDchange[1], 2, 3));
    logFile.print(logArray(PIDchange[2], 2, 3));
    logFile.print(logArray(motorPower, 4, 0));
    logFile.print("," + String(radioReceived));
    logFile.println("," + String(currentAngle[2], 1));
    
    logLoopCounter++;
    if (logLoopCounter == maxLogLoopCounter) {
      logFile.flush();
      logLoopCounter = 0;
    }
  }
}
