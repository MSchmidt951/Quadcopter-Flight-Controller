#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>

//Radio vars
RF24 radio(25, 10);
byte addresses[][6] = {"C","D"};

//MPU control/status vars
MPU6050 mpu;
uint8_t devStatus;      //Return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; //FIFO storage buffer
//MPU orientation/motion vars
Quaternion q;                     //Quaternion container
VectorFloat gravity;              //Gravity vector
float ypr[3];                     //Array containing yaw, pitch and roll
const float MPUmult = 180 / M_PI; //Constant used to convert from MPU6050 data to degrees
const float xyOffset[2] = {0, 0};
//SMA
const int SMA_LEN = 40;
float SMAypr[SMA_LEN][3];
int SMAcounter = 0;


//Input vars
char Data[7];
int xyzr[4] = {0, 0, 0, 0};
float potPercent = 0.0;
const float potMaxDiff = .2;
bool light = false;

//Pos vars
const float defaultZ = 0.4;
const float maxAngle = 127.0/25.0;
float currentAngle[2] = {0, 0};
float xyDiff[2] = {0, 0}; //Difference in x & y from the wanted angle
float xyChange[2] = {0, 0}; //The change that will be applied to the x & y axis
const float maxZdiff[2] = {.12, 0.2}; //% z diff for joystick down &  up, respectively
const float diffMultiplier = .05; //% diff per motor at 45 degrees
const float SMAmult = 4;
const float maxDiff = .03;

const float motorOffset[4] = {0, 0, 0, 0};

//Hardware vars
const int lightPin = 5;
const int motors[4] = {23, 22, 21, 20}; //FL, FR, BL, BR
Servo ESC[4]; //0-180
float motorPower[4] = {0, 0, 0, 0}; //0-1; FL, FR, BL, BR
const int chipSelect = BUILTIN_SDCARD;

//Log vars
File logFile;
unsigned long startTime;
String logStr;
int ESCvals[4];
int radioReceived = 0; //The number of loops done since the last radio signal

//Functions

int toInt(float f){
  return int(f + 0.5);
}

void blink(int d){
  digitalWrite(lightPin, HIGH);
  delay(d);
  digitalWrite(lightPin, LOW);
  delay(d);
}

float getSMA(int index) {
  float SMA = 0;
  for (int i=0; i<SMA_LEN; i++) {
    SMA += SMAypr[i][index];
  }
  return SMA/SMA_LEN;
}

void applyChange(float change, int pA, int pB, int nA, int nB){
  motorPower[pA] += min(change, maxDiff);
  motorPower[pB] += min(change, maxDiff);
  
  motorPower[nA] -= min(change*.5, maxDiff);
  motorPower[nB] -= min(change*.5, maxDiff);
}

String logArray(int *arr, int len){
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "|" + String(arr[i]);
  }
  return arrayString;
}
String logArray(float *arr, int len){
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "|" + String(arr[i]);
  }
  return arrayString;
}

void Standby() {
  //Turn off all motors
  for (int i=0; i<4; i++) {
    ESC[i].write(0);
  }

  //Log the drone going into standby
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Standby");
    logFile.close();
  }

  //Wait until the drone is not on standby
  boolean Exit = false;
  while (!Exit) {
    if (radio.available()) {
      //Get data
      while (radio.available()) {
        radio.read(&Data, sizeof(char[7]));
      }
      
      if (bitRead(Data[6], 1)) {
        Exit = true;
      }
    }
    delay(3);
  }

  //Turn the motors back on at a low power
  for (int i=0; i<4; i++) {
    ESC[i].write(38);
  }
  delay(222); //Allow the motors to start up
}

void ABORT(){ //This can also be used to turn off all the motors after landing
  for (;;){
    for (int i=0; i<4; i++){
      ESC[i].write(0);
    }
    blink(111);
    for (int i=0; i<4; i++){
      ESC[i].write(0);
    }
    blink(1234);
  }
}


void setup(){
  pinMode(lightPin, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);

  //Set up SD card
  if (!SD.begin(chipSelect)) {
    for (;;) {
      blink(1500);
    }
  }
  SD.remove("log.csv"); //Clear the last log
  logFile = SD.open("log.csv", FILE_WRITE);
  logFile.print("diffMultiplier:|");
  logFile.print(diffMultiplier);
  logFile.print("|maxDiff:|");
  logFile.print(maxDiff);
  logFile.print("|xyOffset:");
  logFile.print(logArray(xyOffset, 2));
  logFile.print("|maxZdiff:");
  logFile.print(logArray(maxZdiff, 2));
  logFile.print("|motorOffset:");
  logFile.print(logArray(motorOffset, 2));
  logFile.print("|SMAmult:|");
  logFile.print(SMAmult);
  logFile.println("|changeLog:|re-calibrated ESCs, not using SMA");
  logFile.println("Time|X in|Y in|Z in|R in|Pot|x|y|FL|FR|BL|BR|FL ESC|FR ESC|BL ESC|BR ESC|light|radio");
  logFile.close();

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
    ESC[i].write(20);
    delay(500);
    ESC[i].write(0);
    digitalWrite(lightPin, LOW);
    delay(200);
  }
  
  //Set upu MPU 6050
  digitalWrite(lightPin, HIGH);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(41);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(-13);
  mpu.setXAccelOffset(-5060);
  mpu.setYAccelOffset(267);
  mpu.setZAccelOffset(932);
  
  if (devStatus == 0) {
    ///mpu.CalibrateAccel(6); //Only use if take off from flat surface
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    for (int i=0; i<800; i++) {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
      delay(10);
    }
  } else {
    logFile = SD.open("log.csv", FILE_WRITE);
    logFile.print("MPU6050 error: ");
    logFile.println(devStatus);
    logFile.close();
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

  //Fill SMAypr
  for (int i=0; i<SMA_LEN; i++) {
    for (int j=0; j<3; j++) {
      SMAypr[i][j] = ypr[j];
    }
  }

  for (int i=0; i<5; i++){
    blink(150);
  }
  startTime = millis();
}

void loop(){
  /* Recieve input data */
  radioReceived++;
  if (radio.available()) {
    //Lower the counter for loss of connection after receiving radio
    radioReceived = max(radioReceived-11, 0);
    //Get data from controller
    while (radio.available()) {
      radio.read(&Data, sizeof(char[7]));
    }

    //Get analog info from packet
    for (int i=0; i<4; i++) {
      xyzr[i] = Data[i] - 127;
      if (abs(xyzr[i]) < 4) { //joystick deadzone
       xyzr[i] = 0;
      }
    }
    potPercent = Data[4]/255.0; //Put between 0-1
    //Get binary info from packet

    light = bitRead(Data[6], 2);
    //Abort button
    if (bitRead(Data[6], 0)) {
      ABORT();
    }
    
    if (bitRead(Data[6], 1)) {
      //Standby(); //Doesnt work properly! Needs testing
    }
  }
  //Abort after connection lost for 175 cycles (nominally around a second)
  if (radioReceived > 175) {
    ABORT();
  }
  
  /* Set motor speeds */
  //Set initial speed
  for (int i=0; i<4; i++){
    if (xyzr[2] < 0) {
      motorPower[i] = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[0] * xyzr[2]/127);
    } else {
      motorPower[i] = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[1] * xyzr[2]/127);
    }
  }

  //Get current angle
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  currentAngle[0] = (-ypr[2] * MPUmult) + xyOffset[0]; //x
  currentAngle[1] = (-ypr[1] * MPUmult) + xyOffset[1]; //y
  //Abort if the drone is flipping, is not needed but is a nice failsafe
  if (abs(currentAngle[0]) > 111 or abs(currentAngle[1]) > 111) {
    ABORT();
  }
  
  //Calculate SMA
  for (int i=0; i<3; i++) {
    SMAypr[SMAcounter][i] = ypr[i];
  }
  SMAcounter++;
  SMAcounter %= SMA_LEN;

  //Calculate the change in motor power per axis
  for (int i=0; i<2; i++) {
    //Get difference between wanted and current angle
    xyDiff[i] = (-xyzr[i]/maxAngle) - currentAngle[i];
    xyDiff[i] -= SMAmult * (((getSMA(2-i)-ypr[2-i]) * MPUmult) + xyOffset[i]);
  
    //Get multipliter for the X and Y axis
    xyChange[i] = xyDiff[i]*diffMultiplier/45;
  }
  //Apply calculated change
  applyChange(xyChange[0], 0,2, 1,3);
  applyChange(xyChange[1], 0,1, 2,3);
  //applyMult(diffMultiplier*xyzr[3]/127, 0,3, 1,2);
  
  /* Apply input to hardware */
  digitalWrite(lightPin, light);
  for (int i=0; i<4; i++){
    motorPower[i] += motorOffset[i];
    motorPower[i] = min(max(0, motorPower[i]*100), 100); //Limit values to 0 - 180
    ESCvals[i] = toInt(motorPower[i] * 1.8); //Convert from ESC value to motor power
    ESC[i].write(ESCvals[i]);
  }

  /* Log flight info */
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logStr = String(millis()-startTime);
    logStr += logArray(xyzr, 4);
    logStr += "|" + String(potPercent*100);
    logStr += logArray(currentAngle, 2);
    logStr += logArray(motorPower, 4);
    logStr += logArray(ESCvals, 4);
    logStr += "|" + String(light ? 10 : 0);
    logStr += "|" + String(radioReceived);
    logStr += "|" + String((-getSMA(2) * MPUmult) + xyOffset[0]);
    logStr += "|" + String((((getSMA(2)-ypr[2]) * MPUmult) + xyOffset[0]) * -diffMultiplier/4.5);

    logFile.println(logStr);
    logFile.close();
  }
}
