#include <Arduino.h>
#include <TinyMPU6050.h>
#include <RF24.h> //https://github.com/nRF24/RF24
#include <Servo.h>
#include <SD.h>
#include <SPI.h>

//Hardware setup
RF24 radio(25, 26);
byte addresses[][6] = {"C","D"};
MPU6050 mpu (Wire);

//Input vars
char Data[6];
int xyzr[4] = {0, 0, 0, 0};
float potPercent = 0.0;
bool light = false;

//Pos vars
const float maxZdiff = 0.14;
const float defaultZ = 0.5;
const float maxAngle = 127.0/30.0;
float currentAngle[2] = {0, 0};
float xyDiff[2] = {0, 0};
const float maxDiff = .05;
float xyChange[2] = {0, 0};

//Hardware vars
const int lightPin = 3;
const int motors[4] = {23, 22, 21, 20}; //FL, FR, BL, BR
Servo ESC[4]; //35-135
float motorPower[4] = {0, 0, 0, 0}; //0-1; FL, FR, BL, BR
const int chipSelect = BUILTIN_SDCARD;

//Log vars
File logFile;
unsigned long timerOffset;
String logStr;
int ESCvals[4];
int radioReceived = 0;

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

void applyChange(float change, int pA, int pB, int nA, int nB){
  motorPower[pA] += change;
  motorPower[pB] += change;
  
  motorPower[nA] -= change;
  motorPower[nB] -= change;
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

  //Set up SD card
  if (!SD.begin(chipSelect)) {
    for (;;) {
      blink(1500);
    }
  }
  SD.remove("log.csv"); //Clear the last log

  //Set up motors
  for (int i=0; i<4; i++){
    ESC[i].attach(motors[i], 1000, 2000);
    delay(1);
    ESC[i].write(0);
  }
  digitalWrite(lightPin, HIGH);
  delay(4000);
  digitalWrite(lightPin, LOW);
  delay(1000);
  for (int i=0; i<4; i++){
    digitalWrite(lightPin, HIGH);
    ESC[i].write(40);
    delay(800);
    ESC[i].write(0);
    digitalWrite(lightPin, LOW);
    delay(200);
  }
  
  digitalWrite(lightPin, HIGH);
  mpu.Initialize();
  mpu.Calibrate();
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
  timerOffset = millis();
}

void loop(){
  mpu.Execute();
  
  /* Recieve input data */
  radioReceived++;
  if (radio.available()) {
    radioReceived = 0;
    //Get data from controller
    while (radio.available()) {
      radio.read(&Data, sizeof(char[6]));
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
    light = bitRead(Data[5], 2);

    //abort
    if (bitRead(Data[5], 0)) {
      ABORT();
    }
  }
  //Abort after connection lost for 100 cycles
  if (radioReceived > 100) {
    ABORT();
  }
  
  /* Set motor speeds */
  //Set initial speed
  for (int i=0; i<4; i++){
    motorPower[i] = potPercent/2 + (maxZdiff * xyzr[2]/127);
  }

  //Get current angle
  currentAngle[0] = mpu.GetAngX();
  currentAngle[1] = mpu.GetAngY();
  //Abort if the drone is flipping, is not needed but is a nice failsafe
  if (abs(currentAngle[0]) > 111 or abs(currentAngle[1]) > 111) {
    ABORT();
  }
  
  for (int i=0; i<2; i++) {
    //Get difference between wanted and current angle
    xyDiff[i] = xyzr[i]/maxAngle - currentAngle[i];
  
    //Get percentage change for the X and Y axis
    xyChange[i] = xyDiff[i]*maxDiff/90;
  }
  
  //Apply multipliers
  applyChange(-xyChange[0], 0,2, 1,3);
  applyChange( xyChange[1], 0,1, 2,3);
  //applyMult(maxDiff*xyzr[3]/127, 0,3, 1,2);
  
  /* Apply input to hardware */
  digitalWrite(lightPin, light);
  for (int i=0; i<4; i++){
    motorPower[i] = min(max(0, motorPower[i]*100), 100); //Limit values
    ESCvals[i] = motorPower[i] + 35; //Convert from ESC value to motor power
    ESC[i].write(ESCvals[i]);
  }

  /* Log flight info */
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logStr = String(millis()-timerOffset);
    logStr += logArray(xyzr, 4);
    logStr += "|" + String(potPercent*100);
    logStr += logArray(currentAngle, 2);
    logStr += logArray(motorPower, 4);
    logStr += logArray(ESCvals, 4);
    logStr += "|" + String(light ? 10 : 0);
    logStr += "|" + String(radioReceived);

    logFile.println(logStr);
    logFile.close();
  }
}
