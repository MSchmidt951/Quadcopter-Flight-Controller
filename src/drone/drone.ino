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
const float maxZdiff = 0.15;
const float defaultZ = 0.2;
const float maxAngle = 127.0/30.0;
float xyDiff[2] = {0, 0};
const float maxDiff = .05;
float xyMult[2] = {0, 0};

//Hardware vars
const int lightPin = 3;
const int motors[4] = {20, 21, 22, 23}; //FL, FR, BL, BR
Servo ESC[4]; //65-160
float motorPower[4] = {0, 0, 0, 0}; //0-1; FL, FR, BL, BR
const int chipSelect = BUILTIN_SDCARD;

//Log vars
File logFile;
unsigned long timerOffset;
String logStr;
int ESCvals[4];
bool radioReceived;

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

void applyMult(float mult, int pA, int pB, int nA, int nB){
  motorPower[pA] *= 1+mult;
  motorPower[pB] *= 1+mult;
  
  motorPower[nA] *= 1-mult;
  motorPower[nB] *= 1-mult;
}

String logArray(int *arr, int len){
  String arrayString = " ";
  for (int i=0; i<len; i++) {
    arrayString += "|" + String(arr[i]);
  }
  return arrayString;
}
String logArray(float *arr, int len){
  String arrayString = " ";
  for (int i=0; i<len; i++) {
    arrayString += "|" + String(arr[i]);
  }
  return arrayString;
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
    ESC[i].attach(motors[i]);
    ESC[i].write(0);
    ESC[i].write(5);
  }
  digitalWrite(lightPin, HIGH);
  delay(4000);
  digitalWrite(lightPin, LOW);
  delay(1000);
  for (int i=0; i<4; i++){
    digitalWrite(lightPin, HIGH);
    ESC[i].write(70);
    delay(250);
    ESC[i].write(0);
    digitalWrite(lightPin, LOW);
    delay(750);
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
  radioReceived = false;
  if (radio.available()) {
    radioReceived = true;
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
      for (int i=0; i<4; i++){
        analogWrite(motors[i], 0);
      }
      for (;;){
        blink(111);
        blink(1234);
      }
    }
  }
  
  /* Set motor speeds */
  //Set initial speed
  for (int i=0; i<4; i++){
    motorPower[i] = potPercent/2 + (maxZdiff * xyzr[2]/127);
  }

  //Get difference between wanted and current angle
  xyDiff[1] = xyzr[0]/maxAngle - mpu.GetAngX();
  xyDiff[0] = xyzr[1]/maxAngle - mpu.GetAngY();
  
  //Get multipliter for the X and Y axis
  for (int i=0; i<2; i++){
    xyMult[i] = xyDiff[i]*maxDiff/90;
  }
  
  //Apply multipliers
  applyMult(xyMult[0], 0,1, 2,3);
  applyMult(xyMult[1], 0,2, 1,3);
  //applyMult(xyzr[3], 0,3, 1,2);
  
  /* Apply input to hardware */
  digitalWrite(lightPin, light);
  for (int i=0; i<4; i++){
    motorPower[i] = min(max(0, motorPower[i]*100), 100); //Limit values
    if (motorPower[i] <= 0.1){
      ESC[i].write(0);
    } else {
      ESCvals[i] = map(toInt(motorPower[i]), 1, 100, 65, 160);
      ESC[i].write(ESCvals[i]);
    }
  }

  /* Log flight info */
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logStr = String(millis()-timerOffset);
    logStr += logArray(xyzr, 4);
    logStr += " |" + String(potPercent*100);
    logStr += logArray(xyMult, 2);
    logStr += logArray(motorPower, 4);
    logStr += logArray(ESCvals, 4);
    logStr += " |" + String(light);
    logStr += " |" + String(radioReceived);

    logFile.println(logStr);
    logFile.close();
  }
}
