#include <RF24.h> //https://github.com/nRF24/RF24
#include <Servo.h>

///This is a simple program that will does have any auto levelling forming the bases of later versions

///The drone controlled via the potentiometer setting the strength of the motor
///The joystick sets which motors will get the most power
///If the joystick is pressed down the drone will rotate on the spot

RF24 radio(25, 26);
byte addresses[][6] = {"C","D"};

//Input vars
char Data[6];
int xyzr[4] = {0, 0, 0, 0};
float potPercentage;
bool light = false;

//Hardware vars
const int lightPin = 3;
const int motors[4] = {20, 21, 22, 23}; //FL, FR, BL, BR
Servo ESC[4]; //65-160
float motorPower[4] = {0, 0, 0, 0}; //0-1; FL, FR, BL, BR
//Hardware settings
const float maxChange = .1;
const float zStep = .25/127.0;

//Functions

int toInt(float f){
  return int(f + 0.5);
}

void calculateAxis(int axis, int addA,int addB, int subA,int subB){
  float change = maxChange * (axis/127.0); //get the percentage change
  
  motorPower[addA] += change;
  motorPower[addB] += change;
  motorPower[subA] -= change;
  motorPower[subB] -= change;
}



void setup(){
  Serial.begin(19200);
  
  pinMode(lightPin, OUTPUT);
  //Set up motors
  for (int i=0; i<4; i++){
    ESC[i].attach(motors[i]);
    ESC[i].write(0);
    ESC[i].write(5);
  }
  digitalWrite(lightPin, HIGH);
  delay(4500);
  digitalWrite(lightPin, LOW);
  delay(500);
  for (int i=0; i<4; i++){
    ESC[i].write(70);
    delay(250);
    ESC[i].write(0);
    delay(750);
  }
  
  //Set up communication
  delay(2500);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(124);

  //Open pipe
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  //Ready drone
  for (int i=0; i<4; i++){
    radio.write(0b01010101, sizeof(byte));
    delay(5);
  }
  radio.startListening();
}

void loop(){
  if (radio.available()) {
    /* Recieve input data */
    //Get data from controller
    while (radio.available()) {
      radio.read(&Data, sizeof(char[6]));
    }

    //Get joystick info from packet
    for (int i=0; i<4; i++) {
      xyzr[i] = Data[i] - 127;
      if (abs(xyzr[i]) < 4) { //joystick deadzone
       xyzr[i] = 0;
      }
    }
    
    //Get other info from packet
    potPercentage = Data[4]/255.0; //Put between 0-1
    light = bitRead(Data[5], 2);
    
    /* Set motor speeds */
    motorPower[0] = motorPower[1] = motorPower[2] = motorPower[3] = potPercentage; //Reset the motor power
    
    //X axis control
    calculateAxis(xyzr[0], 0,1, 2,3);
    //Y axis control
    calculateAxis(xyzr[1], 0,2, 1,3);
    //Rotation control
    calculateAxis(xyzr[3], 0,3, 1,2);
    
    //Z axis control
    for (int i=0; i<4; i++){
      motorPower[i] += xyzr[2] * zStep;
      motorPower[i] = max(min(motorPower[i], 1), 0);
    }
    
    /* Apply input to hardware */
    digitalWrite(lightPin, light);
    for (int i=0; i<4; i++){
      Serial.print(motorPower[i]);Serial.print(",\t");///Debug output
      ESC[i].write(map(toInt(motorPower[i]*100), 0, 100, 65, 160));
    }
    
    ///Debug output
    Serial.print("\t");
    for (int i=0; i<4; i++){
      Serial.print("\t");
      Serial.print(xyzr[i]);
    }
    Serial.print("\t\t");
    Serial.println(potPercentage);
  }
}
