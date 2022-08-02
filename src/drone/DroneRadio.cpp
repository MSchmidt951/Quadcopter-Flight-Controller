#include "DroneRadio.h"

void DroneRadio::init() {
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
}

void DroneRadio::getInput() {
  radioReceived++;
  if (radio.available()) {
    //Lower the counter for loss of connection after receiving radio
    radioReceived = max(radioReceived-25, 0);
    //Get data from controller
    while (radio.available()) {
      radio.read(&data, sizeof(char[7]));
    }
    
    //Abort button
    if (bitRead(data[6], 0)) {
      ABORT();
    }
    
    //Get analog info from packet
    for (int i=0; i<4; i++) {
      xyzr[i] = data[i] - 127;
      if (abs(xyzr[i]) < 5) { //Joystick deadzone
       xyzr[i] = 0;
      }
    }
    xyzr[1] = -xyzr[1]; //Correct pitch
    potPercent = data[4]/255.0; //Put between 0-1
    //Get binary info from packet
    light = bitRead(data[6], 2);

    standbyButton = bitRead(data[6], 1);
  }
}

void DroneRadio::checkSignal() {
  //Connection is good unless it is lost for 450 cycles (nominally just under a second)
  if (radioReceived > 450) {
    ABORT();
  }
}
