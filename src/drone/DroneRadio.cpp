#include "DroneRadio.h"

void DroneRadio::init() {
  radio.begin();
  radio.setRadiation(RF24_PA_MAX, RF24_2MBPS);
  radio.setChannel(124);
  radio.setPayloadSize(7);
  
  //Open pipe
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  
  //Ready drone
  data[0] = 0b01010101;
  for (int i=0; i<10; i++){
    radio.write(&data, 1);
    delay(5);
  }
  radio.startListening();
}

void DroneRadio::getInput() {
  if (radio.available()) {
    //Lower the counter for loss of connection after receiving radio
    radioReceived = true;
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

void DroneRadio::checkSignal(unsigned long loopTime, unsigned long currentTime) {
  //On the first loop set lastRadioTime to the current time to avoid a large spike
  if (lastRadioTime == 0) {
    lastRadioTime = currentTime;
  }

  //Add the loop time in microseconds to the timer
  timer += loopTime;

  if (radioReceived) {
    //No penalty if delay less than maximum
    if (currentTime - lastRadioTime <= maxRadioDelay) {
      timer = max(timer-maxRadioDelay, 0);
    }

    lastRadioTime = currentTime;
    radioReceived = false;
  }

  //Abort after one second of loss of communication
  if (timer + (currentTime - lastRadioTime) >= 1000000) {
    ABORT();
  }
}
