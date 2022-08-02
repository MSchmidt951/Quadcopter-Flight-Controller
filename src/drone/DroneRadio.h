#ifndef __DroneRadio_H__
#define __DroneRadio_H__

//Import libraries
#include <RF24Teensy.h>

extern int xyzr[4];
extern float potPercent;
extern bool light;
extern bool standbyButton;

extern void ABORT();


class DroneRadio {
  public:
    void init();
    void getInput();
    void checkSignal();
    int radioReceived = 0; //The number of loops done since the last radio signal

  private:
    RF24 radio{25, 10}; //Sets CE and CSN pins of the radio
    byte addresses[2][6] = {"C", "D"};
    char data[7]; //Raw input data
};
#endif
