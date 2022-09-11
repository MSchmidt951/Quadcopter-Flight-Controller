#ifndef __DroneRadio_H__
#define __DroneRadio_H__

//Import libraries
#include <RF24.h>

extern int xyzr[4];
extern float potPercent;
extern bool light;
extern bool standbyButton;

extern void ABORT();


class DroneRadio {
  public:
    void init();
    void getInput();
    void checkSignal(unsigned long loopTime, unsigned long currentTime);
    
    int timer; //Keeps track of loss of communication

  private:
    RF24 radio{25, 10};                //Sets CE and CSN pins of the radio
    byte addresses[2][6] = {"C", "D"}; //Addresses of the controller and device (drone)
    char data[7];                      //Raw input data
    const int minRadioRate = 50;                    //Minimum wanted rate of the radio (Hz)
    const int maxRadioDelay = 1000000/minRadioRate; //Maximum acceptable delay of the radio (us)
    bool radioReceived = false;                     //Whether the radio signal has been received this loop
    unsigned long lastRadioTime = 0;                //Time in microseconds since the last radio signal
};
#endif
