#ifndef __DroneRadio_H__
#define __DroneRadio_H__

//Import libraries
#include <RF24.h>

extern int xyzr[4];
extern float potPercent;
extern bool light;
extern bool standbyButton;

extern void ABORT();

/**
 * @class DroneRadio
 * @brief Class to control radio
 */
class DroneRadio {
  public:
    /** Initialise the radio. */
    void init();
    /** Recieves the input from the controller, if any was received. */
    void getInput();
    /** Checks the radio signal is being recieved at a fast enough rate.
     *  
     *  @param[in] loopTime length of time to complete previous loop (milliseconds)
     *  @param[in] currentTime uptime of device in milliseconds (excluding standby)
     */
    void checkSignal(unsigned long loopTime, unsigned long currentTime);

    ///Keeps track of loss of communication
    int timer;

  private:
    ///Sets CE and CSN pins of the radio
    RF24 radio{25, 10};
    ///Addresses of the controller and device
    byte addresses[2][6] = {"C", "D"};
    ///Raw input data
    char data[7];
    ///Minimum wanted rate of the radio (Hz)
    const int minRadioRate = 50;
    ///Maximum acceptable delay of the radio (μs)
    const int maxRadioDelay = 1000000/minRadioRate;
    ///Whether the radio signal has been received this loop
    bool radioReceived = false;
    ///Time since the last radio signal (μs)
    unsigned long lastRadioTime = 0;
};
#endif
