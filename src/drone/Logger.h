#ifndef __Logger_H__
#define __Logger_H__

//Import libraries
#include <SdFat.h>

extern void blink(int);


/* Settings */
const int logFileSize = 32 * 1024 * 1024; //32 million bytes of pre allocated data (used up in around 10 mins)
const int maxLogLoopCounter = 10;         //How often the log is written to the SD card
const int maxLoopTimerSections = 10;      //The maximum times calcSectionTime can be called per loop
/* Settings */


class Logger {
  public:
    void init();
    void logSetting(String name, int data, bool seperator=true);
    void logSetting(String name, float data, int decimals, bool seperator=true);
    void logSetting(String name, int *arr, int len, bool seperator=true);
    void logSetting(String name, float *arr, int len, int decimals, bool seperator=true);
    void logString(String s);
    void logData(int data, bool seperator=true);
    void logData(float data, int decimals, bool seperator=true);
    void logArray(int *arr, int len);
    void logArray(float *arr, int len, int decimals);
    void write(bool mustWrite=false);
    void calcSectionTime(); //Calculates how long a section of the main loop takes
    void closeFile();

  private:
    void checkSD(bool condition);
    
    SdFs sd;
    FsFile logFile;
    unsigned long loopTimings[maxLoopTimerSections];
    int logLoopCounter;
    int timerIndex;
};
#endif
