#ifndef __Logger_H__
#define __Logger_H__

//Import libraries
#include <SdFat.h>
#include <ArduinoJson.h>

extern void blink(int);


struct logStruct {
  unsigned long time;
  float roll;
  float pitch;
  int16_t Pp; int16_t Pr;
  int16_t Ip; int16_t Ir;
  int16_t Dp; int16_t Dr;
  uint16_t radio;
  int16_t yaw;
  char rollInput;
  char pitchInput;
  char verticalInput;
  char pot;

  void getBinary(uint32_t buf[]);
};

/* Settings */
const int logFileSize = 10 * sizeof(logStruct)*4000*60; //Reserve enough space for 10 mins
const int maxLoopTimerSections = 8; //The maximum times calcSectionTime can be called per loop
/* Settings */


class Logger {
  public:
    void init();
    void logSetting(String name, int data, bool seperator=true);
    void logSetting(String name, float data, int decimals, bool seperator=true);
    void logSetting(String name, const float *arr, int len, int decimals, bool seperator=true);
    void logArray(const float *arr, int len, int decimals);
    void logString(String s);
    void write();
    void calcSectionTime(); //Calculates how long a section of the main loop takes
    void storeSectionTime(); //Stores the min, max and average time of each section of the main loop
    void closeFile();

    template <typename T> void loadSetting(String name, T &var){
      if (sdSettings.containsKey(name)) {
        var = sdSettings[name];
      }
    }
    template <typename T> void loadSetting(String name, T *var, int len){
      if (sdSettings.containsKey(name)) {
        for (int i=0; i<len; i++) {
          var[i] = sdSettings[name][i];
        }
      }
    }

    logStruct currentLog;

  private:
    void checkSD(bool condition);
    void binToStr();

    uint32_t buf[sizeof(logStruct)/4];
    //File variables
    SdFs sd;
    FsFile logFile;
    FsFile logFileBin;
    StaticJsonDocument<512> sdSettings;
    //Section timer variables
    unsigned long loopTimings[maxLoopTimerSections];
    int timerIndex;
};
#endif
