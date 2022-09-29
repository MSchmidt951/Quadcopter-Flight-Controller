#ifndef __Logger_H__
#define __Logger_H__

//Import libraries
#include <SdFat.h>
#include <ArduinoJson.h>

extern void blink(int);
extern const int loopRate;


/* Settings */
const int logBufferLen = 24; //The max length of the buffer in words (4 bytes each)
const int logFileSize = 10*60 * logBufferLen*4 * loopRate; //Reserve enough space for 10 mins
const int maxLoopTimerSections = 8; //The maximum times calcSectionTime can be called per loop
/* Settings */

const struct TypeID {
  const uint8_t uint8   =   8;
  const uint8_t uint16  =  16;
  const uint8_t int16   = 116;
  const uint8_t uint32  =  32;
  const uint8_t float16 = 216;
  const uint8_t float32 = 232;
} typeID;

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
    template <typename T> void logData(T data, const uint8_t data_typeID) {
      //Convert data
      union unionBuffer {
        T in;
        int16_t float16;
        word w;
      } u;
      if (data_typeID == typeID.float16) {
        u.float16 = (int16_t)(data*10);
      } else {
        u.in = data;
      }

      //Store data type
      if (firstLog) {
        varID[varCount] = data_typeID;
        varCount++;
      }

      //Add data to buffer
      for (int i=0; i<data_typeID%100; i++) {
        if (bitRead(u.w, i)) {
          bitSet(buf[bufOffset/32], i);
        }
        bufOffset++;
      }
    }

  private:
    void checkSD(bool condition);
    void binToStr();

    //File variables
    SdFs sd;
    FsFile logFile;
    FsFile logFileBin;
    StaticJsonDocument<512> sdSettings;
    //Section timer variables
    unsigned long loopTimings[maxLoopTimerSections];
    uint8_t timerIndex;
    //Buffer variables
    word buf[logBufferLen];
    uint16_t bufOffset;
    bool firstLog = true;
    uint8_t varID[32];
    uint8_t varCount;
};
#endif
