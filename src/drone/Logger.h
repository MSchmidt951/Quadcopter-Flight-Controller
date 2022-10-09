#ifndef __Logger_H__
#define __Logger_H__

//Import libraries
#include <SdFat.h>
#include <ArduinoJson.h>

extern void blink(int);
extern const int loopRate;


/* Settings */
const int logBufferLen = 24; //The max amont of buffer sections, each being 4 bytes long
const int logFileSize = 10*60 * logBufferLen*4 * loopRate; //Reserve enough space for 10 mins
const int maxLoopTimerSections = 8; //The maximum times calcSectionTime can be called per loop
const int maxVarCount = 32; //The maximum number of variables that can be stored in the log
/* Settings */

constexpr struct TypeID {
  const uint8_t uint8   =   8; //0 to 255
  const uint8_t uint16  =  16; //0 to 65,535
  const uint8_t uint32  =  32; //0 to 4,294,967,295
  const uint8_t int8    = 108; //-128 to 127
  const uint8_t int16   = 116; //-32,768 to 32,767
  const uint8_t int32   = 132; //-2,147,483,648 to 2,147,483,647
  const uint8_t float16 = 216; //-3,276.8 to 3,276.7, 1 decimal point
  const uint8_t float16k= 166; //-32.768 to 32.767, 3 decimal points
  const uint8_t float32 = 232; //6-7 decimal digits of precision
  const uint8_t time    =  82;
} typeID;

class Logger {
  public:
    void init();
    void logSetting(String name, int data, bool seperator=true);
    void logSetting(String name, float data, int decimals, bool seperator=true);
    void logSetting(String name, const float *arr, int len, int decimals, bool seperator=true);
    void logArray(const float *arr, int len, int decimals);
    void logString(String s);
    void logTime(unsigned int t);
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
        uint32_t w;
      } u;
      if (data_typeID == typeID.float16) {
        u.float16 = (int16_t)round(data*10);
      } else if (data_typeID == typeID.float16k) {
        u.float16 = (int16_t)round(data*1000);
      } else {
        u.in = data;
      }

      //Store data type
      if (firstLog) {
        varID[varCount] = data_typeID;
        varCount++;
      }

      //Add data to buffer
      for (int i=0; i<data_typeID%50; i++) {
        if (bitRead(u.w, i)) {
          bitSet(buf[bufOffset/32], bufOffset%32);
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
    uint32_t buf[logBufferLen];
    uint16_t bufOffset;
    bool firstLog = true;
    uint8_t varID[maxVarCount];
    uint8_t varCount;
};
#endif
