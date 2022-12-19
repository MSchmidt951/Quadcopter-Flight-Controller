#ifndef __Logger_H__
#define __Logger_H__

//Different types of storage defined here
#define RAM -1 //This should only be used for testing and has very little storage space
#define SD_CARD 0

//Select which storage to use
#define STORAGE_TYPE SD_CARD

//Import libraries
#if STORAGE_TYPE == SD_CARD
  #include <SdFat.h>
  #include <ArduinoJson.h>
#elif STORAGE_TYPE == RAM
  #include "Arduino.h"
#endif

extern void blink(int);
extern const int loopRate;


/* Settings */
///The max amont of buffer sections, each being 4 bytes long
const int logBufferLen = 24;
///Sets the logging rate (data is logged every logDiv loops)
const int logDiv = 1;
///Reserve enough space for 10 mins
const int logFileSize = 10*60 * logBufferLen*4 * loopRate/logDiv;
///The maximum times calcSectionTime can be called per loop
const int maxLoopTimerSections = 8;
///The maximum number of variables that can be stored in the log
const int maxVarCount = 32;
/* Settings */

/** 
 * @class TypeID
 * @brief Contains IDs of the data types allowed in the log
 */
constexpr struct TypeID {
  ///0 to 255, 8 bit storage size
  const uint8_t uint8   =   8;
  ///0 to 65,535, 16 bit storage size
  const uint8_t uint16  =  16;
  ///0 to 4,294,967,295, 32 bit storage size
  const uint8_t uint32  =  32;
  ///-128 to 127, 8 bit storage size
  const uint8_t int8    = 108;
  ///-32,768 to 32,767, 16 bit storage size
  const uint8_t int16   = 116;
  ///-2,147,483,648 to 2,147,483,647, 32 bit storage size
  const uint8_t int32   = 132;
  ///-3,276.8 to 3,276.7, 1 decimal point, 16 bit storage size
  const uint8_t float16 = 216;
  ///-32.768 to 32.767, 3 decimal points, 16 bit storage size
  const uint8_t float16k= 166;
  ///6-7 decimal digits of precision, 32 bit storage size
  const uint8_t float32 = 232;
  ///Time in μs, max 4,294,967,295, 32 bit storage size
  const uint8_t time    =  82;
} typeID;

/** 
 * @class Logger
 * @brief Logs device data and loads settings
 */
class Logger {
  public:
    /** Move/remove old logs, allocate space for the current log, read the settings file
     *  @brief Setup storage for logging
     */
    void init();
    /** Log setting (integer) to the current flight log
     *  
     *  @param[in] name Name of the setting
     *  @param[in] data Value of the setting
     *  @param[in] seperator Whether or not to add a seperator before logging the variable
     */
    void logSetting(String name, int data, bool seperator=true);
    /** Log setting (float) to the current flight log
     *  
     *  @param[in] name Name of the setting
     *  @param[in] data Value of the setting
     *  @param[in] decimals How many decimal places to round the setting to
     *  @param[in] seperator Whether or not to add a seperator before logging the variable
     */
    void logSetting(String name, float data, int decimals, bool seperator=true);
    /** Log setting (float array) to the current flight log
     *  
     *  @param[in] name Name of the setting
     *  @param[in] arr Array containing the values of the setting
     *  @param[in] decimals How many decimal places to round the settings to
     *  @param[in] seperator Whether or not to add a seperator before logging the variable
     */
    void logSetting(String name, const float *arr, int len, int decimals, bool seperator=true);
    /** Checks if data should be logged this loop
     *  
     *  @returns true if data should be logged
     */
    bool checkLogReady();
    /** Log an array of float variables
     *  
     *  @param[in] arr Array to log
     *  @param[in] len Length of array
     *  @param[in] decimals The number of decimal places to log
     */
    void logArray(const float *arr, int len, int decimals);
    /** Log a string
     *  
     *  @param[in] s String to log
     */
    void logString(String s);
    /** Log a timestamp. Used at the start of each entry
     *  
     *  @param[in] t Timestamp to log
     */
    void logTime(unsigned int t);
    /** Write any data data in the buffer to the log */
    void write();
    /** Calculates how long a section of the main loop takes */
    void calcSectionTime();
    /** Stores the min, max and average time of each section of the main loop */
    void storeSectionTime();
    /** Write to and close the binary file then run binToStr() */
    void closeFile();

    /** Load a setting from storage
     *  
     *  @param[in] name Name of the setting
     *  @param[out] var Variable to set
     */
    template <typename T> void loadSetting(String name, T &var){
      #if STORAGE_TYPE == SD_CARD
        if (sdSettings.containsKey(name)) {
          if (sdSettings[name] != "default") {
            var = sdSettings[name];
          }
        }
      #endif
    }
    /** Load a setting from storage
     *  
     *  @param[in] name Name of the setting
     *  @param[out] var Variable to set
     *  @param[in] len Length of the array
     */
    template <typename T> void loadSetting(String name, T *var, int len){
      #if STORAGE_TYPE == SD_CARD
        if (sdSettings.containsKey(name)) {
          for (int i=0; i<len; i++) {
            if (sdSettings[name][i] != "default") {
              var[i] = sdSettings[name][i];
            }
          }
        }
      #endif
    }
    /** Log data to the binary log file
     *  
     *  @param[in] data Data to log
     *  @param[in] data_typeID Datatype to log. IDs can be found at TypeID
     */
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
          #if STORAGE_TYPE == RAM
            bitSet(bigBuf[bigBufLen/32], bigBufLen%32);
          #endif
        }
        #if STORAGE_TYPE == RAM
          bigBufLen++;
        #endif
        bufOffset++;
      }
    }

  private:
    /** Checks a condition of the SD card, if false abort
     *  
     *  @param[in] condition Condition to check
     */
    void checkSD(bool condition);
    /** A recursive function to free up the name 'log_0.csv' (the current log).
     *  
     *  Checks if a certain log exists, if it does then check if the next file exists.
     *  If the next file exists run checkLog on that file.
     *  
     *  Then increment the logs number
     *  
     *  @param[in] fileNum File number to check
     */
    void checkLog(int fileNum);
    /** Converts the binary log 'log.bin' to the readable file 'log_0.csv' */
    void binToStr();

    ///The number of loops since data has been logged
    uint8_t loopsSinceLog = 255;
    //File variables
    #if STORAGE_TYPE == SD_CARD
      ///SD card object
      SdFs sd;
      ///Log file object
      FsFile logFile;
      ///Binary log file object
      FsFile logFileBin;
      ///JSON document holding all the settings
      StaticJsonDocument<512> sdSettings;
    #elif STORAGE_TYPE == RAM
      ///Buffer that holds all of the data in RAM mode
      uint32_t bigBuf[2100*36/4];
      ///The length of data that has been written to bigBuf
      uint32_t bigBufLen;
    #endif
    //Section timer variables
    ///Timestamps of when sections of the current loop has been completed (μs)
    unsigned long loopTimings[maxLoopTimerSections];
    ///The next free index of loopTimings
    uint8_t timerIndex;
    //Buffer variables
    ///Buffer holding the data for one loop
    uint32_t buf[logBufferLen];
    ///The offset used when writing to buf
    uint16_t bufOffset;
    ///True if no data has been written to the log
    bool firstLog = true;
    ///An array storing TypeID values of each variable stored
    uint8_t varID[maxVarCount];
    ///How many variables are stored per data entry
    uint8_t varCount;
};
#endif
