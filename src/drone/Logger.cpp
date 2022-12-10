#include "Logger.h"

void Logger::init(){
  #if STORAGE_TYPE == SD_CARD
    checkSD(sd.begin(SdioConfig(FIFO_SDIO)));
    checkLog(0);
  
    sd.remove("log.bin");
    logFileBin.open("log.bin", O_WRITE | O_CREAT | O_TRUNC);
    checkSD(logFileBin.preAllocate(logFileSize));
  
    //Get settings file
    if (sd.exists("settings.json")) {
      FsFile settingsFile;
      settingsFile.open("settings.json");
      if (deserializeJson(sdSettings, settingsFile)) {
        logString("JSON ERROR ");
      }
      settingsFile.close();
    }
  #elif STORAGE_TYPE == RAM
    Serial.begin(115200);
  #endif
}

void Logger::checkSD(bool condition) {
  if (!condition) {
    for (;;) {
      blink(1500);
    }
  }
}

void Logger::checkLog(int fileNum) {
  #if STORAGE_TYPE == SD_CARD
    char fileName[10];
    String("log_" + String(fileNum) + ".csv").toCharArray(fileName, sizeof(fileName));
    if (sd.exists(fileName)) {
      char nextFile[10];
      String("log_" + String(fileNum+1) + ".csv").toCharArray(nextFile, sizeof(nextFile));
      if (sd.exists(nextFile)) {
        checkLog(fileNum+1);
      }
  
      logFile.open(fileName, O_WRITE);
      logFile.rename(nextFile);
      logFile.close();
    }
  #endif
}

void Logger::logSetting(String name, int data, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logString(name + "," + String(data));
}

void Logger::logSetting(String name, float data, int decimals, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logString(name + "," + String(data, decimals));
}

void Logger::logSetting(String name, const float *arr, int len, int decimals, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logString(name);
  logArray(arr, len, decimals);
}

bool Logger::checkLog() {
  if (loopsSinceLog >= logDiv) {
    loopsSinceLog = 0;
    return true;
  } else {
    loopsSinceLog++;
    return false;
  }
}

void Logger::logArray(const float *arr, int len, int decimals) {
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i], decimals);
  }
  logString(arrayString);
}

void Logger::logString(String s) {
  #if STORAGE_TYPE == SD_CARD
    logFile.open("log_0.csv", O_CREAT | O_WRITE | O_APPEND);
    logFile.print(s);
    logFile.close();
  #elif STORAGE_TYPE == RAM
    Serial.print(s);
  #endif
}

void Logger::logTime(unsigned int t) {
  logData(t, typeID.time);
}

void Logger::write() {
  //Store how long each section of the main loop takes
  storeSectionTime();

  //Write to log file
  #if STORAGE_TYPE == SD_CARD
    logFileBin.write(&buf, (bufOffset+7)/8);
  #endif
  firstLog = false;

  //Reset the buffer
  for (uint8_t i=0; i<sizeof(buf)/4; i++) {
    buf[i] = 0;
  }
  bufOffset = 0;
}

void Logger::calcSectionTime() {
  if (timerIndex < maxLoopTimerSections) {
    loopTimings[timerIndex] = micros();
    timerIndex++;
  }
}

void Logger::storeSectionTime() {
  //Process how long each section of the loop takes
  for(int i=0; i<timerIndex-1; i++) {
    loopTimings[i] = loopTimings[i+1] - loopTimings[i];
  }
  loopTimings[timerIndex] = 0;

  //Log how long each section of the loop takes
  if (timerIndex > 1) {
    for (int i=0; i<maxLoopTimerSections; i++) {
      logData((uint16_t)loopTimings[i], typeID.uint16);
    }
  }

  //Reset the variables for the next loop
  timerIndex = 0;
  for(int i=1; i<maxLoopTimerSections; i++) {
    loopTimings[i] = 0;
  }

  calcSectionTime();
}

void Logger::closeFile() {
  #if STORAGE_TYPE == SD_CARD
    //Close the binary file
    logFileBin.flush();
    logFileBin.truncate();
    logFileBin.close();
  
    //Convert the binary data to sring
    logFileBin.open("log.bin", O_READ);
    logFile.open("log_0.csv", O_WRITE | O_APPEND);
    binToStr();
    logFileBin.close();
    logFile.close();
  #elif STORAGE_TYPE == RAM
     binToStr();
  #endif
}

void Logger::binToStr() {
  union unionBuffer {
    int8_t int8;
    int16_t int16;
    int32_t int32;
    uint32_t uinteger;
    float decimal;
  } u;

  //Calculate the size of each log
  int16_t logSize = 0;
  for (int i=0; i<varCount; i++) {
    logSize += (varID[i]%50) / 8;
  }
  char buf[logSize * 100];
  uint16_t bufIndex = 0;
  
  uint32_t prevTime = 0;
  String s;
  bool partialBuffer = false;
  bool eof = false;
  #if STORAGE_TYPE == RAM
    uint32_t bigBufIndex = 0;
  #endif
  while (!eof or partialBuffer) {
    if (bufIndex == 0) {
      if (partialBuffer) {
        break;
      }
      #if STORAGE_TYPE == SD_CARD
        if (sizeof(buf) < logFileBin.size() - logFileBin.position()) {
          logFileBin.read(&buf, sizeof(buf));
        } else {
          logFileBin.read(&buf, logFileBin.available());
          partialBuffer = true;
        }
        eof = logFileBin.position() >= logFileBin.size();
      #elif STORAGE_TYPE == RAM
        //clear buffer
        for (int i=0; i<logSize * 100; i++) {
          buf[i] = 0;
        }
        for (int i=0; i<sizeof(buf) * 8; i++) {
          if (bigBufIndex == bigBufLen) {
            eof = true;
            break;
          }
          if (bitRead(bigBuf[bigBufIndex/32], bigBufIndex%32)) {
            bitSet(buf[i/8], i%8);
          }
          bigBufIndex++;
        }
      #endif
    }

    for (int i=0; i<varCount; i++) {
      if (i == 0) {
        s = "\n";
      } else {
        s += ",";
      }

      u.uinteger = 0; //Reset the variable to zero
      for (int j=0; j<varID[i]%50; j++) {
        if (bitRead(buf[bufIndex/8], bufIndex%8)) {
          bitSet(u.uinteger, j);
        }
        bufIndex++;
      }

      switch (varID[i]) {
        case typeID.time:
          s += String(u.uinteger);
          s += ",";
          s += String(u.uinteger - prevTime);
          prevTime = u.uinteger;
          break;
        case typeID.uint8:
        case typeID.uint16:
        case typeID.uint32:
          s += String(u.uinteger);
          break;
        case typeID.int8:
          s += String(u.int8);
          break;
        case typeID.int16:
          s += String(u.int16);
          break;
        case typeID.int32:
          s += String(u.int32);
          break;
        case typeID.float16:
          s += String(u.int16/10.0, 1);
          break;
        case typeID.float16k:
          s += String(u.int16/1000.0, 3);
          break;
        case typeID.float32:
          s += String(u.decimal, 3);
          break;
        default:
          s += "error";
      }
    }
    #if STORAGE_TYPE == SD_CARD
      logFile.print(s);
    #elif STORAGE_TYPE == RAM
      Serial.print(s);
    #endif
    bufIndex %= sizeof(buf) * 8;
  }
}
