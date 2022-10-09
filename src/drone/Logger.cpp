#include "Logger.h"

void Logger::init(){
  checkSD(sd.begin(SdioConfig(FIFO_SDIO)));
  if (sd.exists("log.csv")) {
    sd.remove("log_old.csv");
    logFile.open("log.csv", O_WRITE);
    logFile.rename("log_old.csv");
    logFile.close();
  }

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
}

void Logger::checkSD(bool condition) {
  if (!condition) {
    for (;;) {
      blink(1500);
    }
  }
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

void Logger::logArray(const float *arr, int len, int decimals) {
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i], decimals);
  }
  logString(arrayString);
}

void Logger::logString(String s) {
  logFile.open("log.csv", O_CREAT | O_WRITE | O_APPEND);
  logFile.print(s);
  logFile.close();
}

void Logger::logTime(unsigned int t) {
  logData(t, typeID.time);
}

void Logger::write() {
  //Store how long each section of the main loop takes
  storeSectionTime();

  //Write to log file
  logFileBin.write(&buf, (bufOffset+7)/8);
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
  //Close the binary file
  logFileBin.flush();
  logFileBin.truncate();
  logFileBin.close();

  //Convert the binary data to sring
  logFileBin.open("log.bin", O_READ);
  logFile.open("log.csv", O_WRITE | O_APPEND);
  binToStr();
  logFileBin.close();
  logFile.close();
}

void Logger::binToStr() {
  union unionBuffer {
    int8_t int8;
    int16_t int16;
    int32_t int32;
    uint32_t uinteger;
    float decimal;
  } u;

  uint32_t prevTime = 0;
  String s;
  while (logFileBin.position() < logFileBin.size()) {
    for (int i=0; i<varCount; i++) {
      if (i == 0) {
        s = "\n";
      } else {
        s += ",";
      }

      u.uinteger = 0; //Reset the variable to zero
      logFileBin.read(&u.uinteger, (varID[i]%50)/8);

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
    logFile.print(s);
  }
}
