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

void Logger::write() {
  //Write to log file
  currentLog.getBinary(buf);
  logFileBin.write(&buf, sizeof(logStruct));

  //Store how long each section of the main loop takes
  storeSectionTime();
}

void Logger::calcSectionTime() {
  if (timerIndex < maxLoopTimerSections) {
    loopTimings[timerIndex] = micros();
    timerIndex++;
  }
}

void Logger::storeSectionTime() {
  //Process how long each section of the loop takes
  for(int i=0; i<maxLoopTimerSections-1; i++) {
    if (loopTimings[i+1] != 0) {
      loopTimings[i] = loopTimings[i+1] - loopTimings[i];
    }
  }

  //Log how long each section of the loop takes
  //TODO - log loopTimings

  //Reset the variables for the next loop
  timerIndex = 0;
  for(int i=1; i<maxLoopTimerSections; i++) {
    loopTimings[i] = 0;
  }

  calcSectionTime();
}

void Logger::closeFile() {
  binToStr();
  logFileBin.flush();
  logFileBin.truncate();
  logFileBin.close();
}

void Logger::binToStr() {
  //TODO - convert binary data to string
}


void logStruct::getBinary(uint32_t buf[]) {
  union unionBuffer {
    float f;
    uint32_t buf;
  } u;

  buf[0] = time;
  u.f = roll;  buf[1] = u.buf;
  u.f = pitch; buf[2] = u.buf;
  buf[3] = (Pp << 16) | Pr;
  buf[4] = (Ip << 16) | Ir;
  buf[5] = (Dp << 16) | Dr;
  buf[6] = (radio << 16) | yaw;
  buf[7] = (rollInput << 24) | (pitchInput << 16) | (verticalInput << 8) | pot;
}
