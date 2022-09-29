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
  //TODO - convert binary data to string
}
