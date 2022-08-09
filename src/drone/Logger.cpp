#include "Logger.h"

void Logger::init(){
  checkSD(sd.begin(SdioConfig(FIFO_SDIO)));
  if (sd.exists("log.csv")) {
    sd.remove("log_old.csv");
    logFile.open("log.csv", O_WRITE);
    logFile.rename("log_old.csv");
    logFile.close();
  }
  logFile.open("log.csv", O_WRITE | O_CREAT | O_TRUNC);
  checkSD(logFile.preAllocate(logFileSize));
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

void Logger::logSetting(String name, int *arr, int len, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logString(name);
  logArray(arr, len);
}

void Logger::logSetting(String name, float *arr, int len, int decimals, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logString(name);
  logArray(arr, len, decimals);
}

void Logger::logString(String s) {
  logFile.print(s);
}

void Logger::logData(int data, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logFile.print(data);
}

void Logger::logData(float data, int decimals, bool seperator) {
  if (seperator) {
    logString(",");
  }
  logFile.print(data);
}

void Logger::logArray(int *arr, int len) {
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i]);
  }
  logString(arrayString);
}

void Logger::logArray(float *arr, int len, int decimals) {
  String arrayString = "";
  for (int i=0; i<len; i++) {
    arrayString += "," + String(arr[i], decimals);
  }
  logString(arrayString);
}

void Logger::write(bool mustWrite) {
  //Process how long each section of the loop takes
  for(int i=0; i<maxLoopTimerSections-1; i++) {
    if (loopTimings[i+1] != 0) {
      loopTimings[i] = loopTimings[i+1] - loopTimings[i];
    }
  }
  
  //Log how long each section of the loop takes
  logArray(loopTimings, timerIndex-1);
  
  //Reset the variables for the next loop
  timerIndex = 0;
  for(int i=1; i<maxLoopTimerSections; i++) {
    loopTimings[i] = 0;
  }

  logString("\n");
  logLoopCounter++;
  if (logLoopCounter == maxLogLoopCounter or mustWrite) {
    logFile.flush();
    logLoopCounter = 0;
  }

  if (!mustWrite) {
    calcSectionTime();
  }
}

void Logger::calcSectionTime() {
  if (timerIndex < maxLoopTimerSections) {
    loopTimings[timerIndex] = micros();
    timerIndex++;
  }
}

void Logger::closeFile() {
  logFile.flush();
  logFile.truncate();
  logFile.close();
}
