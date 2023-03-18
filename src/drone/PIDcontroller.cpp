#include "PIDcontroller.h"
#include "MotorController.h"

void PIDcontroller::init(Logger &logger, const char* parent, const char* name, float* targetPtr, float* currentPtr, float* currentDiffPtr) {
  logger.loadSetting(parent, "PIDs", name, "PIDGains", PIDGains, 3);

  logger.loadSetting(parent, "PIDs", name, "positiveCount", &positivePinCount);
  positivePins = new int[positivePinCount];
  logger.loadSetting(parent, "PIDs", name, "positive", positivePins, positivePinCount);

  logger.loadSetting(parent, "PIDs", name, "negativeCount", &negativePinCount);
  negativePins = new int[negativePinCount];
  logger.loadSetting(parent, "PIDs", name, "negative", negativePins, negativePinCount);

  namePtr = name;
  target = targetPtr;
  current = currentPtr;
  currentDiff = currentDiffPtr;
}

void PIDcontroller::calc(MotorController* controller) {
  float PIDchange = 0;

  //Get difference between target and current angle
  lastErr = currentErr;
  currentErr = (*target)+inputOffset - (*current);

  //Get proportional change
  PIDchange = currentErr * PIDGains[0]/1000.0;

  //Get integral change
  iSum += currentErr * loopTime();
  PIDchange += iSum * PIDGains[1]/1000.0;

  //Get derivative change
  if (currentDiff == NULL) {
    PIDchange += ((currentErr - lastErr)/loopTime()) * -PIDGains[2];
  } else {
    PIDchange += (*currentDiff) * -PIDGains[2];
  }

  if (abs(PIDchange) > .001) {
    for (int i=0; i<positivePinCount; i++) {
      controller->addMotorPower(positivePins[i], PIDchange);
    }
    for (int i=0; i<negativePinCount; i++) {
      controller->addMotorPower(negativePins[i], -PIDchange);
      if (positivePinCount > 0) {
        controller->addMotorPower(negativePins[i], .001);
      }
    }
  }

  //Reset inputs for the next loop
  inputOffset = 0;
}

void PIDcontroller::addInput(float input) {
  inputOffset += input;
}

const char* PIDcontroller::getName() {
  return namePtr;
}
