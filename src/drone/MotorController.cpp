#include "Arduino.h"
#include "MotorController.h"

int toInt(float f){
  return int(f + .5);
}

void MotorController::init(Logger &logger) {
  //Load settings from the SD card
  logger.loadSetting("motorOffset", offset, 4);
  logger.loadSetting("defaultZ", defaultZ);
  logger.loadSetting("maxZdiff", maxZdiff, 2);
  logger.loadSetting("potMaxDiff", potMaxDiff);

  //Arm ESCs
  digitalWrite(lightPin, HIGH);
  for (int i=0; i<4; i++){
    #if ESC_TYPE == PWM
      ESCsignal[i].attach(motors[i], 1000, 2000);
      delay(1);
      writeToMotor(i, 0);
    #endif
  }
  delay(1400);
  digitalWrite(lightPin, LOW);

  //Test spin motors
  for (int i=0; i<4; i++){
    delay(100);
    digitalWrite(lightPin, HIGH);
    writeToMotor(i, 55);
    delay(300);
    writeToMotor(i, 0);
    digitalWrite(lightPin, LOW);
  }

  //Give the motor some time to stop spinning before continuing with the program
  delay(250);
}

void MotorController::addChange(float PIDchange[3][3], int axis, int pA, int pB, int nA, int nB) {
  float change = PIDchange[0][axis] + PIDchange[1][axis] + PIDchange[2][axis];

  if (abs(change) > .001) {
    dynamicChange[pA] += change;
    dynamicChange[pB] += change;

    dynamicChange[nA] -= change - .001;
    dynamicChange[nB] -= change - .001;
  }
}

void MotorController::write() {
  //Set initial motor power
  if (xyzr[2] < 0) {
    initialPower = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[0] * xyzr[2]/127);
  } else {
    initialPower = defaultZ + (potPercent*potMaxDiff) + (maxZdiff[1] * xyzr[2]/127);
  }

  //Apply output to motor
  for (int i=0; i<4; i++){
    motorPower[i] = initialPower;
    motorPower[i] += dynamicChange[i];
    motorPower[i] += offset[i]/100;
    motorPower[i] = min(max(0.0f, motorPower[i]*1000), 1000.0f); //Limit values to 0 - 1000

    //Round motor power and apply it to the ESC
    writeToMotor(i, toInt(motorPower[i]));

    dynamicChange[i] = 0;
  }
}

void MotorController::writeZero() {
  for (int i=0; i<4; i++){
    writeToMotor(i, 0);
  }
}

void MotorController::writeToMotor(int index, int value) {
  #if ESC_TYPE == PWM
    ESCsignal[index].writeMicroseconds(1000 + value);
  #endif
}
