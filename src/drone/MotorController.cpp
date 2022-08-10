#include "Arduino.h"
#include "MotorController.h"

int toInt(float f){
  return int(f + .5);
}

void MotorController::init() {
  //Prime motors
  digitalWrite(lightPin, HIGH);
  for (int i=0; i<4; i++){
    #if ESC_TYPE == PWM
      ESCsignal[i].attach(motors[i], 1000, 2000);
      delay(1);
      ESCsignal[i].write(0);
    #endif
  }
  delay(1400);
  digitalWrite(lightPin, LOW);

  //Test spin motors
  for (int i=0; i<4; i++){
    delay(100);
    digitalWrite(lightPin, HIGH);
    #if ESC_TYPE == PWM
      ESCsignal[i].write(10);
      delay(300);
      ESCsignal[i].write(0);
    #endif
    digitalWrite(lightPin, LOW);
  }
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
    motorPower[i] = min(max(0, motorPower[i]*1000), 1000); //Limit values to 0 - 1000

    #if ESC_TYPE == PWM
      ESCsignal[i].writeMicroseconds(1000 + toInt(motorPower[i])); //Round motor power and apply it to the ESC
    #endif

    dynamicChange[i] = 0;
  }
}

void MotorController::writeZero() {
  for (int i=0; i<4; i++){
    #if ESC_TYPE == PWM
      ESCsignal[i].write(0);
    #endif
  }
}
