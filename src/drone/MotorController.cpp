#include "Arduino.h"
#include "MotorController.h"

int toInt(float f) {
  return int(f + .5);
}


bool MotorController::init(Logger &l, const char* motorName, bool test) {
  logger = &l;
  name = motorName;

  //Load settings from the SD card
  bool loadSuccess = logger->loadSetting(name, "motorCount", &motorCount);
  if (loadSuccess) {
    motors = new int[motorCount];
    defaultValues = new float[motorCount];
    motorPower = new float[motorCount];

    loadSuccess &= logger->loadSetting(name, "pins", motors, motorCount);
    loadSuccess &= logger->loadSetting(name, "defaultValues", defaultValues, motorCount);
    #if ESC_TYPE == ONESHOT125
      loadSuccess &= logger->loadSetting(name, "signalFreq", &signalFreq);
      maxDutyCycle = signalFreq/40.0f;
    #endif
  }
  if (!loadSuccess) {
    return false;
  }

  PIDs = new PIDcontroller[MAX_PIDS];

  //Arm ESCs
  while (millis() < 2500); //Wait for ESC startup
  digitalWrite(lightPin, HIGH);
  #if ESC_TYPE == PWM
    motorSignal = new Servo[motorCount];
  #elif ESC_TYPE == ONESHOT125
    motorSignal = new Teensy_PWM*[motorCount];
  #endif
  for (int i=0; i<motorCount; i++) {
    #if ESC_TYPE == PWM
      motorSignal[i].attach(motors[i], 1000, 2000);
    #elif ESC_TYPE == ONESHOT125
      motorSignal[i] = new Teensy_PWM(motors[i], signalFreq, 0.0f);
    #endif
    writeToMotor(i, 0);
  }

  if (test) {
    delay(2000);
    digitalWrite(lightPin, LOW);
  
    //Test spin motors
    for (int i=0; i<4; i++) {
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

  return true;
}

void MotorController::setupInputs() {
  inputCount = logger->getInputCount(name);
  inputs = new InputHandler[inputCount];
  for (int i=0; i<inputCount; i++) {
    inputs[i].init(*logger, this, name, logger->getInputName(name, i));
  }
}

void MotorController::addPID(const char* PIDName, float* target, float* current, float* currentDiff) {
  if (PIDcount < MAX_PIDS) {
    PIDs[PIDcount].init(*logger, name, PIDName, target, current, currentDiff);
    PIDcount++;
  }
}

void MotorController::addPID(const char* PIDName, float target, float* current, float* currentDiff) {
  addPID(PIDName, &target, current, currentDiff);
}

void MotorController::addMotorPower(int index, float amount) {
  if (index >= 0 and index < motorCount) {
    motorPower[index] += amount;
  }
}

void MotorController::write() {
  //Set initial motor power
  for (int i=0; i<motorCount; i++) {
    motorPower[i] = defaultValues[i];
  }

  //Apply inputs
  for (int i=0; i<inputCount; i++) {
    inputs[i].processInput(this);
  }
  //Apply PIDs
  for (int i=0; i<PIDcount; i++) {
    PIDs[i].calc(this);
  }

  //Apply output to motor
  for (int i=0; i<motorCount; i++) {
    motorPower[i] = min(max(0.0f, motorPower[i]*1000), 1000.0f); //Limit values to 0 - 1000

    //Round motor power and apply it to the ESC
    writeToMotor(i, motorPower[i]);
  }
}

void MotorController::writeZero() {
  for (int i=0; i<4; i++) {
    writeToMotor(i, 0);
  }
}

void MotorController::writeToMotor(int index, float value) {
  value = max(0.0f, min(value, 1000.0f));
  #if ESC_TYPE == PWM
    motorSignal[index].writeMicroseconds(1000 + toInt(value));
  #elif ESC_TYPE == ONESHOT125
    value = map(value, 0.0f, 1000.0f, maxDutyCycle/2.0f, maxDutyCycle);
    motorSignal[index]->setPWM(motors[index], signalFreq, value);
  #endif
}

int MotorController::getPIDcount() {
  return PIDcount;
}

void InputHandler::init(Logger &logger, MotorController* controller, const char* parent, const char* name) {
  logger.loadSetting(parent, "Controls", name, "min", &minControl);
  logger.loadSetting(parent, "Controls", name, "max", &maxControl);
  if (!logger.loadSetting(parent, "Controls", name, "mid", &midControl)) {
    midControl = (minControl + maxControl) / 2;
  }
  if (logger.loadSetting(parent, "Controls", name, "PIDTarget", &PIDTargetStr)) {
    for (int i; i<controller->getPIDcount(); i++) {
      if (strcmp(controller->PIDs[i].getName(), PIDTargetStr) == 0) {
        PIDTarget = &controller->PIDs[i];
      }
    }
  } else {
    logger.loadSetting(parent, "motorCount", &outputPinCount);
    outputPin = new int[outputPinCount];
    logger.loadSetting(parent, "Controls", name, "pins", outputPin, outputPinCount);
  }

  input = 0.0;////TODO: implement
}

void InputHandler::processInput(MotorController* controller) {
  float output;
  if (input == .5) {
    output = midControl;
  } else if (input < .5) {
    output = map(input, 0.0, .5, minControl, midControl);
  } else {
    output = map(input, .5, 1.0, midControl, maxControl);
  }

  if (outputPinCount > 0) {
    for (int i=0; i<outputPinCount; i++) {
      controller->addMotorPower(outputPin[i], output);
    }
  } else {
    PIDTarget->addInput(output);
  }
}
