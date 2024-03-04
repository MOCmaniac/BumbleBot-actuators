#include "DebugPrint.h"
#include <AccelStepper.h>
#include <PWMServo.h>


// defines stepper pin
const int enPotPin = 23;
const int stepPotPin = 22;
const int dirPotPin = 21;

const int enPlantPin = 20;
const int stepPlantPin = 19;
const int dirPlantPin = 18;


// defines servo pin number (from pin 3 to 8)
unsigned long lastServoUpdate = 0;
const int servoTransPin = 3;
//const int servoGripperPin = 4;
const int servoArmPin = 4;
//const int servoWheelPin = 6;

int targetAngleTrans = 90;
int targetAngleArm = 0;

// µs1 = 32, µs2 = 31, µs3 = 30, µs4 = 27
const int microswitchPotPin = 27;    // Green cable
const int microswitchPlantPin = 30;  // Blue cable


// TMC2209 : interpolation to 256 microsteps
const int homingSpeed = 5000;  // max found : 16000 steps/second
const int forkSpeed = 10000;
const int forkAcceleration = 40e3;  // in steps/second²
int homingCompleted = 0;

const int stepsPerRev = 200;
const int microstepping = 8;
const float ratio = 7.5;
const int steps = round(stepsPerRev * ratio * 1.3) * microstepping;  // Reduction 7.5:1
//const int stepsPot = round(stepsPerRev * ratio * 1.3) * microstepping;    // Reduction 7.5:1

int plantHasPriority = 1;
int targetPositionForkPlant = steps;
int targetPositionForkPot = steps;


// LED
const int LEDPin = 13;
int state = HIGH;
unsigned long LEDCurrentDelay = 1000;
const unsigned long LEDDelay = 500;
const unsigned long LEDDelayWarning = 100;
unsigned long previousLED = 0;  // will store last time led state changed


// Serial input variables
const byte numChars = 10;
char receivedChars[numChars];
boolean newData = false;


// Objects declaration
AccelStepper stepperPlant(AccelStepper::DRIVER, stepPlantPin, dirPlantPin);
AccelStepper stepperPot(AccelStepper::DRIVER, stepPotPin, dirPotPin);

PWMServo servoTrans;
//PWMServo servoGripper;
PWMServo servoArm;
//PWMServo servoWheel;


void setup() {
  SERIAL_BEGIN(115200);
  //DBG_PRINTF("\n\nStarting\nSteps : %d\n", steps);

  pinMode(LEDPin, OUTPUT);

  pinMode(microswitchPotPin, INPUT_PULLUP);
  pinMode(microswitchPlantPin, INPUT_PULLUP);

  setupSteppers();
  setupServos();

  DBG_PRINTLN("Setups done");
}

void loop() {
  // Handle translation and lifting of forks, arm and wheel to avoid perimeter problems
  // Plant's fork height should always be higher than the pot's fork
  // Forks should be put to there parking position to be retracted
  recvWithStartEndMarkers();

  if (millis() - previousLED > LEDCurrentDelay) {
    previousLED = millis();
    state = !state;
    digitalWrite(LEDPin, state);
  }

  if (newData) {
    DBG_PRINTLN("Handling data");
    handleCommand(receivedChars);
    newData = false;
  }

  writeServos();


  int posPlant = targetPositionForkPlant;
  int posPot = targetPositionForkPot;
  if (plantHasPriority) {
    posPot = min(targetPositionForkPot, targetPositionForkPlant);
  } else {
    posPlant = max(targetPositionForkPot, targetPositionForkPlant);
  }

  stepperPlant.moveTo(posPlant);
  stepperPot.moveTo(posPot);

  stepperPlant.run();
  stepperPot.run();
}

/*
 * This blocking function does the homing of the stepper motors
 * Returns true if homing is successful
*/
int homing(int speed) {
  stepperPot.setMaxSpeed(speed);  // Used by runToNewPosition()
  stepperPlant.setMaxSpeed(speed);

  DBG_PRINTLN("Moving pot's fork away from the switch");
  int statusPot = homingMove(&stepperPot, -speed, steps * 0.1, microswitchPotPin, true);
  DBG_PRINTLN("Moving plant's fork away from the switch");
  int statusPlant = homingMove(&stepperPlant, -speed, steps * 0.1, microswitchPlantPin, true);

  if (statusPot == 2) {
    stepperPot.setCurrentPosition(0);
    stepperPot.runToNewPosition(-400);
  }
  if (statusPlant == 2) {
    stepperPlant.setCurrentPosition(0);
    stepperPlant.runToNewPosition(-400);
  }

  if (statusPlant) {
    DBG_PRINTLN("Moving plant's fork towards the switch");
    statusPlant = homingMove(&stepperPlant, speed, steps * 1.1, microswitchPlantPin, false);
  }
  if (statusPot) {
    DBG_PRINTLN("Moving pot's fork towards  the switch");
    statusPot = homingMove(&stepperPot, speed, steps * 1.1, microswitchPotPin, false);
  }

  if (statusPlant) {
    stepperPlant.setCurrentPosition(steps - 60);  // define position
  }
  if (statusPot) {
    stepperPot.setCurrentPosition(steps);  // define position
  }

  return statusPlant && statusPot;
}

/*
 * Returns 0 if homing move failed
 * Returns 1 if homing move succeed and motor did not moved
 * Returns 2 if homing move succeed and motor moved
*/
int homingMove(AccelStepper *stepper, int speed, int maxTravelledSteps, int pin, int logic) {
  int startPosition = stepper->currentPosition();
  int stepsTravelled = 0;
  stepper->setSpeed(speed);  // Used by runSpeed()

  while (readMicroswitch(pin) == logic && stepsTravelled < maxTravelledSteps) {
    stepper->runSpeed();
    stepsTravelled = abs(stepper->currentPosition() - startPosition);
  }

  int status = 2;
  if (stepsTravelled >= maxTravelledSteps) {
    status = 0;
  } else if (stepsTravelled == 0) {
    status = 1;
  }

  DBG_PRINTF("Steps travelled : %d (max : %d)\n", stepsTravelled, maxTravelledSteps);
  DBG_PRINTF("Homing move done, status = %d\n", status);
  return status;
}

/*
 * Moves plant's fork to the desired height within range
 * height : height in mm
 */
void setTargetForkPlant(int height) {
  if (height <= 100 && height >= 0) {
    int step = map(height, 0, 100, 0, steps);
    targetPositionForkPlant = constrain(step, 0, steps);
  }
}

void setTargetForkPot(int height) {
  if (height <= 100 && height >= 0) {
    int step = map(height, 0, 100, 0, steps);
    targetPositionForkPot = constrain(step, 0, steps);
  }
}

/*
 * Translation = 0 : forks are retracted
 * Translation = 1 : forks are deployed
 */
void setTranslationForks(int translation) {
  if (translation == 0 || translation == 1) {
    targetAngleTrans = (translation) ? 110 : 0;
  }
}

void setRotationArm(int angle) {
  if (angle >= 0 && angle <= 180) {
    targetAngleArm = angle;
  }
}

/*
 * State = 0 : gripper is open
 * State = 1 : gripper is closed
 */
void setPositionGripper(int state) {
  //(state) ? servoGripper.write(0) : servoGripper.write(40);
}

// returns true if switch is pressed
int readMicroswitch(int pin) {
  return digitalRead(pin);
}

// Code from Robin2 on arduino's website : Serial Input Basics
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = ' ';  // '<'
  char endMarker = ' ';    // '>'
  char rc;

  while (Serial.available() > 0) {  // Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    //Serial.printf("Received character: %c\n", rc);

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        Serial.printf("Message received: %s\n", receivedChars);
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void handleCommand(char *string) {
  char command;
  int value;
  sscanf(string, "%c-%d", &command, &value);

  switch (command) {
    case 'A':
      setRotationArm(value);
      break;
    case 'D':
      stepperPlant.disableOutputs();
      stepperPot.disableOutputs();
      break;
    case 'E':
      stepperPlant.enableOutputs();
      stepperPot.enableOutputs();
      break;
    case 'F':  // Fork pot
      plantHasPriority = 0;
      setTargetForkPot(value);
      break;
    case 'f':  // Fork plant
      plantHasPriority = 1;
      setTargetForkPlant(value);
      break;
    case 'G':  // Plants gripper
      DBG_PRINTLN("Plants gripper");
      break;
    case 'O':
      DBG_PRINTLN("Preparing for power down");
      // set all variables to park forks and servo if needed
      break;
    case 'S':
      setupSteppers();
      break;
    case 'T':  // Forks translation
      setTranslationForks(value);
      break;
    default:
      DBG_PRINTLN("Command not recognized !");
      break;
  }
  DBG_PRINTF("Command received : %s\n", string);
}

void writeServos() {
  static unsigned long lastServoUpdate = 0;
  static int angleTrans = 90;
  static int angleArm = 90;
  if (millis() - lastServoUpdate > 3) {
    lastServoUpdate = millis();

    if (targetAngleTrans > angleTrans) {
      angleTrans++;
    } else if (targetAngleTrans < angleTrans) {
      angleTrans--;
    }
    // Can use servo.writeMicroseconds() if want higher resolution than 1°
    servoTrans.write(angleTrans);
    //DBG_PRINTF("Angle servo translation : %d\n", angleTrans);

    if (targetAngleArm > angleArm) {
      angleArm++;
    } else if (targetAngleArm < angleArm) {
      angleArm--;
    }
    servoArm.write(targetAngleArm);
  }
}

void setupSteppers() {
  pinMode(enPotPin, OUTPUT);
  pinMode(stepPotPin, OUTPUT);
  pinMode(dirPotPin, OUTPUT);

  pinMode(enPlantPin, OUTPUT);
  pinMode(stepPlantPin, OUTPUT);
  pinMode(dirPlantPin, OUTPUT);

  stepperPlant.setMinPulseWidth(2);
  stepperPlant.setEnablePin(enPlantPin);
  stepperPlant.setPinsInverted(false, false, true);  // direction, step, enable
  stepperPlant.enableOutputs();
  stepperPlant.setAcceleration(forkAcceleration);

  stepperPot.setMinPulseWidth(2);
  stepperPot.setEnablePin(enPotPin);
  stepperPot.setPinsInverted(true, false, true);
  stepperPot.enableOutputs();
  stepperPot.setAcceleration(40000.0);

  // /!\ if fork pot to the top can't home fork plant !
  homingCompleted = homing(homingSpeed);
  if (!homingCompleted) DBG_PRINTLN("Homing forks failed");

  LEDCurrentDelay = homingCompleted ? LEDDelay : LEDDelayWarning;
  if (homingCompleted) {
    stepperPlant.setMaxSpeed(forkSpeed);  // in steps/second
    stepperPot.setMaxSpeed(forkSpeed);
    setTargetForkPlant(100);
    setTargetForkPot(100);
  } else {
    stepperPlant.disableOutputs();
    stepperPot.disableOutputs();
  }
}

void setupServos() {
  pinMode(servoTransPin, OUTPUT);
  //pinMode(servoGripperPin, OUTPUT);
  pinMode(servoArmPin, OUTPUT);
  //pinMode(servoWheel, OUTPUT);

  servoTrans.attach(servoTransPin, 500, 2500);
  setTranslationForks(1);
  //servoGripper.attach(servoGripperPin);
  //setPositionGripper(0);

  servoArm.attach(servoArmPin, 1000, 2000);
}
