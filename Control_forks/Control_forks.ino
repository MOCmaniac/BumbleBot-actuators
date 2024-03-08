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
const int feedbackPin = 24;  // Tx J4 feedback is on 3.3V
//const int servoGripperPin = 4;
const int servoArmPin = 5;
//const int servoWheelPin = 6;

const int angleRetracted = 0;
const int angleDeployed = 145;
int targetAngleTrans = angleDeployed;
int targetAngleArm = 0;
int angleArm = 0;
int currentAngleTrans = 0;
int currentFeedbackTrans = 0;
int angle = 0;
int feedback = 0;
int minFeedback = 600;   // Based on a 12 bits resolution
int maxFeedback = 2485;  // 180° : 2485
int deployedFeedback = 2009;

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
const int steps = round(stepsPerRev * ratio * 1.1) * microstepping;  // Reduction 7.5:1
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

  setupServos();
  //setupSteppers();
  //setupSteppers();
  homingCompleted=1;

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

  readServo(0);
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
    targetAngleTrans = (translation) ? angleDeployed : angleRetracted;
  } else if (translation >= 2 && translation <= 180) {
    targetAngleTrans = translation;
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

// Returns value from the angle according to the feedback
void readServo(int forceUpdate) {
  static unsigned long lastServoRead = 0;
  float fb = 0;
  static int FB = 0;

  if (millis() - lastServoRead > 2 || forceUpdate) {
    lastServoRead = millis();

    for (int x = 0; x < 5; x++) {
      fb += analogRead(feedbackPin);
    }
    FB = round(fb / 5);
  }

  feedback = FB;
  angle = map(FB, minFeedback, maxFeedback, 0, 180);  // map function does not constraint
}

void writeServos() {
  static unsigned long lastServoWrite = 0;

  if (millis() - lastServoWrite > 3) {
    lastServoWrite = millis();

    if (currentFeedbackTrans + 70 < feedback) {  // Pulls forks inward
      currentAngleTrans++;
    } else if (currentFeedbackTrans - 16 > feedback) {  // Push forks outward
      currentAngleTrans--;
    } else if (targetAngleTrans > currentAngleTrans) {
      currentAngleTrans++;
    } else if (targetAngleTrans < currentAngleTrans) {
      currentAngleTrans--;
    }
    currentAngleTrans = constrain(currentAngleTrans, 0, 180);
    currentFeedbackTrans = angleToFb(currentAngleTrans);
    // Can use servo.writeMicroseconds() if want higher resolution than 1°
    servoTrans.write(currentAngleTrans);

    if (targetAngleArm > angleArm) {
      angleArm++;
    } else if (targetAngleArm < angleArm) {
      angleArm--;
    }
    servoArm.write(angleArm);
    //DBG_PRINTF("AngleArm : %d\n", angleArm);
  }
}

int angleToFb(int angle) {
  float fb = minFeedback + (maxFeedback - minFeedback) * angle / 180;
  fb = constrain(fb, 0, 4096);  // Can constraint feedback because can read angle a bit below 0 and higher than 180
  return round(fb);
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
  pinMode(servoArmPin, OUTPUT);
  pinMode(feedbackPin, INPUT);

  servoTrans.attach(servoTransPin, 500, 2500);
  analogReadResolution(12);

  //servoArm.attach(servoArmPin, 1000, 2000); // Tested from 750 t0 2350. 800 to 2300 gives 180° (to check !)
  servoArm.attach(servoArmPin, 500, 2500); // Used for solar panel rotation
  // Servo gripper : datasheet 500-2500

  readServo(1);
  currentAngleTrans = angle;
  currentFeedbackTrans = feedback;

  DBG_PRINTF("Min feedback : %d\t max feedback : %d\n", minFeedback, maxFeedback);
  //calibration(&servoTrans, angleRetracted, &minFeedback);
  //calibration(&servoTrans, angleDeployed, &deployedFeedback);
  DBG_PRINTF("Min feedback : %d\t max feedback : %d\n", minFeedback, maxFeedback);
}

/*
 * Does the calibration of the servo feedback value at angle specified.
 * If it fails, leave default values
*/
void calibration(PWMServo *servo, int a, int *oldFb) {
  DBG_PRINTF("\n\nStarting calibration for angle : %d, old feedback : %d\n", a, *oldFb);
  readServo(1);
  DBG_PRINTF("Current angle : %d, current feedback : %d\n", angle, feedback);
  unsigned long start = millis();
  unsigned long time = start;
  float fb = feedback;
  float lastFb = *oldFb;
  //servo.write(angle); // library already constrain written angle value
  targetAngleTrans = a;

  float error = 20.0;
  while ((abs(angle - a) > 1 || error > 0.01) && time < start + 10000) {
    writeServos();
    readServo(1);
    lastFb = fb;
    fb = 0.98 * fb + 0.02 * feedback;
    error = abs(fb - lastFb);
    time = millis();
    //DBG_PRINTF("angle : %d, error : %.3f, time : %d\n", angle, error, start+10000 - time);
    delay(1);
  }

  delay(100);  // wait for the servo to stop moving
  readServo(1);

  if (abs(*oldFb - feedback) < 20) {
    *oldFb = feedback;
    DBG_PRINTF("Calibration successful\n  New feedback value : %d\n", feedback);
  } else {
    DBG_PRINTLN("Calibration failed\n  Value outside range");
  }
}