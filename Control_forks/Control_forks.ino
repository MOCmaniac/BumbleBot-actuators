#include "DebugPrint.h"
#include <AccelStepper.h>
#include <PWMServo.h>

const int ANALOG_RESOLUTION = 12;
const int ANALOG_RANGE = 4096;

// defines stepper pin
const int enPotPin = 23;
const int stepPotPin = 22;
const int dirPotPin = 21;

const int enPlantPin = 20;
const int stepPlantPin = 19;
const int dirPlantPin = 18;


// defines servo pin number (from pin 3 to 8)
unsigned long lastServoUpdate = 0;
const int feedbackPin = 24;  // Tx J4, feedback from servo is on 3.3V
const int SERVO_PIN[] = {3, 4, 5, 6}
const int SERVO_ANGLE_MIN[] = {0, 0, 0, 0};
const int SERVO_ANGLE_DEPLOYED[] = {0, 4, 8, 3};
const int SERVO_ANGLE_MAX[] = {2, 4, 8, 3};
const int SERVO_TARGET_ANGLE[] = {}
const int angleRetracted = 0;
const int angleDeployed = 155;  // if changes value need to change feedback too !
int targetAngleTrans = angleDeployed;
int targetAngleArm = 105;
int angleArm = targetAngleArm;
int currentAngleTrans = 0;
int currentFeedbackTrans = 0;
int angle = 0;
int feedback = 0;
unsigned int timeout = 10000;
int minFeedback = 600;   // Based on a 12 bits resolution
int maxFeedback = 2485;  // 180° : 2485
int deployedFeedback = 2170;
int angleGripper = 180;
int speedRange = 50;
int speedWheel = 0;

// µs1 = 32, µs2 = 31, µs3 = 30, µs4 = 27
const int microswitchPotPin = 27;    // Green cable
const int microswitchPlantPin = 30;  // Blue cable


// TMC2209 : interpolation to 256 microsteps
const int homingSpeed = 5000;  // max found : 16000 steps/second
const int forkSpeed = 10000;
const int forkAcceleration = 40e3;  // in steps/second²
int homingCompleted = 0;
int maxHeight = 155;
int parkingHeight = 40;

const int stepsPerRev = 200;
const int microstepping = 8;
const float ratio = 7.5;
const int steps = round(stepsPerRev * ratio * 1.55) * microstepping;  // Reduction 7.5:1

int plantHasPriority = 1;
int targetPositionForkPlant = steps;
int targetPositionForkPot = steps;


// LED
const int LEDPin = 13;
int LEDState = HIGH;
unsigned long LEDCurrentDelay = 1000;
const unsigned long LEDDelay = 500;
const unsigned long LEDDelayWarning = 100;
unsigned long previousLED = 0;

enum servo {
  SERVO_FORKS,
  SERVO_GRIPPER,
  SERVO_ARM,
  SERVO_WHEEL
};

// States
enum states {
  FORKS,
  ARM,
  RETRACTED,
  UNCALIBRATED
};

// Names should be the same as the states enum and in the same order !
const char *stateNames[] = {
  "Forks",
  "Arm",
  "Retracted",
  "Uncalibrated"
};


// Serial input variables
const byte numChars = 10;
char receivedChars[numChars];
boolean newData = false;


// Object declaration
AccelStepper stepperPlant(AccelStepper::DRIVER, stepPlantPin, dirPlantPin);
AccelStepper stepperPot(AccelStepper::DRIVER, stepPotPin, dirPotPin);

PWMServo servoForks;
PWMServo servoGripper;
PWMServo servoArm;
PWMServo servoWheel;

states previousState = UNCALIBRATED;
states state = UNCALIBRATED;

void setup() {
  SERIAL_BEGIN(115200);
  DBG_PRINTF("\nStarting\nState : %s\n", stateNames[state]);

  analogReadResolution(12);
  pinMode(LEDPin, OUTPUT);

  pinMode(microswitchPotPin, INPUT_PULLUP);
  pinMode(microswitchPlantPin, INPUT_PULLUP);

  setupSteppers();
  if (homingCompleted) {
    setTargetForkPot(parkingHeight);
    setTargetForkPlant(parkingHeight);
    stepperPlant.moveTo(targetPositionForkPlant);
    stepperPot.moveTo(targetPositionForkPot);
    while (stepperPlant.isRunning() && stepperPot.isRunning()) {
      stepperPlant.run();
      stepperPot.run();
    }
    setupServos();
  }
}

void loop() {
  recvWithStartEndMarkers();

  if (millis() - previousLED > LEDCurrentDelay) {
    previousLED = millis();
    LEDState = !LEDState;
    digitalWrite(LEDPin, LEDState);
  }

  if (newData) {
    DBG_PRINTLN("Handling data");
    handleCommand(receivedChars);
    newData = false;
  }

  readServo(0);
  writeServos();

  handleState();


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

void handleState() {
  switch (state) {
    case UNCALIBRATED:
      DBG_PRINTLN("State : uncalibrated");
      break;
    case ARM:
      DBG_PRINTLN("State : arm");
      break;
    case FORKS:
      DBG_PRINTLN("State : forks");
      break;
    case RETRACTED:
      DBG_PRINTLN("State : retracted");
      break;
    default:
      DBG_PRINTLN("State unknown");
      break;
  }
}

/*
 * Sets plant's fork target position to the desired height within range
 * height : height in mm
 */
void setTargetForkPlant(int height) {
  if (height <= maxHeight && height >= 0) {
    int step = map(height, 0, maxHeight, 0, steps);
    targetPositionForkPlant = constrain(step, 0, steps);
  }
}

/*
 * Sets pot's fork target position to the desired height within range
 * height : height in mm
 */
void setTargetForkPot(int height) {
  if (height <= maxHeight && height >= 0) {
    int step = map(height, 0, maxHeight, 0, steps);
    targetPositionForkPot = constrain(step, 0, steps);
  }
}

/*
 * translation = 0 : forks are retracted
 * translation = 1 : forks are deployed
 */
void setTranslationForks(int translation) {
  if (translation == 0 || translation == 1) {
    targetAngleTrans = (translation) ? angleDeployed : angleRetracted;
  } else if (translation >= 2 && translation <= 180) {
    targetAngleTrans = translation;
  }
}

/*
 * angle = 0 : gripper is open
 * angle = 1 : gripper is closed
 */
void setPositionGripper(int angle) {
  if (angle == 1 || angle == 0) {
    angleGripper = angle ? 0 : 180;
  } else if (angle >= 2 && angle <= 180) {
    angleGripper = 180 - angle;
  }
}

/*
 * angle = 0 : arm is up
 * angle = 1 : arm is down
 */
void setRotationArm(int angle) {
  if (angle == 0 || angle == 1) {
    targetAngleArm = (angle) ? 0 : 105;
  } else if (angle >= 2 && angle <= 105) {
    targetAngleArm = 105 - angle;
  }
}

/*
 * speed = 0 : wheel is stopped
 * speed > 0 : wheel turns clockwise
 * speed < 0 : wheel turns counterclockwise
 */
void setSpeedWheel(int speed) {
  DBG_PRINTF("Speed : %d\n", speed);
  if (speed >= -speedRange && speed <= speedRange) {
    speedWheel = speed;
  }
}

/*
 * Read value from servo feedback at set frequency and update the angle and feedback value variables
 * forceUpdate = TRUE : forces the update
 */
void readServo(int forceUpdate) {
  static unsigned long lastServoRead = 0;
  float fb = 0;
  static int FB = 0;

  unsigned long time = millis();
  if (time - lastServoRead > 2 || forceUpdate) {
    lastServoRead = time;

    for (int x = 0; x < 5; x++) {
      fb += analogRead(feedbackPin);
    }
    FB = round(fb / 5);
  }

  feedback = FB;
  angle = map(FB, minFeedback, maxFeedback, 0, 180);  // map function does not constraint
}

void writeServos() {
  static unsigned long lastServoWheelWrite = 0;
  static unsigned long lastServoWrite = 0;
  static int pwm = 0;

  unsigned long time = millis();
  if (time - lastServoWrite > 3) {
    lastServoWrite = time;

    // Servo translation forks
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
    servoForks.write(currentAngleTrans);

    // Servo gripper
    servoGripper.write(angleGripper);

    // Servo rotation arm
    if (targetAngleArm > angleArm) {
      angleArm++;
    } else if (targetAngleArm < angleArm) {
      angleArm--;
    }
    servoArm.write(angleArm);
  }

  if (time - lastServoWheelWrite > 1) {
    lastServoWheelWrite = time;

    //DBG_PRINTF("pwm : %d ", pwm);
    if (abs(speedWheel) > pwm) {
      int value = (speedWheel > 0) ? 160 : 20;
      servoWheel.write(value);
      //DBG_PRINTLN("ON");
    } else {
      servoWheel.write(90);
      //DBG_PRINTLN("OFF");
    }
    pwm = (pwm + 1) % speedRange;
  }
}

int angleToFb(int angle) {
  float fb = minFeedback + (maxFeedback - minFeedback) * angle / 180;
  fb = constrain(fb, 0, ANALOG_RANGE);  // Can constraint feedback because can read angle a bit below 0 and higher than 180
  return round(fb);
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
      setupSteppers();
      break;
    case 'f':  // Fork plant
      plantHasPriority = 1;
      setTargetForkPlant(value);
      break;
    case 'F':  // Fork pot
      plantHasPriority = 0;
      setTargetForkPot(value);
      break;
    case 'G':  // Plants gripper
      setPositionGripper(value);
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
    case 'W':
      setSpeedWheel(value);
      break;
    default:
      DBG_PRINTLN("Command not recognized !");
      break;
  }
  DBG_PRINTF("Command received : %s\n", string);
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

  homingCompleted = homing(homingSpeed);
  if (!homingCompleted) DBG_PRINTLN("Homing forks failed");

  LEDCurrentDelay = homingCompleted ? LEDDelay : LEDDelayWarning;
  if (homingCompleted) {
    stepperPlant.setMaxSpeed(forkSpeed);  // in steps/second
    stepperPot.setMaxSpeed(forkSpeed);
  } else {
    stepperPlant.disableOutputs();
    stepperPot.disableOutputs();
  }
}

void setupServos() {
  pinMode(servoForksPin, OUTPUT);
  pinMode(servoGripperPin, OUTPUT);
  pinMode(servoArmPin, OUTPUT);
  pinMode(servoWheelPin, OUTPUT);
  pinMode(feedbackPin, INPUT);

  servoForks.attach(servoForksPin, 500, 2500);
  readServo(1);
  currentAngleTrans = angle;
  currentFeedbackTrans = feedback;

  servoGripper.attach(servoGripperPin, 500, 2500);  // datasheet 500-2500

  servoArm.attach(servoArmPin, 800, 2320);  // datasheet 700-2300. Tested from 740 to 2380 without stalling.

  servoWheel.attach(servoWheelPin, 1400, 1600);  // datasheet 500-2500

  DBG_PRINTF("Min feedback : %d\t deployed feedback : %d\n", minFeedback, deployedFeedback);
  calibration(&servoForks, angleRetracted, &minFeedback);
  calibration(&servoForks, angleDeployed, &deployedFeedback);
  DBG_PRINTF("Min feedback : %d\t deployed feedback : %d\n", minFeedback, deployedFeedback);
}

/*
 * This blocking function does the homing of the stepper motors
 * Returns true if homing is successful
*/
int homing(int speed) {
  stepperPot.setMaxSpeed(speed);  // Used by runToNewPosition()
  stepperPlant.setMaxSpeed(speed);

  DBG_PRINTLN("\nMoving pot's fork away from the switch");
  int statusPot = homingMove(&stepperPot, -speed, steps * 0.1, microswitchPotPin, true);
  if (statusPot == 2) {
    stepperPot.setCurrentPosition(0);
    stepperPot.runToNewPosition(-steps * 0.03);
  }

  DBG_PRINTLN("Moving plant's fork away from the switch");
  int statusPlant = homingMove(&stepperPlant, -speed, steps * 0.1, microswitchPlantPin, true);
  if (statusPlant == 2) {
    stepperPlant.setCurrentPosition(0);
    stepperPlant.runToNewPosition(-steps * 0.03);
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
  while ((abs(angle - a) > 1 || error > 0.01) && time < start + timeout) {
    writeServos();
    readServo(1);
    lastFb = fb;
    fb = 0.98 * fb + 0.02 * feedback;
    error = abs(fb - lastFb);
    time = millis();
    //DBG_PRINTF("angle : %d, error : %.3f, time : %d\n", angle, error, start+10000 - time);
    delay(1);
  }
  time = millis() - start;

  delay(100);  // wait for the servo to stop moving
  readServo(1);

  if (abs(*oldFb - feedback) < 25) {
    *oldFb = feedback;
    DBG_PRINTF("Calibration completed in %dms\n  New feedback value : %d\n", time, feedback);
  } else if (time >= timeout) {
    DBG_PRINTLN("Calibration failed : timeout");
  } else {
    DBG_PRINTF("Calibration failed : value outside range : %d\n", feedback);
  }
}
