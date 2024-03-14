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
const int feedbackPin = 24;  // Tx J4, feedback from servo is on 3.3V
const int SERVO_PIN[] = { 3, 4, 5, 6 };
// Changing the rotation direction is done in servo.attach() by reversing the min pulse width and max pulse width
const int SERVO_MIN[] = { 0, 0, 0, -50 };
const int SERVO_DEPLOYED[] = { 155, 180, 105, 0 };  // if changes value fork SERVO_FORKS need to change feedback too !
const int SERVO_MAX[] = { 170, 180, 115, 50 };      // SERVO_MAX[3] = -SERVO_MIN[3]
int SERVO_TARGET[] = { 155, 0, 0, 90 };
int SERVO_ALLOWED[] = { 0, 0, 0, 0 };

unsigned long lastServoUpdate = 0;
unsigned int timeout = 10000;
int allowedFeedbackForks = 0;
int angleMeasured = 0;
int feedbackMeasured = 0;
int minFeedback = 600;  // Based on a 12 bits resolution
int deployedFeedback = 2225;
int maxFeedback = 2485;  // 180° : 2485


// µs1 = 32, µs2 = 31, µs3 = 30, µs4 = 27
const int microswitchPotPin = 27;    // Green cable
const int microswitchPlantPin = 30;  // Blue cable


// TMC2209 : interpolation to 256 microsteps
const int homingSpeed = 5000;  // max found : 16000 steps/second
const int forkSpeed = 4000;
const int forkAcceleration = 40e3;  // in steps/second²
int homingCompleted = 0;
int maxHeight = 100;
int parkingHeight = 50;

const int stepsPerRev = 200;
const int microstepping = 8;
const float ratio = 7.5;
const int steps = round(stepsPerRev * ratio * 1.55) * microstepping;  // Reduction 7.5:1

int armHasPriority = 0;
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


void setup() {
  SERIAL_BEGIN(115200);
  //DBG_PRINTF("\nStarting\nState : %s\n", stateNames[state]);

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

  handleSystem();

  readServo(0);
  writeServos();

  stepperPlant.run();
  stepperPot.run();
}


void handleSystem() {
  static int lastPriority = 0;
  if (lastPriority != armHasPriority) {
    lastPriority = armHasPriority;
    DBG_PRINTF("Arm has priority : %s\n", armHasPriority ? "YES" : "NO");
  }

  if (armHasPriority) {
    setTargetForkPlant(parkingHeight);
    setTargetForkPot(parkingHeight);
    stepperPlant.moveTo(targetPositionForkPlant);
    stepperPot.moveTo(targetPositionForkPot);

    if (!stepperPlant.isRunning() && !stepperPot.isRunning()) {
      SERVO_TARGET[SERVO_FORKS] = SERVO_MIN[SERVO_FORKS];
    }

    if (SERVO_ALLOWED[SERVO_FORKS] > SERVO_MIN[SERVO_FORKS] + 5) {
      // bad to directly write the allowed values
      SERVO_ALLOWED[SERVO_ARM] = SERVO_MIN[SERVO_ARM];
      SERVO_ALLOWED[SERVO_WHEEL] = SERVO_DEPLOYED[SERVO_WHEEL];
    }
  } else {  // fork has priority
    SERVO_TARGET[SERVO_ARM] = SERVO_MIN[SERVO_ARM];
    SERVO_TARGET[SERVO_WHEEL] = SERVO_DEPLOYED[SERVO_WHEEL];

    if (SERVO_ALLOWED[SERVO_ARM] > SERVO_MIN[SERVO_ARM] + 3) {
      // bad to directly write the allowed value
      SERVO_ALLOWED[SERVO_FORKS] = SERVO_MIN[SERVO_FORKS];
    }

    if (SERVO_ALLOWED[SERVO_ARM] <= SERVO_MIN[SERVO_ARM]) {
      int posPlant = targetPositionForkPlant;
      int posPot = targetPositionForkPot;
      if (plantHasPriority) {
        posPot = min(targetPositionForkPot, targetPositionForkPlant);
      } else {
        posPlant = max(targetPositionForkPot, targetPositionForkPlant);
      }
      stepperPlant.moveTo(posPlant);
      stepperPot.moveTo(posPot);
    }
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
    SERVO_TARGET[SERVO_FORKS] = (translation) ? SERVO_DEPLOYED[SERVO_FORKS] : SERVO_MIN[SERVO_FORKS];
    armHasPriority = translation ? 0 : 1;
  } else if (translation >= 2 && translation <= SERVO_MAX[SERVO_FORKS]) {
    SERVO_TARGET[SERVO_FORKS] = translation;
    armHasPriority = (translation > 5) ? 0 : 1;
  }
}

/*
 * angle = 0 : gripper is open
 * angle = 1 : gripper is closed
 */
void setPositionGripper(int angle) {
  if (angle == 1 || angle == 0) {
    SERVO_TARGET[SERVO_GRIPPER] = angle ? SERVO_DEPLOYED[SERVO_GRIPPER] : SERVO_MIN[SERVO_GRIPPER];
  } else if (angle >= 2 && angle <= 180) {
    SERVO_TARGET[SERVO_GRIPPER] = angle;
  }
}

/*
 * angle = 0 : arm is up
 * angle = 1 : arm is down
 */
void setRotationArm(int angle) {
  if (angle == 0 || angle == 1) {
    SERVO_TARGET[SERVO_ARM] = (angle) ? SERVO_DEPLOYED[SERVO_ARM] : SERVO_MIN[SERVO_ARM];
    armHasPriority = angle ? 1 : 0;
  } else if (angle >= 2 && angle <= SERVO_DEPLOYED[SERVO_ARM]) {
    SERVO_TARGET[SERVO_ARM] = angle;
    armHasPriority = (angle > 5) ? 1 : 0;
  }
}

/*
 * speed = 0 : wheel is stopped
 * speed > 0 : wheel turns clockwise
 * speed < 0 : wheel turns counterclockwise
 */
void setSpeedWheel(int speed) {
  DBG_PRINTF("Speed : %d\n", speed);
  if (speed >= SERVO_MIN[SERVO_WHEEL] && speed <= SERVO_MAX[SERVO_WHEEL]) {
    SERVO_TARGET[SERVO_WHEEL] = speed;
  }
}

/*
 * Read value from servo feedback at set frequency and update the angle and feedback value variables
 * forceUpdate = TRUE : forces the update
 * TODO check if EMA could be a better solution to read the values
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

  feedbackMeasured = FB;
  angleMeasured = map(FB, minFeedback, maxFeedback, 0, 180);  // map function does not constraint
}

void writeServos() {
  static unsigned long lastServoWheelWrite = 0;
  static unsigned long lastServoWrite = 0;
  static int pwm = 0;

  unsigned long time = millis();
  if (time - lastServoWrite > 20) {  // 3
    lastServoWrite = time;

    // Servo translation forks
    if (allowedFeedbackForks + 70 < feedbackMeasured) {  // Pulls forks inward
      SERVO_ALLOWED[SERVO_FORKS]++;
    } else if (allowedFeedbackForks - 16 > feedbackMeasured) {  // Push forks outward
      SERVO_ALLOWED[SERVO_FORKS]--;
    } else if (SERVO_TARGET[SERVO_FORKS] > SERVO_ALLOWED[SERVO_FORKS]) {
      SERVO_ALLOWED[SERVO_FORKS]++;
    } else if (SERVO_TARGET[SERVO_FORKS] < SERVO_ALLOWED[SERVO_FORKS]) {
      SERVO_ALLOWED[SERVO_FORKS]--;
    }
    SERVO_ALLOWED[SERVO_FORKS] = constrain(SERVO_ALLOWED[SERVO_FORKS], SERVO_MIN[SERVO_FORKS], SERVO_MAX[SERVO_FORKS]);
    allowedFeedbackForks = angleToFb(SERVO_ALLOWED[SERVO_FORKS]);
    servoForks.write(SERVO_ALLOWED[SERVO_FORKS]);

    // Servo gripper
    servoGripper.write(SERVO_TARGET[SERVO_GRIPPER]);

    // Servo rotation arm
    if (SERVO_TARGET[SERVO_ARM] > SERVO_ALLOWED[SERVO_ARM]) {
      SERVO_ALLOWED[SERVO_ARM]++;
    } else if (SERVO_TARGET[SERVO_ARM] < SERVO_ALLOWED[SERVO_ARM]) {
      SERVO_ALLOWED[SERVO_ARM]--;
    }
    servoArm.write(SERVO_ALLOWED[SERVO_ARM]);
  }

  time = millis();
  if (time - lastServoWheelWrite > 1) {
    lastServoWheelWrite = time;

    //DBG_PRINTF("pwm : %d ", pwm);
    if (abs(SERVO_TARGET[SERVO_WHEEL]) > pwm) {
      int value = (SERVO_TARGET[SERVO_WHEEL] > 0) ? 160 : 20;
      servoWheel.write(value);
    } else {
      servoWheel.write(90);
    }
    pwm = (pwm + 1) % SERVO_MAX[SERVO_WHEEL];
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
  pinMode(SERVO_PIN[SERVO_FORKS], OUTPUT);
  pinMode(SERVO_PIN[SERVO_GRIPPER], OUTPUT);
  pinMode(SERVO_PIN[SERVO_ARM], OUTPUT);
  pinMode(SERVO_PIN[SERVO_WHEEL], OUTPUT);
  pinMode(feedbackPin, INPUT);

  servoForks.attach(SERVO_PIN[SERVO_FORKS], 500, 2500);
  readServo(1);
  SERVO_ALLOWED[SERVO_FORKS] = angleMeasured;
  allowedFeedbackForks = feedbackMeasured;

  servoGripper.attach(SERVO_PIN[SERVO_GRIPPER], 2500, 500);  // datasheet 500-2500

  //servoArm.attach(SERVO_PIN[SERVO_ARM], 2320, 800);  // datasheet 700-2300. Tested from 740 to 2380 without stalling.
  servoArm.attach(SERVO_PIN[SERVO_ARM], 2500, 500);


  servoWheel.attach(SERVO_PIN[SERVO_WHEEL], 1400, 1600);  // datasheet 500-2500

  DBG_PRINTF("Min feedback : %d\t deployed feedback : %d\n", minFeedback, deployedFeedback);
  calibration(&servoForks, SERVO_MIN[SERVO_FORKS], &minFeedback);
  calibration(&servoForks, SERVO_DEPLOYED[SERVO_FORKS], &deployedFeedback);
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
  DBG_PRINTF("Current angle : %d, current feedback : %d\n", angleMeasured, feedbackMeasured);
  unsigned long start = millis();
  unsigned long time = start;
  float fb = feedbackMeasured;
  float lastFb = *oldFb;
  //servo.write(angleMeasured); // library already constrain written angle value
  SERVO_TARGET[SERVO_FORKS] = a;

  float error = 20.0;
  while ((abs(angleMeasured - a) > 1 || error > 0.01) && time < start + timeout) {
    writeServos();
    readServo(1);
    lastFb = fb;
    fb = 0.98 * fb + 0.02 * feedbackMeasured;
    error = abs(fb - lastFb);
    time = millis();
    //DBG_PRINTF("angleMeasured : %d, error : %.3f, time : %d\n", angleMeasured, error, start+10000 - time);
    delay(1);
  }
  time = millis() - start;

  delay(100);  // wait for the servo to stop moving
  readServo(1);

  if (abs(*oldFb - feedbackMeasured) < 25) {
    *oldFb = feedbackMeasured;
    DBG_PRINTF("Calibration completed in %dms\n  New feedback value : %d\n", time, feedbackMeasured);
  } else if (time >= timeout) {
    DBG_PRINTLN("Calibration failed : timeout");
  } else {
    DBG_PRINTF("Calibration failed : value outside range : %d\n", feedbackMeasured);
  }
}
