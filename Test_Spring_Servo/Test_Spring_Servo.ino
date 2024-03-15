#include "DebugPrint.h"
#include <PWMServo.h>  // Not modified to add writeMicroseconds() C:\Users\arnau\AppData\Local\Arduino15\packages\teensy\hardware\avr\1.59.0\libraries\PWMServo

// defines servo pin number (from pin 3 to 8)
unsigned long lastServoUpdate = 0;
const int servoTransPin = 3;
const int feedbackPin = 24;  // Tx J4 feedback is on 3.3V

unsigned int timeout = 15000;

int targetAngle = 0;
int allowedAngle = 0;
int angleMeasured = 0;
int allowedFeedback = 0;
int feedbackMeasured = 0;
int minFeedback = 600;   // Based on a 12 bits resolution
int maxFeedback = 2485;  // 180° : 2485
int deployedFeedback = 2010;
const int angleRetracted = 0;
const int angleDeployed = 135;

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

PWMServo servoTrans;

void setup() {
  SERIAL_BEGIN(115200);
  //delay(100);
  //DBG_PRINTF("\n\nStarting\nSteps : %d\n", steps);

  pinMode(LEDPin, OUTPUT);

  setupServos();

  DBG_PRINTLN("Setups done");
  delay(5000);
  DBG_PRINTLN("Starting loop");
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

  static unsigned long lastMillis = millis();
  if (millis() - lastMillis > 100) {
    lastMillis = millis();
    DBG_PRINTF("Target angle : %d\tAllowed angle : %d\tMeasured angle : %d\tAllowed feedback : %d\tMeasured feedback : %d\tError : %d\n",
               targetAngle, allowedAngle, angleMeasured, allowedFeedback, feedbackMeasured, allowedFeedback - feedbackMeasured);
  }
}

/*
 * Translation = 0 : forks are retracted
 * Translation = 1 : forks are deployed
 */
void setTranslationForks(int translation) {
  if (translation == 0 || translation == 1) {
    targetAngle = (translation) ? angleDeployed : 0;
  } else if (translation >= 2 && translation <= 180) {
    targetAngle = translation;
  }
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
    case 'O':
      digitalWrite(servoTransPin, LOW);
      pinMode(servoTransPin, INPUT);
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
  static unsigned long lastServoWrite = 0;


  if (millis() - lastServoWrite > 3) {
    lastServoWrite = millis();

    if (allowedFeedback + 70 < feedbackMeasured) {  // Pulls forks inward
      allowedAngle++;
    } else if (allowedFeedback - 12 > feedbackMeasured) {  // Push forks outward
      allowedAngle--;
    } else if (targetAngle > allowedAngle) {
      allowedAngle++;
    } else if (targetAngle < allowedAngle) {
      allowedAngle--;
    }
  }
  allowedAngle = constrain(allowedAngle, 0, 180);
  allowedFeedback = angleToFb(allowedAngle);


  // Can use servo.writeMicroseconds() if want higher resolution than 1°
  servoTrans.write(allowedAngle);
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

  feedbackMeasured = FB;
  angleMeasured = map(FB, minFeedback, maxFeedback, 0, 180);  // map function does not constraint
}

void readServoEMA(int forceUpdate) {
  static unsigned long lastServoRead = 0;
  static float fb = 0;

  if (fb == 0) {
    for (int x = 0; x < 5; x++) {
      fb += analogRead(feedbackPin);
    }
    fb = round(fb / 5);
  }

  unsigned long time = millis();
  if (time - lastServoRead > 2 || forceUpdate) {
    lastServoRead = time;
    fb = 0.99 * fb + 0.01 * analogRead(feedbackPin);
  }

  feedbackMeasured = round(fb);
  angleMeasured = map(feedbackMeasured, minFeedback, maxFeedback, 0, 180);  // map function does not constraint
}


int angleToFb(int angle) {
  float fb = minFeedback + (maxFeedback - minFeedback) * angle / 180;
  fb = constrain(fb, 0, 4096);  // Can constraint feedback because can read angle a bit below 0 and higher than 180
  return round(fb);
}

int fbToAngle(int fb) {
  float angle = (fb - minFeedback) / (maxFeedback - minFeedback) * 180;
  //angle = contraint(angle, 0, 180); // Can't constraint angle !
  return round(angle);
}

/*
 * Does the calibration of the servo feedback value at angle specified.
 * If it fails, leave default values
*/
void calibration(PWMServo *servo, int a, int *oldFb) {
  DBG_PRINTF("\n\nStarting calibration for angle : %d, old feedback : %d\n", a, *oldFb);
  readServo(1);
  DBG_PRINTF("Measured angle : %d, measured feedback : %d\n", angleMeasured, feedbackMeasured);
  unsigned long start = millis();
  unsigned long time = start;
  float fb = feedbackMeasured;
  float lastFb = *oldFb;
  //servo.write(angleMeasured); // library already constrain written angle value
  targetAngle = a;

  float error = 20.0;
  while ((abs(angleMeasured - a) > 1 || error > 0.01) && time < start + timeout) {
    writeServos();
    readServo(0);
    lastFb = fb;
    fb = 0.98 * fb + 0.02 * feedbackMeasured;
    error = abs(fb - lastFb);
    time = millis();
    //DBG_PRINTF("angleMeasured : %d, error : %.3f, time : %d\n", angleMeasured, error, start+10000 - time);
    delay(1);
  }
  time = millis() - start;
  
  writeServos();
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

void setupServos() {
  pinMode(servoTransPin, OUTPUT);
  pinMode(feedbackPin, INPUT);

  servoTrans.attach(servoTransPin, 500, 2500);
  analogReadResolution(12);

  readServo(1);
  allowedAngle = angleMeasured;
  allowedFeedback = feedbackMeasured;

  DBG_PRINTF("Min feedback : %d\t max feedback : %d\n", minFeedback, maxFeedback);
  calibration(&servoTrans, angleRetracted, &minFeedback);
  calibration(&servoTrans, angleDeployed, &deployedFeedback);
  DBG_PRINTF("Min feedback : %d\t max feedback : %d\n", minFeedback, maxFeedback);
}
