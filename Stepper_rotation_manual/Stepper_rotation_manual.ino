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


// µs1 = 32, µs2 = 31, µs3 = 30, µs4 = 27
const int microswitchPotPin = 27;    // Green cable
const int microswitchPlantPin = 30;  // Blue cable


// TMC2209 : interpolation to 256 microsteps
const int forkSpeed = 5000;
const int forkAcceleration = 10e3;  // in steps/second²

const int stepsPerRev = 200;
const int microstepping = 8;
const float ratio = 7.5;
const int steps = round(stepsPerRev * ratio * 1.1) * microstepping;  // Reduction 7.5:1

int targetPositionForkPlant = 100000;
int targetPositionForkPot = 100000;


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


void setup() {
  SERIAL_BEGIN(115200);
  //DBG_PRINTF("\n\nStarting\nSteps : %d\n", steps);

  pinMode(LEDPin, OUTPUT);

  pinMode(microswitchPotPin, INPUT_PULLUP);
  pinMode(microswitchPlantPin, INPUT_PULLUP);

  setupSteppers();

  DBG_PRINTLN("Setups done");
  DBG_PRINTF("\nCURRENT POSITION PLANT : %d\n", targetPositionForkPlant);
  DBG_PRINTF("CURRENT POSITION POT : %d\n", targetPositionForkPot);
  DBG_PRINTLN("Higher numbers wind the cable");
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

  stepperPlant.moveTo(targetPositionForkPlant);
  stepperPot.moveTo(targetPositionForkPot);

  stepperPlant.run();
  stepperPot.run();
}


/*
 * Moves plant's fork to the desired height within range
 * height : height in mm
 */
void setTargetForkPlant(int steps) {
  targetPositionForkPlant = steps;
}

void setTargetForkPot(int steps) {
  targetPositionForkPot = steps;
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
    case 'D':
      stepperPlant.disableOutputs();
      stepperPot.disableOutputs();
      break;
    case 'E':
      stepperPlant.enableOutputs();
      stepperPot.enableOutputs();
      break;
    case 'F':  // Fork pot
      setTargetForkPot(value);
      break;
    case 'f':  // Fork plant
      setTargetForkPlant(value);
      break;
    case 'S':
      setupSteppers();
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
  stepperPlant.setMaxSpeed(forkSpeed);
  stepperPlant.setAcceleration(forkAcceleration);
  stepperPlant.setCurrentPosition(targetPositionForkPlant);

  stepperPot.setMinPulseWidth(2);
  stepperPot.setEnablePin(enPotPin);
  stepperPot.setPinsInverted(true, false, true);
  stepperPot.enableOutputs();
  stepperPot.setMaxSpeed(forkSpeed);
  stepperPot.setAcceleration(forkAcceleration);
  stepperPot.setCurrentPosition(targetPositionForkPot);

  LEDCurrentDelay = LEDDelay;
}
