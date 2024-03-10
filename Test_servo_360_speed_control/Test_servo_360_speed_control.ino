#include "DebugPrint.h"
#include <PWMServo.h>


// defines servo pin number (from pin 3 to 8)
unsigned long lastServoUpdate = 0;
const int servoWheelPin = 6;
int speedRange = 50;
int speedWheel = 0;


// LED
const int LEDPin = 13;
int state = HIGH;
unsigned long LEDCurrentDelay = 500;
unsigned long previousLED = 0;


// Serial input variables
const byte numChars = 10;
char receivedChars[numChars];
boolean newData = false;


PWMServo servoWheel;


void setup() {
  SERIAL_BEGIN(115200);

  pinMode(LEDPin, OUTPUT);

  setupServos();

  DBG_PRINTLN("\nSetups done");
}

void loop() {
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
}

void setSpeedWheel(int speed) {
  DBG_PRINTF("Speed : %d\n", speed);
  if (speed >= -speedRange && speed <= speedRange) {
    speedWheel = speed;
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
    case 'W':
      setSpeedWheel(value);
      break;
    default:
      DBG_PRINTLN("Command not recognized !");
      break;
  }
  DBG_PRINTF("Command received : %s\n", string);
}

void writeServos() {
  static unsigned long lastServoWrite = 0;
  static int pwm = 0;

  int time = millis();
  if (time - lastServoWrite > 1) {
    lastServoWrite = time;

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

void setupServos() {
  pinMode(servoWheelPin, OUTPUT);
  servoWheel.attach(servoWheelPin, 1400, 1600);  // datasheet 500-2500
}