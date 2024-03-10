#include <AccelStepper.h>

// defines pins numbers Arduino nano
// max freq step pin : setMinPulseWidth(20) : 5.2kHz
//                     setMinPulseWidth(2) : 5.92kHz
/*const int stepPin = 5;
const int dirPin = 6;
const int potPin = A1;*/

// defines pins numbers Wemos d1 mini pro 
// @160MHz
// max freq step pin : setMinPulseWidth(20) : 8.69kHz
//                     setMinPulseWidth(2) : 10.31kHz

// @ 80MHz
// max freq step pin : setMinPulseWidth(20) : 7.02kHz
//                     setMinPulseWidth(2) : 8.04kHz
//const int stepPin = D5;
//const int dirPin = D6;
//const int potPin = A0;

// defines pins numbers Teensy 4.1
// max freq step pin : setMinPulseWidth(20) : 40kHz
//                     setMinPulseWidth(2) : 62+kHz
const int stepPin = 5;
const int dirPin = 6;
const int potPin = A9;

const int speedMax = 80000;

int potValue = 0;
int targetSpeed = 0;

const unsigned long potReadDelay = 50;
unsigned long previousPotRead = 0;  // will store last time pot was updated

// Define some steppers and the pins the will use
AccelStepper stepperPot(AccelStepper::DRIVER, stepPin, dirPin);  // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//AccelStepper stepper2(AccelStepper::DRIVER, 6, 7, 8, 9);

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nStarting");
  Serial.print("Max speed : ");
  Serial.println(speedMax);

  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);

  stepperPot.setMinPulseWidth(2);
  stepperPot.setMaxSpeed(speedMax);  // in steps/second
  stepperPot.setAcceleration(100*speedMax);
}

void loop() {
  int value = analogRead(potPin);
  if (millis() - previousPotRead >= potReadDelay) { // Correct way to avoid overflow problems
    previousPotRead = millis();
    if (abs(value - potValue) > 3) {  // Avoid pot jitter
      targetSpeed = map(value, 0, 1023, 0, speedMax);
      stepperPot.setSpeed(targetSpeed);
      //Serial.printf("New target position : %d\n", steps);
    }
  }

  stepperPot.runSpeed();
}
