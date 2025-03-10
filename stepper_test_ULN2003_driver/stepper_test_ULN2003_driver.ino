#include <Stepper.h>

#define LASER_PIN 4
#define LASER_DETECTOR_PIN A0

const float STEPS_PER_REVOLUTION = 200;
const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(220);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(LASER_DETECTOR_PIN, INPUT);
}

void loop() {
  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
      stepper.step(2);
      
      if (i != 2 && i != 5) {
        delayMicroseconds(500);
      }

      analogWrite(LASER_PIN, 255);
      delayMicroseconds(LASER_ON_DURATION);
      analogWrite(LASER_PIN, 0);
      
  }
  stepper.step(-NUMBER_OF_STRINGS * 2);
}