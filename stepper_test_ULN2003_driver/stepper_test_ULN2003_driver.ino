#include <Stepper.h>

#define LASER_PIN 2
#define LASER_DETECTOR_PIN A0

const float STEPS_PER_REVOLUTION = 200;
const unsigned long LASER_ON_DURATION = 1; // Laser on for 1 ms

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(220);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(LASER_DETECTOR_PIN, INPUT);
}

void loop() {
  // Forward 7 times
  for (int i = 0; i < 5; i++) {
      stepper.step(2);
      if (i == 0) {
        delay(5);
      } else {
        delay(4);
      }
      digitalWrite(LASER_PIN, HIGH);
      delay(1);
      digitalWrite(LASER_PIN, LOW);
  }
    
  stepper.step(-10);

  Serial.println(analogRead(LASER_DETECTOR_PIN));
}