#include <Stepper.h>

// #define LASER_PIN 12
#define LASER_PIN_1 2

const float STEPS_PER_REVOLUTION = 1000;
const unsigned long LASER_ON_DURATION = 1; // Laser on for 1 ms

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
  stepper.setSpeed(40);
  pinMode(LASER_PIN_1, OUTPUT);
}

void loop() {
  // Forward 7 times
  for (int i = 0; i < 5; i++) {
      stepper.step(2);
      delay(4);
      digitalWrite(LASER_PIN_1, HIGH);
      delay(1);
      digitalWrite(LASER_PIN_1, LOW);
  }
    
  stepper.step(-10);
}