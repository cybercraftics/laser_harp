#include <Stepper.h>

// #define LASER_PIN 12
#define LASER_PIN_1 2
#define LASER_PIN_2 3
#define LASER_PIN_3 4

const float STEPS_PER_REVOLUTION = 1000;
const unsigned long LASER_ON_DURATION = 1; // Laser on for 1 ms

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
  stepper.setSpeed(50);
  pinMode(LASER_PIN_1, OUTPUT);
  pinMode(LASER_PIN_2, OUTPUT);
  pinMode(LASER_PIN_3, OUTPUT);
}

void loop() {
  // Forward 7 times
  for (int i = 0; i < 4; i++) {
    stepper.step(2);
    delay(10);
    digitalWrite(LASER_PIN_1, HIGH);
    digitalWrite(LASER_PIN_2, HIGH);
    digitalWrite(LASER_PIN_3, HIGH);
    delay(3);
    digitalWrite(LASER_PIN_1, LOW);
    digitalWrite(LASER_PIN_2, LOW);
    digitalWrite(LASER_PIN_3, LOW);
  }

  // Backward 7 times
  for (int i = 0; i < 4; i++) {
    stepper.step(-2);
    delay(10);
    digitalWrite(LASER_PIN_1, HIGH);
    digitalWrite(LASER_PIN_2, HIGH);
    digitalWrite(LASER_PIN_3, HIGH);
    delay(3);
    digitalWrite(LASER_PIN_1, LOW);
    digitalWrite(LASER_PIN_2, LOW);
    digitalWrite(LASER_PIN_3, LOW);
  }
}