#include <Stepper.h>

const float STEPS_PER_REVOLUTION = 50;

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
  stepper.setSpeed(1000);
}

void loop() {
    stepper.step(18);
    delay(1);
    stepper.step(-18);
    delay(1);
}



