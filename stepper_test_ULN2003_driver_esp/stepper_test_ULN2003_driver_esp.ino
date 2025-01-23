#include <Stepper.h>

const float STEPS_PER_REVOLUTION = 200;
const unsigned long LASER_ON_DURATION = 1; // Laser on for 1 ms

Stepper stepper(STEPS_PER_REVOLUTION, 0, 2, 1, 3);

void setup() {
  stepper.setSpeed(220);
}

void loop() {

    stepper.step(200);

}