#include <Stepper.h>
#include <Wire.h>

#define GREEN_LASER_PIN  2
#define RED_LASER_PIN    3
#define BLUE_LASER_PIN   4
#define LASER_ENABLE_PIN 5
#define CLOSED_LOOP_DO_PIN 6
#define CLOSED_LOOP_VCC_PIN 7

const unsigned int STEPS_PER_REVOLUTION = 200;
const unsigned int STEPPER_SPEED = 220;
const unsigned int STEPPER_HOMING_SPEED = 10;
const unsigned long LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;
const unsigned int HOME_POSITION = -8;
int counter = 0;

Stepper stepper(STEPS_PER_REVOLUTION, 10, 12, 11, 13);

void handleMotorAndLaser();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(GREEN_LASER_PIN, OUTPUT);
  pinMode(RED_LASER_PIN,   OUTPUT);
  pinMode(BLUE_LASER_PIN,  OUTPUT);
  pinMode(LASER_ENABLE_PIN, OUTPUT);
  
  analogWrite(GREEN_LASER_PIN, 0);
  analogWrite(RED_LASER_PIN,   0);
  analogWrite(BLUE_LASER_PIN,  0);

  pinMode(CLOSED_LOOP_VCC_PIN, OUTPUT);
  pinMode(CLOSED_LOOP_DO_PIN, INPUT);

  homingRoutine();

  analogWrite(LASER_ENABLE_PIN, 255);

  stepper.setSpeed(STEPPER_SPEED);
}

void loop() {
  handleMotorAndLaser();
  Wire.beginTransmission(0x08);
  Wire.write(counter);
  Wire.endTransmission();
  
  counter++;
}

void homingRoutine() {
  Serial.println("Homing...");

  digitalWrite(CLOSED_LOOP_VCC_PIN, HIGH);

  stepper.setSpeed(STEPPER_HOMING_SPEED);
  stepper.step(40);
  int maxSteps = 40;
  while (digitalRead(CLOSED_LOOP_DO_PIN) != 0 && maxSteps > 0) {
    stepper.step(-1);
    maxSteps--;
  }

  digitalWrite(CLOSED_LOOP_VCC_PIN, LOW);

  stepper.step(HOME_POSITION);

  Serial.println("Homing complete.");
}

void handleMotorAndLaser() {
  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    stepper.step(2);
    
    if (i == 3 || i == 4 || i == 6) {
      delayMicroseconds(800);
    }

    analogWrite(GREEN_LASER_PIN, 255);

    delayMicroseconds(LASER_ON_DURATION);

    analogWrite(GREEN_LASER_PIN, 0);
  }
  stepper.step(-NUMBER_OF_STRINGS * 2);
}
