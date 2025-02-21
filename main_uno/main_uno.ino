#include <Stepper.h>
#include <Wire.h>

#define GREEN_LASER_PIN  2
#define RED_LASER_PIN    3
#define BLUE_LASER_PIN   4
#define CLOSED_LOOP_VCC_PIN 7
#define CLOSED_LOOP_DO_PIN A3

#define BUZZER_PIN 5

const unsigned int STEPS_PER_REVOLUTION = 200;
const unsigned int STEPPER_SPEED = 220;
const unsigned int STEPPER_HOMING_SPEED = 10;
const unsigned long LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;
const unsigned int HOME_POSITION = -5;
int counter = 0;

Stepper stepper(STEPS_PER_REVOLUTION, 10, 12, 11, 13);

void handleMotorAndLaser();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(GREEN_LASER_PIN, OUTPUT);
  pinMode(RED_LASER_PIN,   OUTPUT);
  pinMode(BLUE_LASER_PIN,  OUTPUT);
  analogWrite(GREEN_LASER_PIN, 0);
  analogWrite(RED_LASER_PIN,   0);
  analogWrite(BLUE_LASER_PIN,  0);

  pinMode(CLOSED_LOOP_VCC_PIN, OUTPUT);
  pinMode(CLOSED_LOOP_DO_PIN, INPUT);

  homingRoutine();

  stepper.setSpeed(STEPPER_SPEED);
}

void loop() {
  handleMotorAndLaser();
  Wire.beginTransmission(0x08); // Start communication with device at address 8
  Wire.write(counter); // Send counter value
  Wire.endTransmission(); // Stop transmission
  
  counter++; // Increment counter
}

void homingRoutine() {
  Serial.println("Homing...");

  digitalWrite(CLOSED_LOOP_VCC_PIN, HIGH);

  stepper.setSpeed(STEPPER_HOMING_SPEED);
  stepper.step(50);
  while (digitalRead(CLOSED_LOOP_DO_PIN) != 0) {
    stepper.step(-1);
  }

  digitalWrite(CLOSED_LOOP_VCC_PIN, LOW);

  stepper.step(HOME_POSITION);

  Serial.println("Homing complete.");
}

void handleMotorAndLaser() {
  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    stepper.step(2);
    
    if (i == 3) {
      delayMicroseconds(1000);
    }
    if (i == 4 || i == 5 || i == 6) {
      delayMicroseconds(800);
    }

    analogWrite(GREEN_LASER_PIN, 255);

    delayMicroseconds(LASER_ON_DURATION);

    analogWrite(GREEN_LASER_PIN, 0);
  }
  stepper.step(-NUMBER_OF_STRINGS * 2);
}
