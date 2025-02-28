#include <Stepper.h>
#include <Wire.h>

#define GREEN_LASER_PIN 2
#define RED_LASER_PIN 3
#define BLUE_LASER_PIN 4
#define LASER_ENABLE_PIN 5
#define CLOSED_LOOP_DO_PIN 6
#define CLOSED_LOOP_VCC_PIN 7

const unsigned int STEPS_PER_REVOLUTION = 200;
const unsigned int STEPPER_SPEED = 220;
const unsigned int STEPPER_HOMING_SPEED = 10;
const unsigned long LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;
const unsigned int HOME_POSITION = -8;
const unsigned int LIGHT_DETECTOR_PINS[] = {A0, A1, A2, A3};

Stepper stepper(STEPS_PER_REVOLUTION, 10, 12, 11, 13);

unsigned int sensorBaseline[4];
unsigned long sensorSum[4];
int calibrationCyclesLeft = 10;
byte interruptedBeams = 0;

void homingRoutine();
void handleMotorAndLaser();
void handleBeamsInterrupt(int currentBeam);

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
  pinMode(CLOSED_LOOP_DO_PIN,  INPUT);

  homingRoutine();

  // Turn on laser fully
  analogWrite(LASER_ENABLE_PIN, 255);

  // ******** SPEED UP ADC ********
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // prescaler = 16 instead of default 128

  stepper.setSpeed(STEPPER_SPEED);
}

void loop() {
  handleMotorAndLaser();

  Wire.beginTransmission(0x08);
  Wire.write(interruptedBeams);
  Wire.endTransmission();
}

void homingRoutine() {
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
}

void handleMotorAndLaser() {
  interruptedBeams = 0;
  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    stepper.step(2);
    if (i == 3 || i == 4 || i == 6) {
      delayMicroseconds(800);
    }
    analogWrite(GREEN_LASER_PIN, 255);
    handleBeamsInterrupt(i);
    analogWrite(GREEN_LASER_PIN, 0);
  }
  stepper.step(-NUMBER_OF_STRINGS * 2);

  if (calibrationCyclesLeft > 0) {
    calibrationCyclesLeft--;
    if (calibrationCyclesLeft == 0) {
      for (int j = 0; j < 4; j++) {
        sensorBaseline[j] = sensorSum[j] / (10UL * NUMBER_OF_STRINGS);
      }
    }
  }
}

void handleBeamsInterrupt(int currentBeam) {
  bool beamInterrupted = false;
  for (int j = 0; j < 4; j++) {
    int val = analogRead(LIGHT_DETECTOR_PINS[j]);
    if (calibrationCyclesLeft > 0) {
      sensorSum[j] += val;
    }
    else {
      if (val > sensorBaseline[j] + 5) {
        beamInterrupted = true;
      }
    }
  }
  if (beamInterrupted) {
    interruptedBeams |= (1 << currentBeam);
  }
}