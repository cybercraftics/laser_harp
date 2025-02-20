#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include "MIDIUSB.h"

#define GREEN_LASER_PIN  2
#define RED_LASER_PIN    3
#define BLUE_LASER_PIN   4
#define CLOSED_LOOP_SENSOR_ENABLE_PIN 7
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5

#define BUZZER_PIN 5

SoftwareSerial ultrasonicSensor1(8, 9);
AccelStepper stepper(AccelStepper::FULL4WIRE, 10, 12, 11, 13);

const float MAX_STEPPER_SPEED = 100000.0;
const float STEPPER_ACCEL     = 100000.0;

bool          laserActive         = false;
unsigned long laserOnStartMicros  = 0;
const unsigned long LASER_ON_DURATION = 100;
const unsigned int NUMBER_OF_STRINGS = 7;

int positionsForward[NUMBER_OF_STRINGS]  = {0, 2, 4, 6, 8, 10, 12};
int positionsReverse[NUMBER_OF_STRINGS]  = {12, 10, 8, 6, 4, 2, 0};

int  stringIndex    = 0;
bool goingForward   = true;

struct SensorState {
  bool playing;
  int  currentPitch;
};

SensorState sensor1State = { false, 0 };
SensorState sensor2State = { false, 0 };

unsigned long lastSensorCheck       = 0;
const unsigned long sensorCheckRate = 200;

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;

void handleMotorAndLaser();
void triggerAndReadUltrasonics();
void processSensorReading(unsigned int distanceMM, SensorState &state, byte channel);
int  mapDistanceToPitch(unsigned int distanceMM, byte sensorIndex);

void noteOn(byte channel, byte pitch, byte velocity);
void noteOff(byte channel, byte pitch, byte velocity);

void setup() {
  Serial.begin(9600);               // debug
  ultrasonicSensor1.begin(9600);    // ultrasonic sensor 1
  Serial1.begin(9600);              // ultrasonic sensor 2

  pinMode(GREEN_LASER_PIN, OUTPUT);
  pinMode(RED_LASER_PIN,   OUTPUT);
  pinMode(BLUE_LASER_PIN,  OUTPUT);
  analogWrite(GREEN_LASER_PIN, 0);
  analogWrite(RED_LASER_PIN,   0);
  analogWrite(BLUE_LASER_PIN,  0);

  pinMode(CLOSED_LOOP_SENSOR_ENABLE_PIN, OUTPUT);
  pinMode(CLOSED_LOOP_MOTOR_SENSOR_PIN, INPUT);

  homingRoutine();

  stepper.setMaxSpeed(MAX_STEPPER_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);
}

void loop() {
  stepper.run();

  handleMotorAndLaser();

  unsigned long now = millis();
  if (now - lastSensorCheck >= sensorCheckRate) {
    lastSensorCheck = now;
    triggerAndReadUltrasonics();
  }

  MidiUSB.flush();
}

void homingRoutine() {
  Serial.println("Homing...");

  digitalWrite(CLOSED_LOOP_SENSOR_ENABLE_PIN, HIGH);

  stepper.setMaxSpeed(50.0);
  stepper.setAcceleration(50.0);

  stepper.runToNewPosition(50);

  stepper.moveTo(-20000);
  while (digitalRead(CLOSED_LOOP_MOTOR_SENSOR_PIN) != 0) {
    stepper.run();
  }

  stepper.stop();
  stepper.setCurrentPosition(0);

  delay(100);
  stepper.setMaxSpeed(5);

  stepper.runToNewPosition(-9);
  stepper.setCurrentPosition(0);

  digitalWrite(CLOSED_LOOP_SENSOR_ENABLE_PIN, LOW);

  Serial.println("Homing complete.");
}

void handleMotorAndLaser() {
  // --- 1) FORWARD DIRECTION (laser active) ---
  if (goingForward) {
    // If the stepper has finished moving and the laser is off,
    // turn on the laser for LASER_ON_DURATION microseconds.
    if (!stepper.isRunning() && !laserActive) {
      analogWrite(GREEN_LASER_PIN, 255);
      laserActive = true;
      laserOnStartMicros = micros();
    }

    // If the laser is currently on, check how long it’s been
    if (laserActive) {
      unsigned long elapsed = micros() - laserOnStartMicros;
      if (elapsed >= LASER_ON_DURATION) {
        // Laser time is up—turn it off
        analogWrite(GREEN_LASER_PIN, 0);
        laserActive = false;

        // Advance to the next forward position
        stringIndex++;
        if (stringIndex >= 7) {
          // Finished the forward run, switch to backward
          stringIndex = 0;
          goingForward = false;
        }

        // Command the stepper to move to the new position
        if (goingForward) {
          stepper.moveTo(positionsForward[stringIndex]);
        } else {
          stepper.moveTo(positionsReverse[stringIndex]);
        }
      }
    }

  // --- 2) REVERSE DIRECTION (no laser) ---
  } else {
    // In reverse, we simply move the stepper without firing the laser.
    // Once the stepper finishes, go to the next reverse position.
    if (!stepper.isRunning()) {
      stringIndex++;
      if (stringIndex >= 7) {
        // Finished the reverse run, switch to forward
        stringIndex = 0;
        goingForward = true;
      }

      if (goingForward) {
        stepper.moveTo(positionsForward[stringIndex]);
      } else {
        stepper.moveTo(positionsReverse[stringIndex]);
      }
    }
  }
}

// --------------------------------------------------------------------------
//                       Trigger & Read Ultrasonics
// --------------------------------------------------------------------------
void triggerAndReadUltrasonics() {
  // Send "0x55" to each sensor to trigger measurement
  ultrasonicSensor1.write(0x55);   // Sensor1
  Serial1.write(0x55);    // Sensor2

  // Read sensor1 if data is available
  if (ultrasonicSensor1.available() >= 2) {
    HighByte = ultrasonicSensor1.read();
    LowByte  = ultrasonicSensor1.read();
    Len      = (HighByte << 8) + LowByte;
    // Valid range check
    if (Len > 1 && Len <= 1500) {
      processSensorReading(Len, sensor1State, 0); // channel 0
    } else {
      if (sensor1State.playing) {
        noteOff(0, sensor1State.currentPitch, 0x40);
        sensor1State.playing = false;
      }
    }
  }

  // Read sensor2 if data is available
  if (Serial1.available() >= 2) {
    HighByte = Serial1.read();
    LowByte  = Serial1.read();
    Len      = (HighByte << 8) + LowByte;
    if (Len > 1 && Len <= 1500) {
      processSensorReading(Len, sensor2State, 1); // channel 1
    } else {
      if (sensor2State.playing) {
        noteOff(1, sensor2State.currentPitch, 0x40);
        sensor2State.playing = false;
      }
    }
  }
}

void processSensorReading(unsigned int distanceMM, SensorState &state, byte channel) {
  int newPitch = mapDistanceToPitch(distanceMM, channel);

  if (!state.playing) {
    state.playing = true;
    state.currentPitch = newPitch;
    noteOn(channel, newPitch, 100); 
  } else {
    if (newPitch != state.currentPitch) {
      noteOff(channel, state.currentPitch, 0x40);
      state.currentPitch = newPitch;
      noteOn(channel, newPitch, 100);
    }
  }
}

int mapDistanceToPitch(unsigned int distanceMM, byte sensorIndex) {
  int minPitch = 48;  // C2
  int maxPitch = 72;  // C5
  int pitch    = map(distanceMM, 0, 1500, maxPitch, minPitch);

  // Optionally offset one sensor by an octave
  // if (sensorIndex == 1) pitch += 12;

  return constrain(pitch, minPitch, maxPitch);
}

// --------------------------------------------------------------------------
//                       MIDI Helper Functions
// --------------------------------------------------------------------------
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOnPacket = {
    0x09,                    // USB MIDI event (Note On)
    (byte)(0x90 | channel),  // MIDI command (Note On) + channel
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOnPacket);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOffPacket = {
    0x08,                   // USB MIDI event (Note Off)
    (byte)(0x80 | channel), // MIDI command (Note Off) + channel
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOffPacket);
}