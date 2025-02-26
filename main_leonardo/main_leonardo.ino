#include <SoftwareSerial.h>
#include "MIDIUSB.h"
#include <Wire.h>

SoftwareSerial ultrasonicSensor1(8, 9); // RX, TX

struct SensorState {
  bool playing;
  int  currentPitch;
};

SensorState sensor1State = { false, 0 };
SensorState sensor2State = { false, 0 };

unsigned long lastSensorCheck       = 0;
const unsigned long sensorCheckRate = 50; // Reduced from 200 ms for faster responsiveness

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;
volatile int receivedData = 0;

void triggerAndReadUltrasonics();
void processSensorReading(unsigned int distanceMM, SensorState &state, byte channel);
int  mapDistanceToPitch(unsigned int distanceMM, byte sensorIndex);

void noteOn(byte channel, byte pitch, byte velocity);
void noteOff(byte channel, byte pitch, byte velocity);

void setup() {
  Serial.begin(9600);               // debug
  ultrasonicSensor1.begin(9600);     // ultrasonic sensor 1
  Serial1.begin(9600);              // ultrasonic sensor 2
  Wire.begin(0x08); // Join I2C bus as Slave with address 8
  Wire.onReceive(receiveEvent); // Register function to handle received data
}

void loop() {
  unsigned long now = millis();
  if (now - lastSensorCheck >= sensorCheckRate) {
    lastSensorCheck = now;
    triggerAndReadUltrasonics();
  }

  MidiUSB.flush();
}

void receiveEvent(int bytesReceived) {
  while (Wire.available()) {
    receivedData = Wire.read(); // Read incoming data
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}

// --------------------------------------------------------------------------
//                       Trigger & Read Ultrasonics
// --------------------------------------------------------------------------
void triggerAndReadUltrasonics() {
  // Send "0x55" to each sensor to trigger measurement
  ultrasonicSensor1.write(0x55);   // Sensor1
  Serial1.write(0x55);             // Sensor2

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

  if (sensorIndex == 1) pitch += 12;

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