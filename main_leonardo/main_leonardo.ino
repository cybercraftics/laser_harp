#include <SoftwareSerial.h>
#include "MIDIUSB.h"
#include <Wire.h>

SoftwareSerial ultrasonicSensor1(8, 9);

struct BeamState {
  bool playing;
  int  currentPitch;
  unsigned long offStartTime;
};

const int NUM_BEAMS = 7;
BeamState beamStates[NUM_BEAMS];

const int baseNotes[NUM_BEAMS] = {36, 38, 40, 41, 43, 45, 47};

const unsigned long OFF_DEBOUNCE_MS = 10; 

unsigned int sensor1Distance = 0;
unsigned int sensor2Distance = 0;

unsigned int lastSensor1Filtered = 0;
unsigned int lastSensor2Filtered = 0;

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;

volatile byte receivedData = 0;

unsigned long lastSensorCheck       = 0;
const unsigned long sensorCheckRate = 20;

int oldOffsetChannel0 = -1; 
int oldOffsetChannel1 = -1;

void triggerAndReadUltrasonics();
int getOctaveOffset(unsigned int distanceMM);
int stabilizeOffset(int newOffset, int &oldOffset);

void noteOn(byte channel, byte pitch, byte velocity);
void noteOff(byte channel, byte pitch, byte velocity);

void receiveEvent(int bytesReceived);

void setup() {
  Serial.begin(9600);
  ultrasonicSensor1.begin(9600); 
  Serial1.begin(9600); 

  for (int i = 0; i < NUM_BEAMS; i++) {
    beamStates[i].playing       = false;
    beamStates[i].currentPitch  = 0;
    beamStates[i].offStartTime  = 0;
  }

  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);

  Serial.println("Slave ready, listening for beam data...");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorCheck >= sensorCheckRate) {
    lastSensorCheck = now;
    triggerAndReadUltrasonics();
  }

  byte stableData = 0;
  int firstOn = -1;
  for (int i = 0; i < NUM_BEAMS; i++) {
    if (receivedData & (1 << i)) {
      if (firstOn < 0) {
        firstOn = i;
      }
    }
  }
  if (firstOn >= 0) {
    stableData |= (1 << firstOn);
  }

  for (int i = 0; i < NUM_BEAMS; i++) {
    bool beamIsInterrupted = (stableData & (1 << i)) != 0;

    if (beamIsInterrupted) {
      beamStates[i].offStartTime = 0;
      byte channel = (i < 4) ? 0 : 1;
      unsigned int dist = (i < 4) ? sensor1Distance : sensor2Distance;
      int rawOffset = getOctaveOffset(dist);
      int stableOffset = (channel == 0) 
                           ? stabilizeOffset(rawOffset, oldOffsetChannel0)
                           : stabilizeOffset(rawOffset, oldOffsetChannel1);
      int newPitch = baseNotes[i] + stableOffset;

      if (!beamStates[i].playing) {
        if (channel == 0) {
          oldOffsetChannel0 = -1;
        } else {
          oldOffsetChannel1 = -1;
        }
        beamStates[i].playing = true;
        beamStates[i].currentPitch = newPitch;
        noteOn(channel, newPitch, 100);
      } else {
        if (beamStates[i].currentPitch != newPitch) {
          noteOff(channel, beamStates[i].currentPitch, 0x40);
          beamStates[i].currentPitch = newPitch;
          noteOn(channel, newPitch, 100);
        }
      }
    }
    else {
      if (beamStates[i].playing) {
        if (beamStates[i].offStartTime == 0) {
          beamStates[i].offStartTime = now;
        } 
        else {
          if (now - beamStates[i].offStartTime >= OFF_DEBOUNCE_MS) {
            byte channel = (i < 4) ? 0 : 1;
            noteOff(channel, beamStates[i].currentPitch, 0x40);
            beamStates[i].playing = false;
            if (channel == 0) {
              oldOffsetChannel0 = -1;
            } else {
              oldOffsetChannel1 = -1;
            }
          }
        }
      }
    }
  }

  MidiUSB.flush();
}

void receiveEvent(int bytesReceived) {
  while (Wire.available()) {
    receivedData = Wire.read();
  }
}

void triggerAndReadUltrasonics() {
  ultrasonicSensor1.write(0x55);
  delay(50);

  if (ultrasonicSensor1.available() >= 2) {
    HighByte = ultrasonicSensor1.read();
    LowByte = ultrasonicSensor1.read();
    sensor1Distance = (HighByte << 8) + LowByte;
    if (sensor1Distance < 2 || sensor1Distance > 1800) {
      sensor1Distance = 0;
    }
    if (abs((int)sensor1Distance - (int)lastSensor1Filtered) > 20) {
      sensor1Distance = lastSensor1Filtered;
    } else {
      lastSensor1Filtered = sensor1Distance;
    }
  }

  Serial1.write(0x55);
  delay(50);

  if (Serial1.available() >= 2) {
    HighByte = Serial1.read();
    LowByte = Serial1.read();
    sensor2Distance = (HighByte << 8) + LowByte;
    if (sensor2Distance < 2 || sensor2Distance > 1800) {
      sensor2Distance = 0;
    }
    if (abs((int)sensor2Distance - (int)lastSensor2Filtered) > 20) {
      sensor2Distance = lastSensor2Filtered;
    } else {
      lastSensor2Filtered = sensor2Distance;
    }
  }
}

int getOctaveOffset(unsigned int distanceMM) {
  int offset = map(distanceMM, 0, 1500, 24, 0);
  return constrain(offset, 0, 24);
}

int stabilizeOffset(int newOffset, int &oldOffset) {
  if (oldOffset < 0) {
    oldOffset = newOffset;
    return newOffset;
  }
  if (abs(newOffset - oldOffset) <= 1) {
    return oldOffset;
  }
  oldOffset = newOffset;
  return newOffset;
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOnPacket = {
    0x09,
    (byte)(0x90 | channel),
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOnPacket);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOffPacket = {
    0x08,
    (byte)(0x80 | channel),
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOffPacket);
}