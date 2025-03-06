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

const int baseNotes[NUM_BEAMS] = {24, 26, 28, 29, 31, 33, 35};

const unsigned long OFF_DEBOUNCE_MS = 200; 

unsigned int sensor1Distance = 0;
unsigned int sensor2Distance = 0;

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;

volatile byte receivedData = 0;

unsigned long lastSensorCheck       = 0;
const unsigned long sensorCheckRate = 50;

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

  // 1) Periodically read the ultrasonic sensors
  if (now - lastSensorCheck >= sensorCheckRate) {
    lastSensorCheck = now;
    triggerAndReadUltrasonics();
  }

  for (int i = 0; i < NUM_BEAMS; i++) {
    bool beamIsInterrupted = (receivedData & (1 << i)) != 0;  // check bit i

    if (beamIsInterrupted) {
      Serial.print("Beam ");
      Serial.print(i);
      Serial.println(" is ON");

      beamStates[i].offStartTime = 0;

      // Decide which channel and which sensor distance
      byte channel      = i
      unsigned int dist = (i < 4) ? sensor1Distance : sensor2Distance;

      // 1) Determine the semitone offset from the base note
      int rawOffset = getOctaveOffset(dist);

      // 2) Stabilize that offset so it doesn’t flicker
      int stableOffset = (channel == 0) 
                           ? stabilizeOffset(rawOffset, oldOffsetChannel0)
                           : stabilizeOffset(rawOffset, oldOffsetChannel1);

      // 3) Final pitch = baseNotes[i] + stableOffset
      int newPitch = baseNotes[i] + stableOffset;

      if (!beamStates[i].playing) {
        // If this beam wasn’t playing, start a new note
        beamStates[i].playing      = true;
        beamStates[i].currentPitch = newPitch;
        noteOn(channel, newPitch, 100);
      } else {
        // Beam is already playing; check if pitch changed
        if (beamStates[i].currentPitch != newPitch) {
          noteOff(channel, beamStates[i].currentPitch, 0x40);
          beamStates[i].currentPitch = newPitch;
          noteOn(channel, newPitch, 100);
        }
      }
    }
    else {
      // Beam is OFF => maybe turn the note off, but use debounce
      if (beamStates[i].playing) {
        // If we haven’t started counting “off” time yet, start now
        if (beamStates[i].offStartTime == 0) {
          beamStates[i].offStartTime = now;
        } 
        else {
          // If it’s been off for longer than OFF_DEBOUNCE_MS, turn the note off
          if (now - beamStates[i].offStartTime >= OFF_DEBOUNCE_MS) {
            byte channel = (i < 4) ? 0 : 1;
            noteOff(channel, beamStates[i].currentPitch, 0x40);
            beamStates[i].playing = false;
          }
        }
      }
    }
  }

  // 3) Flush MIDI
  MidiUSB.flush();
}

void receiveEvent(int bytesReceived) {
  while (Wire.available()) {
    receivedData = Wire.read(); 
    Serial.print("Received beams: ");
    Serial.println(receivedData, BIN); // e.g. "1010101" bits
  }
}

// --------------------------------------------------------------------------
//               Ultrasonic Trigger & Read
// --------------------------------------------------------------------------
void triggerAndReadUltrasonics() {
  // Send trigger byte "0x55" to each sensor
  ultrasonicSensor1.write(0x55);
  Serial1.write(0x55);

  // --- Read sensor1 (ultrasonicSensor1) ---
  if (ultrasonicSensor1.available() >= 2) {
    HighByte          = ultrasonicSensor1.read();
    LowByte           = ultrasonicSensor1.read();
    sensor1Distance   = (HighByte << 8) + LowByte; // in millimeters

    // Optional validity check
    if (sensor1Distance < 2 || sensor1Distance > 1800) {
      sensor1Distance = 0;
    }
  }

  // --- Read sensor2 (Serial1) ---
  if (Serial1.available() >= 2) {
    HighByte          = Serial1.read();
    LowByte           = Serial1.read();
    sensor2Distance   = (HighByte << 8) + LowByte;

    if (sensor2Distance < 2 || sensor2Distance > 1800) {
      sensor2Distance = 0;
    }
  }
}

// --------------------------------------------------------------------------
//    Convert distance to pitch offset in semitones (up to 2 octaves, e.g.)
// --------------------------------------------------------------------------
int getOctaveOffset(unsigned int distanceMM) {
  // Example: map 0..1500 mm => 24..0 semitones
  int offset = map(distanceMM, 0, 1500, 24, 0);
  return constrain(offset, 0, 24);
}

// --------------------------------------------------------------------------
//  Keep pitch from jumping if the new offset is close to the old offset
// --------------------------------------------------------------------------
int stabilizeOffset(int newOffset, int &oldOffset) {
  // If first time, just adopt the new offset
  if (oldOffset < 0) {
    oldOffset = newOffset;
    return newOffset;
  }

  // If difference is small (<= 1 semitone), stay on old offset
  if (abs(newOffset - oldOffset) <= 1) {
    return oldOffset;
  }

  // Otherwise update old offset
  oldOffset = newOffset;
  return newOffset;
}

// --------------------------------------------------------------------------
//                           MIDI Helpers
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
    0x08,                    // USB MIDI event (Note Off)
    (byte)(0x80 | channel),  // MIDI command (Note Off) + channel
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOffPacket);
}