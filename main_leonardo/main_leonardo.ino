#include <SoftwareSerial.h>
#include "MIDIUSB.h"
#include <Wire.h>

// --------------------------------------------------------------------------
//                     GLOBALS & CONSTANTS
// --------------------------------------------------------------------------
SoftwareSerial ultrasonicSensor1(8, 9); // RX, TX for Sensor1

// This struct tracks whether a beam is playing and the current pitch
struct BeamState {
  bool playing;
  int  currentPitch;
};

// We'll track 7 beams
const int NUM_BEAMS = 7;
BeamState beamStates[NUM_BEAMS];

// Assign a base note to each beam (C, D, E, F, G, A, B) around middle C
const int baseNotes[NUM_BEAMS] = {60, 62, 64, 65, 67, 69, 71};

// Store the last distances measured from each sensor
unsigned int sensor1Distance = 0;
unsigned int sensor2Distance = 0;

// For reading raw bytes from ultrasonic
unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;

// I2C data: single byte, bits 0..6 indicate if beam i is interrupted
volatile byte receivedData = 0;

// Timers
unsigned long lastSensorCheck       = 0;
const unsigned long sensorCheckRate = 50; // check sensors every 50ms

// Offsets from the sensors to be stabilized
int oldOffsetChannel0 = -1; // used for beams 0..3
int oldOffsetChannel1 = -1; // used for beams 4..6

// --------------------------------------------------------------------------
//                     FUNCTION DECLARATIONS
// --------------------------------------------------------------------------
void triggerAndReadUltrasonics();
int getOctaveOffset(unsigned int distanceMM);
int stabilizeOffset(int newOffset, int &oldOffset);

void noteOn(byte channel, byte pitch, byte velocity);
void noteOff(byte channel, byte pitch, byte velocity);

// --------------------------------------------------------------------------
//                              SETUP
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);             // Debug prints
  ultrasonicSensor1.begin(9600);  // Ultrasonic Sensor1
  Serial1.begin(9600);            // Ultrasonic Sensor2 (e.g. on Mega pins 19 (RX), 18 (TX))

  // Initialize beam states
  for (int i = 0; i < NUM_BEAMS; i++) {
    beamStates[i].playing = false;
    beamStates[i].currentPitch = 0;
  }

  // Set up I2C as Slave at address 0x08
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);

  Serial.println("Slave ready, listening for beam data...");
}

// --------------------------------------------------------------------------
//                              LOOP
// --------------------------------------------------------------------------
void loop() {
  // 1) Periodically read the ultrasonic sensors to update distances
  unsigned long now = millis();
  if (now - lastSensorCheck >= sensorCheckRate) {
    lastSensorCheck = now;
    triggerAndReadUltrasonics();
  }

  // 2) Check which beams are on/off and assign notes
  for (int i = 0; i < NUM_BEAMS; i++) {
    bool beamIsInterrupted = (receivedData & (1 << i)) != 0;  // check bit i

    if (beamIsInterrupted) {
      // Decide channel and which sensor's distance
      byte channel      = (i < 4) ? 0 : 1;   // first 4 beams on channel 0, last 3 on channel 1
      unsigned int dist = (i < 4) ? sensor1Distance : sensor2Distance;

      // 1) Get how many semitones we offset from the base note
      int rawOffset = getOctaveOffset(dist);

      // 2) Stabilize that offset so it doesn't flicker
      int stableOffset = (channel == 0) 
                           ? stabilizeOffset(rawOffset, oldOffsetChannel0)
                           : stabilizeOffset(rawOffset, oldOffsetChannel1);

      // 3) Final pitch = base note (depends on beam) + stable offset
      int newPitch = baseNotes[i] + stableOffset;

      if (!beamStates[i].playing) {
        // If this beam was not playing, start a new note
        beamStates[i].playing = true;
        beamStates[i].currentPitch = newPitch;
        noteOn(channel, newPitch, 100);
      } else {
        // Beam is already playing, check if pitch changed
        if (beamStates[i].currentPitch != newPitch) {
          noteOff(channel, beamStates[i].currentPitch, 0x40);
          beamStates[i].currentPitch = newPitch;
          noteOn(channel, newPitch, 100);
        }
      }
    }
    else {
      // Beam bit is OFF => turn off if currently playing
      if (beamStates[i].playing) {
        byte channel = (i < 4) ? 0 : 1;
        noteOff(channel, beamStates[i].currentPitch, 0x40);
        beamStates[i].playing = false;
      }
    }
  }

  // 3) Flush MIDI
  MidiUSB.flush();
}

// --------------------------------------------------------------------------
//               I2C Receive Callback: store single byte in receivedData
// --------------------------------------------------------------------------
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
//    Convert distance to pitch offset in semitones (e.g. up to 2 octaves)
// --------------------------------------------------------------------------
int getOctaveOffset(unsigned int distanceMM) {
  // Example: map 0..1500 => 24..0 (2 octaves)
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

  // If difference is small (<= 1 semitone), keep old offset
  if (abs(newOffset - oldOffset) <= 1) {
    return oldOffset;
  }

  // Otherwise update old offset to new
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
    0x08,                   // USB MIDI event (Note Off)
    (byte)(0x80 | channel), // MIDI command (Note Off) + channel
    pitch,
    velocity
  };
  MidiUSB.sendMIDI(noteOffPacket);
}130