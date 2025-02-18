#include <SoftwareSerial.h>
#include <Stepper.h>

// --- Pin Definitions ---
#define LIGHT_DETECTOR_PIN_1         A0
#define LIGHT_DETECTOR_PIN_2         A1
#define LIGHT_DETECTOR_PIN_3         A2
#define LIGHT_DETECTOR_PIN_4         A3
#define LIGHT_DETECTOR_PIN_5         A4
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5

#define GREEN_LASER_PIN  2
#define RED_LASER_PIN    3
#define BLUE_LASER_PIN   4
#define BUZZER_PIN       5

// --- Sound Sensor Serial ---
SoftwareSerial mySerial(8, 9);

// --- Stepper Motor Setup ---
const float STEPS_PER_REVOLUTION = 200;
Stepper stepper(STEPS_PER_REVOLUTION, 10, 12, 11, 13);

const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;

// --- Distance Measurement Globals ---
unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len      = 0;

// --- Timer for Sound Sensors ---
unsigned long lastSoundSensorCheck = 0;
const unsigned long soundSensorInterval = 100; // check distance every 100ms

// -----------------------------------------------------
// 1. Homing Routine
// -----------------------------------------------------
void homingRoutine() {
  Serial.println("Starting homing routine...");

  // Step backwards until CLOSED_LOOP_MOTOR_SENSOR_PIN reads 0
  while (digitalRead(CLOSED_LOOP_MOTOR_SENSOR_PIN) != 0) {
    // Step the motor one step at a time (adjust direction if reversed)
    stepper.step(-1);
    // Tiny delay to avoid going too fast
    delay(50);
  }

  stepper.step(-9);
}

// -----------------------------------------------------
// 2. Setup
// -----------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  mySerial.begin(9600);

  pinMode(GREEN_LASER_PIN, OUTPUT);
  pinMode(RED_LASER_PIN, OUTPUT);
  pinMode(BLUE_LASER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LIGHT_DETECTOR_PIN_1, INPUT);
  pinMode(LIGHT_DETECTOR_PIN_2, INPUT);
  pinMode(LIGHT_DETECTOR_PIN_3, INPUT);
  pinMode(LIGHT_DETECTOR_PIN_4, INPUT);
  pinMode(LIGHT_DETECTOR_PIN_5, INPUT);
  pinMode(CLOSED_LOOP_MOTOR_SENSOR_PIN, INPUT);

  // Stepper speed
  stepper.setSpeed(220);

  // Call homing once before starting the main loop
  homingRoutine();
}

// -----------------------------------------------------
// 3. Main Loop
// -----------------------------------------------------
void loop() {
  // (A) YOUR MOTOR AND LASER CODE
  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    stepper.step(2);

    if (i == 3) {
      delayMicroseconds(1000);
    }
    if (i == 4 || i == 5 || i == 6) {
      delayMicroseconds(800);
    }

    // Fire green laser briefly
    analogWrite(GREEN_LASER_PIN, 255);
    delayMicroseconds(LASER_ON_DURATION);
    analogWrite(GREEN_LASER_PIN, 0);
  }

  // Move stepper back
  stepper.step(-NUMBER_OF_STRINGS * 2);

  // Read closed loop data (optional)
  int closedLoopData = digitalRead(CLOSED_LOOP_MOTOR_SENSOR_PIN);
  // Serial.println(closedLoopData);

  // (B) SOUND SENSORS ON A TIMER
  unsigned long currentTime = millis();
  if (currentTime - lastSoundSensorCheck >= soundSensorInterval) {
    lastSoundSensorCheck = currentTime;
    updateSoundSensors();
  }

  // ... other logic can go here
}

// -----------------------------------------------------
// 4. Sound Sensor Update Function
// -----------------------------------------------------
void updateSoundSensors() {
  // *** Sensor 1 (SoftwareSerial) ***
  mySerial.write(0x55); // Trigger measurement

  // *** Sensor 2 (Serial1) ***
  Serial1.write(0x55); // Trigger measurement

  if (mySerial.available() >= 2) {
    HighByte = mySerial.read();
    LowByte  = mySerial.read();
    Len  = HighByte * 256 + LowByte;
    if (Len > 1 && Len <= 1500) {
      // For distances up to 1.50m, play a tone
      playDistanceTone(Len, "Sensor1");
    } 
    else {
      // Too far or invalid reading: no tone
    }
  }

  if (Serial1.available() >= 2) {
    HighByte = Serial1.read();
    LowByte  = Serial1.read();
    Len  = HighByte * 256 + LowByte;
    if (Len > 1 && Len <= 1500) {
      // For distances up to 1.50m, play a tone
      playDistanceTone(Len, "Sensor2");
    } 
    else {
      // Too far or invalid reading: no tone
    }
  }
}

// -----------------------------------------------------
// 5. Play a Tone Based on Distance
// -----------------------------------------------------
void playDistanceTone(unsigned int distanceMM, const char* sensorName) {
  // Example mapping: 0 mm -> 2000 Hz, 1500 mm -> 200 Hz
  // Adjust frequency range to taste
  unsigned int frequency = map(distanceMM, 0, 1500, 2000, 200);
  
  // Debug if you want
  // Serial.print(sensorName);
  // Serial.print(" Distance: ");
  // Serial.print(distanceMM);
  // Serial.print(" mm => Freq: ");
  // Serial.println(frequency);

  // Play tone for 50 ms on the buzzer
  tone(BUZZER_PIN, frequency, 50);
  
  // tone() with duration is non-blocking, 
  // so it will stop after 50 ms automatically.
}