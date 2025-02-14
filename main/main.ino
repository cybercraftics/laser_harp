#include <Stepper.h>
#include <SoftwareSerial.h>
#include <avr/interrupt.h>

// --- Pins ---
#define LASER_PIN 4
#define LIGHT_DETECTOR_PIN_1 A0
#define LIGHT_DETECTOR_PIN_2 A1
#define LIGHT_DETECTOR_PIN_3 A2
#define LIGHT_DETECTOR_PIN_4 A3
#define LIGHT_DETECTOR_PIN_5 A4
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5

// ADD THIS:
#define BUZZER_PIN 6

// --- US-100 stuff ---
#define US100_TRIGGER_INTERVAL 100 // Measurement interval in ms

const float STEPS_PER_REVOLUTION = 200;
const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;
volatile unsigned int distance_cm = 0;

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

// Choose pins that are NOT 0 and 1 for SoftwareSerial on Arduino Uno
SoftwareSerial us100(2, 3); // RX, TX

void setup() {
    // Serial.begin(115200);
    us100.begin(9600);
    
    stepper.setSpeed(220);
    
    pinMode(LASER_PIN, OUTPUT);
    pinMode(LIGHT_DETECTOR_PIN_1, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_2, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_3, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_4, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_5, INPUT);
    pinMode(CLOSED_LOOP_MOTOR_SENSOR_PIN, INPUT);

    // Buzzer pin
    pinMode(BUZZER_PIN, OUTPUT);

    // --- Timer setup for ~100 ms (10 Hz) ---
    cli();                // Disable global interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    // For 20ms = 312 counts
    OCR1A = 312; // ~20ms at 16MHz/1024
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
    sei();                // Enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
    requestDistance();
}

void requestDistance() {
    us100.write(0x55);

    // Read sensor output if available
    if (us100.available() >= 2) {
        byte highByte = us100.read();
        byte lowByte  = us100.read();
        distance_cm = (highByte << 8) | lowByte;
    }
}

// ---------------------------------------------------
// Buzzer function: The closer your hand, the higher the pitch
// ---------------------------------------------------
void playBuzzerByDistance(unsigned int dist) {
    // For example, if the distance is between 2cm and 80cm:
    // We'll map that range to a pitch between 2000 Hz (close) and 200 Hz (far)
    // Adjust these bounds to suit your preference!
    
    const int minDistance = 2;   // cm
    const int maxDistance = 1500;  // cm
    const int maxPitch = 2000;   // Hz
    const int minPitch = 10;    // Hz

    if (dist >= minDistance && dist <= 1500) {
        // map distance to pitch
        int pitch = map(dist, minDistance, maxDistance, maxPitch, minPitch);
        
        // Constrain pitch so we don't go negative or above our range
        pitch = constrain(pitch, minPitch, maxPitch);

        // Generate the tone on BUZZER_PIN
        tone(BUZZER_PIN, pitch);
    } 
    else {
        // If distance is out of a sensible range (too far or zero), turn buzzer off
        noTone(BUZZER_PIN);
    }
}

void loop() {
    // Your existing stepping/laser code
    for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
        stepper.step(2);
        if (i == 3) {
            delayMicroseconds(1000);
        }
        if (i == 4 || i == 5 || i == 6) {
            delayMicroseconds(800);
        }
        analogWrite(LASER_PIN, 255);
        delayMicroseconds(LASER_ON_DURATION);
        analogWrite(LASER_PIN, 0);
    }
    stepper.step(-NUMBER_OF_STRINGS * 2);

    // Play the buzzer based on distance
    playBuzzerByDistance(distance_cm);
}