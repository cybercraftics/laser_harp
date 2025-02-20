#include <Stepper.h>

// Pin definitions
#define LASER_PIN                 7
#define LIGHT_DETECTOR_PIN_1      A0
#define LIGHT_DETECTOR_PIN_2      A1
#define LIGHT_DETECTOR_PIN_3      A2
#define LIGHT_DETECTOR_PIN_4      A3
#define LIGHT_DETECTOR_PIN_5      A4
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5

// Ultrasonic pins
#define ULTRASONIC_SENSOR_1_TRIGGER_PIN  2
#define ULTRASONIC_SENSOR_1_ECHO_PIN     3
#define ULTRASONIC_SENSOR_2_TRIGGER_PIN  4
#define ULTRASONIC_SENSOR_2_ECHO_PIN     5

// Buzzer pin
#define BUZZER_PIN               6

// Stepper config
const float STEPS_PER_REVOLUTION = 200;
const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;

// Create the Stepper instance: 
//   For the Leonardo, pins 8,9,10,11 are typically free for stepper use.
Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

// Exponential smoothing (optional—currently not used in single-sample approach)
static float filteredDistance1 = -1;
static float filteredDistance2 = -1;
const float SMOOTHING_FACTOR = 0.4f;

// Global flag set by Timer1 ISR 
volatile bool gUltrasonicMeasureFlag = false;

// We'll store the latest distances in these (optional)
float gLatestDistance1 = -1;
float gLatestDistance2 = -1;

void setup() {
    // Stepper speed
    stepper.setSpeed(220);

    // Configure pins
    pinMode(LASER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(LIGHT_DETECTOR_PIN_1, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_2, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_3, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_4, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_5, INPUT);
    pinMode(CLOSED_LOOP_MOTOR_SENSOR_PIN, INPUT);

    pinMode(ULTRASONIC_SENSOR_1_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_SENSOR_1_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_SENSOR_2_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_SENSOR_2_ECHO_PIN, INPUT);

    // Initialize the timer so we measure ultrasonic sensors every 20ms
    initTimer();
}

void loop() {
    // ---------------------------------------------------------------------
    // Example "blocking" stepper/laser code:
    //  - This loop steps the motor 7 times forward, firing the laser each time,
    //    then returns back. This can block for a while.
    //  - If you need faster sensor reaction, see "Optional: Break Up Stepper" below.
    // ---------------------------------------------------------------------
    for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
        stepper.step(2);

        // Fire laser
        analogWrite(LASER_PIN, 255);
        delayMicroseconds(LASER_ON_DURATION);
        analogWrite(LASER_PIN, 0);
    }
    stepper.step(-NUMBER_OF_STRINGS * 2);

    // ---------------------------------------------------------------------
    // Check if the Timer1 interrupt says "time to measure ultrasound"
    // ---------------------------------------------------------------------
    if (gUltrasonicMeasureFlag) {
        gUltrasonicMeasureFlag = false; // Reset the flag

        // Get single-sample distances for both sensors
        float dist1 = getDistance(ULTRASONIC_SENSOR_1_TRIGGER_PIN, ULTRASONIC_SENSOR_1_ECHO_PIN);
        float dist2 = getDistance(ULTRASONIC_SENSOR_2_TRIGGER_PIN, ULTRASONIC_SENSOR_2_ECHO_PIN);

        // Store globally if desired
        gLatestDistance1 = dist1;
        gLatestDistance2 = dist2;

        // Example if you wanted to do exponential smoothing:
        /*
        filteredDistance1 = updateFilteredDistance(filteredDistance1, dist1);
        filteredDistance2 = updateFilteredDistance(filteredDistance2, dist2);
        */

        // Decide which distance(s) get used for the buzzer
        bool firstValid = (dist1 > 0);
        bool secondValid = (dist2 > 0);

        if (firstValid && secondValid) {
            unsigned int combinedDist = (unsigned int)((dist1 + dist2) / 2.0f);
            playBuzzerByDistance(combinedDist);
        } else if (firstValid) {
            playBuzzerByDistance((unsigned int)dist1);
        } else if (secondValid) {
            playBuzzerByDistance((unsigned int)dist2);
        } else {
            noTone(BUZZER_PIN);
        }
    }

    // ...Your other non-blocking code could go here...
}

// -------------------------------------------------------------------------
// TIMER SETUP for ~20ms using Timer1 (ATmega32U4 - e.g. Arduino Leonardo)
// -------------------------------------------------------------------------
void initTimer() {
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // CPU clock is 16MHz.
    // Prescaler = 64 => each timer tick = 4 microseconds.
    // For 20ms => 20,000us / 4us = 5,000 ticks
    OCR1A = 5000;             // Compare match register
    TCCR1B |= (1 << WGM12);   // CTC mode (WGM12=1)
    TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler=64
    TIMSK1 |= (1 << OCIE1A);  // Enable compare match interrupt

    interrupts();
}

// -------------------------------------------------------------------------
// TIMER1 COMPARE MATCH INTERRUPT ~ 20ms
// -------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
    gUltrasonicMeasureFlag = true; // Let main loop know to measure
}

// -------------------------------------------------------------------------
// Single-Sample Ultrasonic Reading (No median)
// Lower pulseIn timeout to 15ms
// -------------------------------------------------------------------------
float getDistance(int triggerPin, int echoPin) {
    // Trigger a pulse
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(15);
    digitalWrite(triggerPin, LOW);

    // Wait up to 15,000 microseconds (~15ms) for echo
    long duration = pulseIn(echoPin, HIGH, 15000);

    // Convert to distance in cm
    float distance = duration * 0.0343f / 2.0f;
    if (distance > 2 && distance <= 150) {
        return distance;
    }
    return -1;
}

// -------------------------------------------------------------------------
// OPTIONAL: Exponential Smoothing
// -------------------------------------------------------------------------
float updateFilteredDistance(float currentFiltered, float newReading) {
    if (currentFiltered < 0 && newReading > 0) {
        // If we had no valid reading, jump to the new reading
        return newReading;
    } else if (newReading > 0) {
        // Standard exponential smoothing
        return (currentFiltered * (1.0f - SMOOTHING_FACTOR)) + (newReading * SMOOTHING_FACTOR);
    } else {
        // If the new reading is invalid, treat as -1
        return -1;
    }
}

// -------------------------------------------------------------------------
// Map distance -> pitch and play it
// Adjust as desired
// -------------------------------------------------------------------------
void playBuzzerByDistance(unsigned int dist) {
    const int minDistance = 10;   // cm
    const int maxDistance = 150;  // cm
    const int maxPitch    = 2000; // Hz
    const int minPitch    = 200;  // Hz

    if (dist >= minDistance && dist <= maxDistance) {
        int pitch = map(dist, minDistance, maxDistance, maxPitch, minPitch);
        pitch = constrain(pitch, minPitch, maxPitch);
        tone(BUZZER_PIN, pitch);
    } else {
        noTone(BUZZER_PIN);
    }
}