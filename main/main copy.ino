#include <Stepper.h>

// Pin definitions
#define LASER_PIN 4
#define LIGHT_DETECTOR_PIN_1 A0
#define LIGHT_DETECTOR_PIN_2 A1
#define LIGHT_DETECTOR_PIN_3 A2
#define LIGHT_DETECTOR_PIN_4 A3
#define LIGHT_DETECTOR_PIN_5 A4
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5

// Ultrasonic pins
#define ULTRASONIC_SENSOR_1_TRIGGER_PIN 7
#define ULTRASONIC_SENSOR_1_ECHO_PIN 2
#define ULTRASONIC_SENSOR_2_TRIGGER_PIN 3
#define ULTRASONIC_SENSOR_2_ECHO_PIN 5

// Buzzer pin
#define BUZZER_PIN 6

// Stepper config
const float STEPS_PER_REVOLUTION = 200;
const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

// Use a smaller median window for faster response
const int NUM_SAMPLES = 3;

// Higher smoothing factor -> more weight on new readings (faster response)
static float filteredDistance1 = -1;
static float filteredDistance2 = -1;
const float SMOOTHING_FACTOR = 0.4f;  // 0.4 is more responsive than 0.2

void setup() {
    Serial.begin(115200);
    stepper.setSpeed(220);

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

    Serial.println("Ultrasonic + Stepper + Buzzer (More Responsive) started...");
}

void loop() {
    // Move stepper and blink laser (as in your original code)
    // for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    //     stepper.step(2);
        
    //     if (i == 3) {
    //         delayMicroseconds(1000);
    //     }
    //     if (i == 4 || i == 5 || i == 6) {
    //         delayMicroseconds(800);
    //     }

    //     analogWrite(LASER_PIN, 255);
    //     delayMicroseconds(LASER_ON_DURATION);
    //     analogWrite(LASER_PIN, 0);
    // }
    // stepper.step(-NUMBER_OF_STRINGS * 2);

    // 1) Get a median-filtered reading (just 3 samples now)
    float medianDistance1 = getMedianDistance(ULTRASONIC_SENSOR_1_TRIGGER_PIN, ULTRASONIC_SENSOR_1_ECHO_PIN);
    float medianDistance2 = getMedianDistance(ULTRASONIC_SENSOR_2_TRIGGER_PIN, ULTRASONIC_SENSOR_2_ECHO_PIN);

    // 2) Exponential smoothing with a higher SMOOTHING_FACTOR
    filteredDistance1 = updateFilteredDistance(filteredDistance1, medianDistance1);
    filteredDistance2 = updateFilteredDistance(filteredDistance2, medianDistance2);

    // Play separate sounds for each sensor
    // if (filteredDistance1 > 0) {
    //     playHighPitch(filteredDistance1);
    // } else {
    //     noTone(BUZZER_PIN);
    // }
    if (filteredDistance2 > 0) {
        playLowPitch(filteredDistance2);
    } else {
        noTone(BUZZER_PIN);
    }

    // Reduce the overall loop delay for faster updates
    delay(20);
}

// (1) MEDIAN FILTERING of 3 samples -> faster sampling and less overkill smoothing
float getMedianDistance(int triggerPin, int echoPin) {
    float readings[NUM_SAMPLES];

    for (int i = 0; i < NUM_SAMPLES; i++) {
        readings[i] = getRawDistance(triggerPin, echoPin);
        // Short delay between samples
        delay(2);
    }

    // Sort the array (simple bubble sort for demonstration)
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
        for (int j = 0; j < NUM_SAMPLES - i - 1; j++) {
            if (readings[j] > readings[j + 1]) {
                float temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    // Middle element is the median
    float median = readings[NUM_SAMPLES / 2];
    if (median < 0) {
        return -1;
    }
    return median;
}

float getRawDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(5);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(triggerPin, LOW);

    // 45ms for ~4.5m
    long duration = pulseIn(echoPin, HIGH, 40000);

    float distance = duration * 0.0343f / 2.0f; 
    if (distance > 2 && distance <= 150) {
        return distance;
    }
    return -1;
}

// (2) Exponential smoothing
float updateFilteredDistance(float currentFiltered, float newReading) {
    if (currentFiltered < 0 && newReading > 0) {
        return newReading;  // first valid reading
    } else if (newReading > 0) {
        return (currentFiltered * (1.0f - SMOOTHING_FACTOR)) + (newReading * SMOOTHING_FACTOR);
    } else {
        return -1;
    }
}

// Adjusted pitch mapping for "high" sensor
void playHighPitch(float distance) {
    // range: 2 -> 450 cm mapped to 400 -> 1500 Hz
    int freq = map(distance, 2, 450, 400, 1500);
    tone(BUZZER_PIN, freq);
}

// Adjusted pitch mapping for "low" sensor
void playLowPitch(float distance) {
    // range: 2 -> 450 cm mapped to 50 -> 500 Hz
    int freq = map(distance, 2, 450, 50, 500);
    tone(BUZZER_PIN, freq);
}