#include <Stepper.h>

#define LASER_PIN 4
#define LIGHT_DETECTOR_PIN_1 A0
#define LIGHT_DETECTOR_PIN_2 A1
#define LIGHT_DETECTOR_PIN_3 A2
#define LIGHT_DETECTOR_PIN_4 A3
#define LIGHT_DETECTOR_PIN_5 A4
// #define CLOSED_LOOP_MOTOR_SENSOR_PIN 19 //21 Leonardo
#define CLOSED_LOOP_MOTOR_SENSOR_PIN A5


const float STEPS_PER_REVOLUTION = 200;
const unsigned int LASER_ON_DURATION = 150;
const unsigned int NUMBER_OF_STRINGS = 7;

Stepper stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);

void setup() {
    Serial.begin(115200);
    stepper.setSpeed(220);

    pinMode(LASER_PIN, OUTPUT);

    pinMode(LIGHT_DETECTOR_PIN_1, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_2, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_3, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_4, INPUT);
    pinMode(LIGHT_DETECTOR_PIN_5, INPUT);
    pinMode(CLOSED_LOOP_MOTOR_SENSOR_PIN, INPUT);
}

void loop() {
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

    // Serial.print(analogRead(LIGHT_DETECTOR_PIN_1));
    // Serial.print(", ");
    // Serial.print(analogRead(LIGHT_DETECTOR_PIN_2));
    // Serial.print(", ");
    // Serial.print(analogRead(LIGHT_DETECTOR_PIN_3));
    // Serial.print(", ");
    // Serial.print(analogRead(LIGHT_DETECTOR_PIN_4));
    // Serial.print(", ");
    // Serial.println(analogRead(LIGHT_DETECTOR_PIN_5));
    // Serial.println(analogRead(CLOSED_LOOP_MOTOR_SENSOR_PIN));
}