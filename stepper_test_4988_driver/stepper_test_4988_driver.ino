#define DIRECTION_PIN 2
#define STEP_PIN 3

// Define motor properties
const int STEPS_PER_REVOLUTION = 200;


void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}

void loop() {
    digitalWrite(DIRECTION_PIN, HIGH);

    for (int i = 0; i < STEPS_PER_REVOLUTION; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(2000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(2000);
    }

    delay(1000);

    digitalWrite(DIRECTION_PIN, LOW);

    for (int i = 0; i < STEPS_PER_REVOLUTION; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(2000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(2000);
    }

    delay(1000);
}

