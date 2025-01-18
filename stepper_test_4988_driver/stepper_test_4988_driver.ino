#define DIRECTION_PIN 2
#define STEP_PIN 3

// Define motor properties
const int TRAVEL_DISTANCE = 50; // Total travel distance in steps


void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}

void loop() {
    digitalWrite(DIRECTION_PIN, HIGH);

    for (int i = 0; i < TRAVEL_DISTANCE; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(600);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(600);
    }

    delay(10);

    digitalWrite(DIRECTION_PIN, LOW);

    for (int i = 0; i < TRAVEL_DISTANCE; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(600);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(600);
    }

    delay(10);
}

