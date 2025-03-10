#define DIRECTION_PIN 2
#define STEP_PIN 3
#define LASER_PIN 4

const int TRAVEL_DISTANCE = 15;
const int TRAVEL_SPEED = 600;
const int LASER_ON_DURATION = 80;


void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  analogWrite(LASER_PIN, 255);
}

void loop() {
    digitalWrite(DIRECTION_PIN, HIGH);

    for (int i = 0; i < TRAVEL_DISTANCE; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(TRAVEL_SPEED);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(TRAVEL_SPEED);
        
        if (i % 2 == 0 && i != 0) {
          analogWrite(LASER_PIN, 255);
          delayMicroseconds(LASER_ON_DURATION);
          analogWrite(LASER_PIN, 0);
        }
    }

    digitalWrite(DIRECTION_PIN, LOW);

    for (int i = 0; i < TRAVEL_DISTANCE; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(TRAVEL_SPEED);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(TRAVEL_SPEED);
    }
}

