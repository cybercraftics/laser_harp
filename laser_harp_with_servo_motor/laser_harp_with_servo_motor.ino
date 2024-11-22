const int dirPin = 2;  // Direction pin
const int stepPin = 3; // Step pin

// Define motor and harp properties
const int TOTAL_STEPS = 15;         // Total travel distance in steps
const int NUM_STRINGS = 6;          // Number of laser harp strings
const int STEPS_PER_STRING = TOTAL_STEPS / NUM_STRINGS; // Steps between strings
const int STEP_DELAY = 500;         // Minimum reliable delay between steps in microseconds
const int PAUSE_DELAY = 4;         // Pause at each string in milliseconds

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Set initial direction
  digitalWrite(dirPin, HIGH);
}

void loop() {
  // Travel through all string positions
  for (int i = 0; i < NUM_STRINGS; i++) {
    // Move the motor to the next string
    moveSteps(STEPS_PER_STRING);
    // Briefly pause to "draw" the string
    delay(PAUSE_DELAY);
  }

  // Reverse direction for the return path
  digitalWrite(dirPin, !digitalRead(dirPin));
}

// Function to move a specific number of steps at maximum speed
void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY); // Short delay for maximum speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}