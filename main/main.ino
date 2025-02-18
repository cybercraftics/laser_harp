#include <SoftwareSerial.h>
#include <Stepper.h>

#define LIGHT_DETECTOR_PIN_1             A0
#define LIGHT_DETECTOR_PIN_2             A1
#define LIGHT_DETECTOR_PIN_3             A2
#define LIGHT_DETECTOR_PIN_4             A3
#define LIGHT_DETECTOR_PIN_5             A4
#define CLOSED_LOOP_MOTOR_SENSOR_PIN     A5

#define GREEN_LASER_PIN                   2
#define RED_LASER_PIN                     3
#define BLUE_LASER_PIN                    4
#define BUZZER_PIN                        5

SoftwareSerial mySerial(8, 9);

const float STEPS_PER_REVOLUTION        = 200;
const unsigned int LASER_ON_DURATION    = 150;
const unsigned int NUMBER_OF_STRINGS    = 7;
Stepper stepper(STEPS_PER_REVOLUTION, 10, 12, 11, 13);

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len  = 0;

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

  stepper.setSpeed(220);
}

void loop() {
  mySerial.flush();
  mySerial.write(0X55);                           // trig US-100 begin to measure the distance

  Serial1.flush();
  Serial1.write(0X55);                           // trig US-100 begin to measure the distance

  for (int i = 0; i < NUMBER_OF_STRINGS; i++) {
    stepper.step(2);

    if (i == 3) {
        delayMicroseconds(1000);
    }
    if (i == 4 || i == 5 || i == 6) {
        delayMicroseconds(800);
    }

    analogWrite(GREEN_LASER_PIN, 255);
    delayMicroseconds(LASER_ON_DURATION);
    analogWrite(GREEN_LASER_PIN, 0);
  }
  stepper.step(-NUMBER_OF_STRINGS * 2);

  // Read closed loop data
  int closedLoopData = digitalRead(CLOSED_LOOP_MOTOR_SENSOR_PIN);
  // Serial.print("Closed loop data: ");
  // Serial.println(closedLoopData);

  if (mySerial.available() >= 2)                  
  {
    HighByte = mySerial.read();
    LowByte  = mySerial.read();
    Len  = HighByte * 256 + LowByte;          
    if ((Len > 1) && (Len < 10000))
    {
      // Serial.print(">>>>>>Distance1: ");
      // Serial.print(Len, DEC);          
      // Serial.println("mm");                  
    }
  }
  
  if (Serial1.available() >= 2)                  
  {
    HighByte = Serial1.read();
    LowByte  = Serial1.read();
    Len  = HighByte * 256 + LowByte;          
    if ((Len > 1) && (Len < 10000))
    {
      // Serial.print("Distance2: ");
      // Serial.print(Len, DEC);          
      // Serial.println("mm");                  
    }
  }
}
