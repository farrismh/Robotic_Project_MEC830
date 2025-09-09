#include <Arduino.h>
#include <Servo.h>

Servo myServo;  // Create a servo object

#define TRIG_PIN 7 // Trigger pin for the ultrasonic sensor
#define ECHO_PIN 10 // Echo pin for the ultrasonic sensor
#define DISTANCE_THRESHOLD 20 // Set the distance threshold for obstacle detection in centimeters

const int enaPin = 12;  // Enable pin for motor A
const int in1Pin = 2;   // Input 1 pin for motor A
const int in2Pin = 3;   // Input 2 pin for motor A

const int enbPin = 11;  // Enable pin for motor B
const int in3Pin = 5;   // Input 3 pin for motor B
const int in4Pin = 4;   // Input 4 pin for motor B

int turncounter = 0;
const int servoPin = 6;  // Connect the signal wire of the servo to PWM pin 6

void setup() {
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  myServo.attach(servoPin);  // Attach the servo to the specified pin
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 1; i++) {
    forward();
    forward();
    delay(550);
    right();
    forward();
    forward();
   delay(550);
    left();
    forward();
    forward();
    delay(550);
    
  if(i=1){while(true){stop();}}
  }
}

void right() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  digitalWrite(enbPin, HIGH);  // Full speed
  delay(3500);
  stop();
}

void left() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  digitalWrite(enbPin, HIGH);  // Full speed
  delay(4000);
  stop();
}

void forward() {
  digitalWrite(enaPin, HIGH);
  digitalWrite(enbPin, HIGH);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  delay(1000);
}

void stop() {
  digitalWrite(enaPin, HIGH); // Set motor A to full speed
  digitalWrite(enbPin, HIGH); // Set motor B to full speed
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, HIGH);
  delay(300);
}

bool isObstacleDetected() {
  delay(100);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);

  return (distance < DISTANCE_THRESHOLD);
}
