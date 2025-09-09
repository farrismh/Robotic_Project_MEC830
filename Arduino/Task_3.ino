#include <Arduino.h>
#include <Servo.h>

Servo myServo;  // Create a servo object

#define TRIG_PIN 7 // Trigger pin for the ultrasonic sensor
#define ECHO_PIN 10 // Echo pin for the ultrasonic sensor
#define DISTANCE_THRESHOLD 20 // Set the distance threshold for obstacle detection in centimeters
#define TARGET_DISTANCE 276 // The target distance from point A to B in centimeters
#define MAX_DIST_TIME 8380 // Maximum time to reach the target distance in milliseconds (8.38 seconds)

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

  unsigned long startTime = millis();
  unsigned long elapsedTime;

  // Move forward until the target distance is reached or the time limit is reached
  while (true) {
    elapsedTime = millis() - startTime;

    if (elapsedTime >= MAX_DIST_TIME) {
      // Time limit reached, stop
      stop();
      delay(50000);
      break;
    }

    forward();
    delay(100); // Adjust the delay based on the behavior of your robot

    if (isObstacleDetected()) {
      // Obstacle detected, stop and take a fixed diversion
      stop();
      delay(500); // Pause for 0.5 seconds

      // Divert around the obstacle
      divertObstacle();

      // Return to the original path
      returnToPath();

      // Update start time to include the time spent on diversion
      startTime = millis();
    }
  }
}
 
void divertObstacle() {
  // For simplicity, we'll turn left for 90 degrees (adjust as needed)
  left(); // Pause for 0.5 seconds to allow the robot to complete the turn
stop();
  // Move forward to clear the obstacle
  forward();// Adjust the delay based on the behavior of your robot
delay(500);
  // Turn back in the opposite direction to return to the original path
  right();
  stop();
  // Move forward to clear the obstacle
  forward();
  delay(500); // Adjust the delay based on the behavior of your robot

   // Turn back in the opposite direction to return to the original path
  right(); // Pause for 0.5 seconds to allow the robot to complete the turn
stop();
  // Move forward to clear the obstacle
  forward();
  delay(500); // Adjust the delay based on the behavior of your robot
  stop(); // Stop the robot
}
void returnToPath() {
  // For simplicity, we'll turn right for 90 degrees (adjust as needed)
  left(); // Pause for 0.5 seconds to allow the robot to complete the turn
}

  
void right() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  digitalWrite(enbPin, HIGH);  // Full speed
  delay(2000);
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

void left() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  digitalWrite(enbPin, HIGH);  // Full speed
  delay(2000);
  stop();
}

void shome() {
  myServo.write(90);
}

void sright() {
  myServo.write(0);
}

void sleft() {
  myServo.write(180);
}

bool isObstacleDetected() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;

  return (distance < DISTANCE_THRESHOLD);
}
