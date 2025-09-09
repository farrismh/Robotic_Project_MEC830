#include <SoftwareSerial.h>


// Define motor control pins
int in1 = 2;
int in2 = 3;
int in3 = 5;
int in4 = 4;
int ena = 12;
int enb = 11;


// Define Bluetooth module pins
SoftwareSerial bluetooth(9, 8); // RX, TX


void setup() {
  // Set up motor control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);


  // Set up Bluetooth communication
  Serial.begin(9600);
  Serial.println("Bluetooth initialized");
  bluetooth.begin(9600); // Set the baud rate of your Bluetooth module


  // Stop the motors initially
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}


void loop() {
  if (bluetooth.available() > 0) {
    char command = bluetooth.read();
    Serial.print("Received command: ");
    Serial.println(command);


    // Drive commands from the RC controller app
    switch (command) {
      case 'F':
        // Move forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 255);
        analogWrite(enb, 255);
        break;


      case 'B':
        // Move backward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, 255);
        analogWrite(enb, 255);
        break;


      case 'L':
        // Turn left
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 255);
        analogWrite(enb, 255);
        break;


      case 'R':
        // Turn right
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, 255);
        analogWrite(enb, 255);
        break;


      case 'S':
        // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        break;


      default:
        break;
    }
  }
}

