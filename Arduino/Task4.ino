const int enaPin = 12;  // Enable pin for motor A
const int in1Pin = 2;  // Input 1 pin for motor A
const int in2Pin = 3;  // Input 2 pin for motor A

const int enbPin = 11;  // Enable pin for motor B
const int in3Pin = 5;  // Input 3 pin for motor B
const int in4Pin = 4;  // Input 4 pin for motor B
void setup() {
 // Set motor control pins as outputs
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
for (int i=0; i<3;i++){  // put your main code here, to run repeatedly:
forward();
right ();
forward();
right();
forward();
right();
forward();
stop();
if (i=2){forward();}
Serial.print(i);
while(i==2){
  stop();
}
}
}
void right (){
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  digitalWrite(enbPin, HIGH);  // Full speed
  delay (2000);
}

void forward () {
   digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(enbPin, HIGH);  // Full speed
 digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  delay(1000);
}

void stop(void){
  digitalWrite(enaPin, LOW);
  digitalWrite(enbPin, LOW);
  
}
void left (){
   digitalWrite(enaPin, HIGH);  // Full speed
  digitalWrite(enbPin, HIGH);  // Full speed
 digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  delay(2000);
}