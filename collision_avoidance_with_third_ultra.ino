#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

const int aheadTrigPin = 9;       // Trig pin for the ahead ultrasonic sensor
const int aheadEchoPin = 10;      // Echo pin for the ahead ultrasonic sensor
const int leftTrigPin = 6;   // Trig pin for the left ultrasonic sensor
const int leftEchoPin = 7;   // Echo pin for the left ultrasonic sensor
const int topleftTrigPin = 11; //trig pin for the top left ultrasonic sensor
const int topleftEchoPin = 12; //echo pin for the top left ultrasonic sensor

float aheadDuration, leftDuration, topLeftDuration;
float aheadDistance, leftDistance, topleftDistance;


void forward() {
  motor2->setSpeed(50);     // Set speed for left motor
  motor2->run(FORWARD);     // Run left motor forward
  
  motor1->setSpeed(50);     // Set speed for right motor
  motor1->run(FORWARD);     // Run right motor forward
}

void turnRight() { 
  motor2->setSpeed(70);     // Set speed for left motor
  motor2->run(BACKWARD);    // Run left motor backward
  
  motor1->setSpeed(30);     // Set speed for right motor
  motor1->run(FORWARD);     // Run right motor forward
  delay(800);
  motor2->run(RELEASE);
  motor1->run(RELEASE);
}

void turnLeft() { 
  motor2->setSpeed(30);     // Set speed for left motor
  motor2->run(FORWARD);    // Run left motor backward
  
  motor1->setSpeed(70);     // Set speed for right motor
  motor1->run(BACKWARD);     // Run right motor forward
  delay(800);
  motor2->run(RELEASE);
  motor1->run(RELEASE);
}

void moveForward(int speed) {
  motor2->setSpeed(speed);  // Set speed for left motor
  motor2->run(FORWARD);     // Run left motor forward
  
  motor1->setSpeed(speed);  // Set speed for right motor
  motor1->run(FORWARD);     // Run right motor forward
}

void setup() {
  AFMS.begin();
  pinMode(aheadTrigPin, OUTPUT);
  pinMode(aheadEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  ultrasonics();
  delay(100);
  if (aheadDistance < 30) {
    turnRight();
    
    moveForward(50);
    moveForward(50);
  } else {
    moveForward(30);
    
    if ((leftDistance > 20 && leftDistance < 45) || (topleftDistance > 23 && topleftDistance < 48))
    turnLeft();
    
    moveForward(50);
    moveForward(50);
    //add line follower here
  }
  if (leftDistance < 10 || topleftDistance < 13){

    moveForward(50);
   
    turnRight(); 
    
  } 
 
}

void ultrasonics() {
  // Measure aheadDistance for ahead ultrasonic
  digitalWrite(aheadTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(aheadTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(aheadTrigPin, LOW);

  aheadDuration = pulseIn(aheadEchoPin, HIGH);
  aheadDistance = (aheadDuration * 0.0343) / 2;
  Serial.print("Distance for ahead ultrasonic: ");
  Serial.print(aheadDistance);
  Serial.println(" cm");

    // Measure aheadDistance for topLeft ultrasonic// topLeftDuration // topleftDistance
  digitalWrite(topleftTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(topleftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(topleftTrigPin, LOW);

  topLeftDuration = pulseIn(topleftEchoPin, HIGH);
  topleftDistance = (aheadDuration * 0.0343) / 2;
  Serial.print("Distance for top left ultrasonic: ");
  Serial.print(topleftDistance);
  Serial.println(" cm");

  // Measure aheadDistance for left ultrasonic
  digitalWrite(leftTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrigPin, LOW);

  leftDuration = pulseIn(leftEchoPin, HIGH);
  leftDistance = (leftDuration * 0.0343) / 2;
  Serial.print("Distance for left ultrasonic: ");
  Serial.print(leftDistance);
  Serial.println(" cm");

  delay(100);
}
