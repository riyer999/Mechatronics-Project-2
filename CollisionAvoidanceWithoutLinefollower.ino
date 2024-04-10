#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

const int trigPin = 9;       // Trig pin for the ahead ultrasonic sensor
const int echoPin = 10;      // Echo pin for the ahead ultrasonic sensor
const int leftTrigPin = 6;   // Trig pin for the left ultrasonic sensor
const int leftEchoPin = 7;   // Echo pin for the left ultrasonic sensor

float duration, leftDuration;
float distance, leftDistance;


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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  ultrasonics();
  delay(100);
  if (distance < 30) {
    turnRight();
    
    moveForward(50);
    moveForward(50);
  } else {
    moveForward(30);
    
    if (leftDistance > 20 && leftDistance < 45)
    turnLeft();
    
    moveForward(50);
    moveForward(50);
    //add line follower here
  }
  if (leftDistance < 10){

    moveForward(50);
   
    turnRight(); 
    
  } 
 
}

void ultrasonics() {
  // Measure distance for ahead ultrasonic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;
  Serial.print("Distance for ahead ultrasonic: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Measure distance for left ultrasonic
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
