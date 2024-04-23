#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"



const int irSensorPinR = A8; // Analog pin for IR sensor
const int irSensorPinL = A9; // Analog pin for IR sensor
const float conversionFactor = 5.0 / 1023.0; // Conversion factor for analog readings to voltage
  
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

///////ultra sonic setup
// Pin configuration for first ultrasonic sensor
// Pin configuration for the forward-facing ultrasonic sensor
const int trigPin = 9;
const int echoPin = 10;
float duration, distance;

// Pin configuration for the left-facing ultrasonic sensor
const int trigPinLeft = 11;
const int echoPinLeft = 12;


float durationForward, aheadUltra;
float durationLeft, leftUltra;
/////////////// ultra sonic setup complete
void setup() {
  Serial.begin(9600);
  AFMS.begin();
    /// initializing pins for the ultrasonic sensors ////
  pinMode(trigPin, OUTPUT);//forward ultra
  pinMode(echoPin, INPUT); //forward ultra
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  /// initializing pins for the ultrasonic sensors ////

// Connect the motors to port M1 and M2

}

void loop() {

    defaultlinefollowing();


}




void defaultlinefollowing(){
    // Read the analog value from the IR sensor
  int sensorValueR = analogRead(irSensorPinR);
  int sensorValueL = analogRead(irSensorPinL);


  // Convert the analog value to voltage
  float voltageR = sensorValueR * conversionFactor;
  float voltageL = sensorValueL * conversionFactor;


  // Convert voltage to distance (you may need to adjust these constants based on your sensor)
  float distanceR = 13.0 / (voltageR - 0.2);
  float distanceL = 13.0 / (voltageL - 0.2);

//when the left one is 
  // Print the distance to the serial monitor
 // Serial.print("Distance: ");
 // Serial.println(distanceR);
  
  if(distanceR > 0){
  Serial.println("Right - Black");
  } else {
  Serial.println("Right - White");
  }
  //delay(500); // Adjust delay as needed

  if(distanceL > 0){
  Serial.println("Left - Black");
  } else {
  Serial.println("Left - White");
  }
  //delay(500); // Adjust delay as needed
  
  if(distanceR < 0 && distanceL < 0 ){
    Serial.println("Both sensors see white.");
    motor1->setSpeed(35);  // set the speed to maximum
    motor1->run(FORWARD);
    motor2->setSpeed(35);  // set the speed to maximum
    motor2->run(FORWARD);
  }
  else if (distanceR > 0)
  {
    Serial.println("Right sensor sees black.");
    motor1->setSpeed(50);  // set the speed to maximum
    motor1->run(FORWARD);
    motor2->setSpeed(42);  // stop left motor
    motor2->run(BACKWARD);
  }

  else if (distanceL > 0)
  {
    Serial.println("Left sensor sees black.");
    motor1->setSpeed(42);  // stop right motor
    motor1->run(BACKWARD);
    motor2->setSpeed(50);  // set the speed to maximum
    motor2->run(FORWARD);
  }


}





void ultrasonics()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance for ahead ultrasonic: ");
  Serial.println(distance);
  delay(100);
}
