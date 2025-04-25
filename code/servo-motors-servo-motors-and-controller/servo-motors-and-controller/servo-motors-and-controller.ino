#include <ESP32Servo.h>

// Motor Driver Pins
#define ENA 12
#define IN1 27
#define IN2 14
#define IN3 25
#define IN4 26
#define ENB 33
#define Front_Motor_EN 13
#define Front_Motor_ENA 18
#define Front_Motor_ENB 19

// Servo Pins
#define BASE_SERVO 15
#define SHOULDER_SERVO 2
#define ELBOW_SERVO 5
#define GRIPPER_SERVO 4

Servo base, shoulder, elbow, gripper;

// Servo positions
int basePos = 90, shoulderPos = 90, elbowPos = 15, gripperPos = 90;

void setup() {
  Serial.begin(9600); // HC-05 default baud rate
  // Motor pins setup
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(Front_Motor_EN, OUTPUT);
  pinMode(Front_Motor_ENA, OUTPUT); pinMode(Front_Motor_ENB, OUTPUT);
  // Attach servos
  base.attach(BASE_SERVO);
  shoulder.attach(SHOULDER_SERVO);
  elbow.attach(ELBOW_SERVO);
  gripper.attach(GRIPPER_SERVO);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      // Car Movement
      case 'F': moveCar(200, 200); break;
      case 'B': moveCar(-200, -200); break;
      case 'L': moveCar(-150, 150); break;
      case 'R': moveCar(150, -150); break;
      case 'S': moveCar(0, 0); break;

      // Front Motor
      case 'M': digitalWrite(Front_Motor_EN, HIGH); break;
      case 'm': digitalWrite(Front_Motor_EN, LOW); break;

      // Arm Base
      case 'A': basePos += 5; break;   // Right
      case 'a': basePos -= 5; break;   // Left

      // Shoulder
      case 'U': shoulderPos += 5; break;
      case 'u': shoulderPos -= 5; break;

      // Elbow
      case 'E': elbowPos += 5; break;
      case 'e': elbowPos -= 5; break;

      // Gripper
      case 'G': gripperPos += 10; break;
      case 'g': gripperPos -= 10; break;
    }

    // Limit servo ranges
    basePos = constrain(basePos, 0, 180);
    shoulderPos = constrain(shoulderPos, 80, 180);
    elbowPos = constrain(elbowPos, 0, 90);
    gripperPos = constrain(gripperPos, 0, 180);

    // Update servos
    base.write(basePos);
    shoulder.write(shoulderPos);
    elbow.write(elbowPos);
    gripper.write(gripperPos);
  }
}

void moveCar(int leftSpeed, int rightSpeed) {
  digitalWrite(IN3, leftSpeed > 0);
  digitalWrite(IN4, leftSpeed < 0);
  analogWrite(ENB, abs(leftSpeed)*(170/300*1.00));
  analogWrite(Front_Motor_ENB, abs(leftSpeed));

  digitalWrite(IN1, rightSpeed > 0);
  digitalWrite(IN2, rightSpeed < 0);
  analogWrite(ENA, abs(rightSpeed));
  analogWrite(Front_Motor_ENA, abs(rightSpeed)*(170/300*1.00));
}
