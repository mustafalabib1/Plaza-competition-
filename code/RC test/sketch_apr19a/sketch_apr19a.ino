#include <Servo.h>

// Motor Pins
#define ENA 5  // Right motor PWM
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 6  // Left motor PWM
#define Front_Motor_ENA 3
#define Front_Motor_ENB 11

// Sensor Pins
#define LEFT_SENSOR A2
#define MIDDLE_SENSOR A1
#define RIGHT_SENSOR A0

// PD Control Parameters
float Kp = 250;  // Proportional gain
float Kd = 150;  // Derivative gain
int previousError = 0;
const int BASE_SPEED = 150;
const int MAX_SPEED = 200;

// Line Detection Thresholds

unsigned long delayTime = 50;

void setup() {
  Serial.begin(9600);

  // Motor Setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(Front_Motor_ENA, OUTPUT);
  pinMode(Front_Motor_ENB, OUTPUT);
}

void loop() {
  int leftVal = digitalRead(LEFT_SENSOR);
  int middleVal = digitalRead(MIDDLE_SENSOR);
  int rightVal = digitalRead(RIGHT_SENSOR);

  float error = calculateError(leftVal, middleVal, rightVal);

  // PD Control
  int motorAdjust = Kp * error + Kd * (error - previousError);
  previousError = error;

  int leftSpeed = BASE_SPEED + motorAdjust;
  int rightSpeed = BASE_SPEED - motorAdjust;

  // Speed constraints
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  moveCar(leftSpeed, rightSpeed);
  debugOutput(leftVal, middleVal, rightVal, error, leftSpeed, rightSpeed);
  // delay(delayTime);
}

float calculateError(bool left, bool mid, bool right) {
  /* Error values:
   *  -2: Far left
   *  -1: Left
   *   0: Center
   *   1: Right
   *   2: Far right
   *  -1000: Line lost
   */

  if (mid) {
    if (left && !right) return -1.5;  // Left curve
    if (!left && right) return 1.5;   // Right curve
    return 0;                       // Straight
  } else {
    if (left) return -1;  // Far left
    if (right) return 1;  // Far right
    return previousError;         // Line lost
  }
}

void moveCar(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  digitalWrite(IN3, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, leftSpeed < 0 ? HIGH : LOW);
  analogWrite(ENB, abs(leftSpeed));

  // Right Motor Control
  digitalWrite(IN1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, rightSpeed < 0 ? HIGH : LOW);
  analogWrite(ENA, abs(rightSpeed));

  // Front Motors (mirror main motors)
  analogWrite(Front_Motor_ENB, abs(leftSpeed));
  analogWrite(Front_Motor_ENA, abs(rightSpeed));
}

void debugOutput(int left, int mid, int right, float error, int lSpeed, int rSpeed) {
  Serial.print("Sensors: ");
  Serial.print(left);
  Serial.print(" ");
  Serial.print(mid);
  Serial.print(" ");
  Serial.print(right);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Speeds: ");
  Serial.print(lSpeed);
  Serial.print(" ");
  Serial.println(rSpeed);
}