#include <PS4Controller.h>
#include <ESP32Servo.h>

// Motor Driver Pins (L298N)
#define ENA 12
#define IN3 27
#define IN4 14

#define IN1 25
#define IN2 26

#define ENB 33

#define Front_Motor_EN 13

// // Sensor Pins
#define FarLeft_SENSOR 16
#define LEFT_SENSOR 17
#define MIDDLE_SENSOR 18
#define Right_SENSOR 19
#define FarRight_SENSOR 23

// Servo Pins
#define BASE_SERVO 15     //رمادي
#define SHOULDER_SERVO 2  //بني
#define ELBOW_SERVO 5     //برتقالي
#define GRIPPER_SERVO 4   //ابيض

Servo base, shoulder, elbow, gripper;

// Servo positions
int basePos = 90, shoulderPos = 90, elbowPos = 15, gripperPos = 90;
void moveCar(int leftSpeed, int rightSpeed);

void setup() {
  Serial.begin(115200);
  PS4.begin("5c:96:56:af:ad:a0");  // Replace with your ESP32's MAC address
  Serial.println("Waiting for PS4 Controller...");

  // Motor Pins Setup
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(Front_Motor_EN, OUTPUT);

  // Initialize IR Sensor Pins
  pinMode(FarLeft_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(MIDDLE_SENSOR, INPUT);
  pinMode(Right_SENSOR, INPUT);
  pinMode(FarRight_SENSOR, INPUT);

  // Attach Servos
  base.attach(BASE_SERVO);
  shoulder.attach(SHOULDER_SERVO);
  elbow.attach(ELBOW_SERVO);
  gripper.attach(GRIPPER_SERVO);
}

void loop() {
  if (PS4.isConnected()) {
    // Front Motor Control (R1 trigger)
    digitalWrite(Front_Motor_EN, PS4.R1());

    // Replace the existing movement logic with:
    float deadzone = 0.15f;  // 15% deadzone
    float y = constrain(-PS4.LStickY() / 127.0f, -1.0f, 1.0f);
    float x = constrain(PS4.LStickX() / 127.0f, -1.0f, 1.0f);

    // Apply deadzone
    if (fabs(y) < deadzone) y = 0;
    if (fabs(x) < deadzone) x = 0;

    // Non-linear response curve
    y = y * fabs(y);  // Square the input for better control
    x = x * fabs(x);

    // Mixing algorithm for differential drive
    int leftSpeed = constrain((y + x) * 255, -255, 255);
    int rightSpeed = constrain((y - x) * 255, -255, 255);

    moveCar(leftSpeed, rightSpeed);

    // --- Existing Arm Control Logic (unchanged) ---
    basePos = 90;
    if (PS4.R2()) basePos = 150;
    if (PS4.L2()) basePos = 30;

    if (PS4.Triangle()) shoulderPos += 1;
    if (PS4.Square()) shoulderPos -= 1;

    if (PS4.Up()) elbowPos += 1;
    if (PS4.Down()) elbowPos -= 1;

    if (PS4.Circle()) gripperPos += 1;
    if (PS4.Cross()) gripperPos -= 1;

    basePos = constrain(basePos, 0, 180);
    shoulderPos = constrain(shoulderPos, 80, 180);
    elbowPos = constrain(elbowPos, 0, 180);
    gripperPos = constrain(gripperPos, 0, 180);

    base.write(basePos);
    shoulder.write(shoulderPos);
    elbow.write(elbowPos);
    gripper.write(gripperPos);

    delay(10);
  }
}

void moveCar(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  digitalWrite(IN3, leftSpeed > 0);
  digitalWrite(IN4, leftSpeed < 0);
  analogWrite(ENB, abs(leftSpeed));

  // Right Motor Control
  digitalWrite(IN1, rightSpeed > 0);
  digitalWrite(IN2, rightSpeed < 0);
  analogWrite(ENA, abs(rightSpeed));
}