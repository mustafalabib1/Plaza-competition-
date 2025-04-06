#include <PS4Controller.h>
#include <ESP32Servo.h>

// Motor Driver Pins (L298N)
#define ENA 5
#define IN1 18
#define IN2 19
#define ENB 23
#define IN3 21
#define IN4 22

// Servo Pins
#define BASE_SERVO 13
#define SHOULDER_SERVO 12
#define ELBOW_SERVO 14
#define GRIPPER_SERVO 27

Servo base, shoulder, elbow, gripper;

// Servo positions
int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;

void setup() {
    Serial.begin(115200);
    PS4.begin("5c:96:56:af:ad:a1");  // Replace with ESP32 Bluetooth MAC address
    Serial.println("Waiting for PS4 Controller...");

    // Setup Motor Pins
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    // Attach Servos
    base.attach(BASE_SERVO);
    shoulder.attach(SHOULDER_SERVO);
    elbow.attach(ELBOW_SERVO);
    gripper.attach(GRIPPER_SERVO);
}

void loop() {
    if (PS4.isConnected()) {
        int speed = abs(PS4.LStickY()) * 2.55;
        int turn = PS4.LStickX();

        // Car Movement
        if (PS4.LStickY() < -10) moveCar(speed, speed);
        else if (PS4.LStickY() > 10) moveCar(-speed, -speed);
        else if (PS4.LStickX() < -10) moveCar(-speed, speed);
        else if (PS4.LStickX() > 10) moveCar(speed, -speed);
        else moveCar(0, 0);

        // Arm Controls
        if (PS4.Cross()) basePos += 5;  // Rotate Right
        if (PS4.Circle()) basePos -= 5; // Rotate Left
        if (PS4.Triangle()) shoulderPos += 5; // Shoulder Up
        if (PS4.Square()) shoulderPos -= 5; // Shoulder Down
        if (PS4.Up()) elbowPos += 5; // Elbow Up
        if (PS4.Down()) elbowPos -= 5; // Elbow Down
        if (PS4.R1()) gripperPos += 5; // Open Gripper
        if (PS4.L1()) gripperPos -= 5; // Close Gripper

        // Limit servo range
        basePos = constrain(basePos, 0, 180);
        shoulderPos = constrain(shoulderPos, 0, 180);
        elbowPos = constrain(elbowPos, 0, 180);
        gripperPos = constrain(gripperPos, 0, 180);

        // Move servos
        base.write(basePos);
        shoulder.write(shoulderPos);
        elbow.write(elbowPos);
        gripper.write(gripperPos);

        delay(100);
    }
}

void moveCar(int leftSpeed, int rightSpeed) {
    digitalWrite(IN1, leftSpeed > 0);
    digitalWrite(IN2, leftSpeed < 0);
    analogWrite(ENA, abs(leftSpeed));

    digitalWrite(IN3, rightSpeed > 0);
    digitalWrite(IN4, rightSpeed < 0);
    analogWrite(ENB, abs(rightSpeed));
}