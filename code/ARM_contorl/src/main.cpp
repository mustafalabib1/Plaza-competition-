#include <Arduino.h>

#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>

// Motor Pins (adjust to your wiring)
#define  rightMotor1  12
#define  rightMotor2  14
#define  rightMotorPWM  12
#define  leftMotor1 25
#define  leftMotor2 26
#define  leftMotorPWM 33

//d15 y
//d2 o
//d18 black 
// d19 blue 

// Servo Pins
#define BASE_SERVO 19
#define SHOULDER_SERVO 15
#define ELBOW_SERVO 18
#define GRIPPER_SERVO 2

Servo base, shoulder, elbow, gripper;

// Servo positions
int basePos = 90, shoulderPos = 90, elbowPos = 30, gripperPos = 90;

void setup() {
    Serial.begin(115200);
    PS4.begin("5c:96:56:af:ad:a0");  // Replace with ESP32 Bluetooth MAC address
    Serial.println("Waiting for PS4 Controller...");

    // Setup Motor Pins
    pinMode(leftMotorPWM, OUTPUT); pinMode(leftMotor1, OUTPUT); pinMode(leftMotor2, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT); pinMode(rightMotor1, OUTPUT); pinMode(rightMotor2, OUTPUT);

    // Attach Servos
    base.attach(BASE_SERVO);
    shoulder.attach(SHOULDER_SERVO);
    elbow.attach(ELBOW_SERVO);
    gripper.attach(GRIPPER_SERVO);
}

void loop() {
    if (PS4.isConnected()) {
        Serial.println("ps4 is connected");

        int speed = abs(PS4.LStickY()) * 2.55;
        int turn = PS4.LStickX();

        // Car Movement
        if (PS4.LStickY() < -10) moveCar(speed, speed);
        else if (PS4.LStickY() > 10) moveCar(-speed, -speed);
        else if (PS4.LStickX() < -10) moveCar(-speed, speed);
        else if (PS4.LStickX() > 10) moveCar(speed, -speed);
        else moveCar(0, 0);

        // Arm Controls
        basePos =90;
        if (PS4.Cross()) basePos=180;  // Rotate Right
        if (PS4.Circle()) basePos=0; // Rotate Left
        
        if (PS4.data.button.triangle) {shoulderPos += 1;} // Shoulder Up
        if (PS4.Square()) shoulderPos -= 1;;// Shoulder Down

        if (PS4.Up()) elbowPos += 5; // Elbow Up
        if (PS4.Down()) elbowPos -= 5; // Elbow Down

        if (PS4.R1()) gripperPos += 10; // Open Gripper
        if (PS4.L1()) gripperPos -= 10; // Close Gripper

        // Limit servo range
        basePos = constrain(basePos, 0, 180);
        shoulderPos = constrain(shoulderPos, 0, 180);
        elbowPos = constrain(elbowPos, 90, 180);
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
    digitalWrite(leftMotor1, leftSpeed > 0);
    digitalWrite(leftMotor2, leftSpeed < 0);
    analogWrite(leftMotorPWM, abs(leftSpeed));

    digitalWrite(rightMotor1, rightSpeed > 0);
    digitalWrite(rightMotor2, rightSpeed < 0);
    analogWrite(rightMotorPWM, abs(rightSpeed));
}