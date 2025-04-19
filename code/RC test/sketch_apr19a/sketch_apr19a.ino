#include "BluetoothSerial.h"

BluetoothSerial BT;

#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 33
#define IN3 14
#define IN4 12

#define HEADLIGHT_PIN 2  // Headlight control

char command;

void setup() {
  Serial.begin(115200);
  BT.begin("ESP32-RC-Car"); // Bluetooth name

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Headlight pin
  pinMode(HEADLIGHT_PIN, OUTPUT);
  digitalWrite(HEADLIGHT_PIN, LOW); // Start OFF
}

void loop() {
  if (BT.available()) {
    command = BT.read();
    Serial.println(command);
    handleCommand(command);
  }
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'H': digitalWrite(HEADLIGHT_PIN, HIGH); break;
    case 'h': digitalWrite(HEADLIGHT_PIN, LOW); break;
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
