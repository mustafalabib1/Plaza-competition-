#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Use ESP32-compatible version
#include <driver/ledc.h>       // ESP32-specific PWM

// LCD Configuration (0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// QTR Sensors
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 2
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Motors Pins
#define rightMotor1 16
#define rightMotor2 17
#define rightMotorPWM 18
#define leftMotor1 19
#define leftMotor2 23
#define leftMotorPWM 5

// Potentiometer pins for tuning
#define KP_POT 34
#define KI_POT 35
#define KD_POT 32

// Motor control parameters
#define rightMaxSpeed 200
#define leftMaxSpeed 200
#define rightBaseSpeed 150
#define leftBaseSpeed 150

// PID Constants (adjust through potentiometers)
float Kp = 0.1;
float Ki = 0.001;
float Kd = 0.5;
int lastError = 0;
int integral = 0; // For integral term
unsigned long lastDisplayUpdate = 0;

void setup()
{
  Serial.begin(115200);

  // LCD Init
  lcd.init();
  lcd.backlight();
  lcd.print("Starting...");

  // QTR Sensor Setup
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){36, 39, 34, 35, 32, 33, 25, 26}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  // Motor Control Setup
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);

  // Sensor Calibration
  lcd.clear();
  lcd.print("Calibrating...");
  for (int i = 0; i < 100; i++)
  {
    moveCar(75, -75);
    qtr.calibrate();
    delay(20);
  }
  wait(); // Stop motors after calibration
  lcd.clear();
  lcd.print("Ready!");
  delay(3000);
}

void loop()
{
  LineFollowerPID();
}

void LineFollowerPID()
{
  while (true)
  {
    // Read PID values from potentiometers
    Kp = map(analogRead(KP_POT), 0, 4095, 0, 1000) / 1000.0; // 0.0 to 1.0
    Ki = map(analogRead(KI_POT), 0, 4095, 0, 500) / 10000.0; // 0.0 to 0.05
    Kd = map(analogRead(KD_POT), 0, 4095, 0, 1000) / 1000.0; // 0.0 to 1.0

    unsigned int sensors[NUM_SENSORS];
    // Read line position
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;

    // Calculate PID terms
    integral += error;
    integral = constrain(integral, -1000, 1000); // Anti-windup

    int motorSpeed = Kp * error + Ki * integral + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    // Constrain motor speeds
    rightMotorSpeed = constrain(rightMotorSpeed, -rightMaxSpeed, rightMaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, -rightMaxSpeed, leftMaxSpeed);

    // Motor control
    moveCar(leftMotorSpeed, rightMotorSpeed);

    // Update LCD every 200ms to prevent flickering
    if (millis() - lastDisplayUpdate > 200)
    {
      lastDisplayUpdate = millis();

      // First line: PID values
      lcd.setCursor(0, 0);
      lcd.print("Kp:");
      lcd.print(Kp, 3);
      lcd.print(" Ki:");
      lcd.print(Ki, 3);

      // Second line: Kd and position
      lcd.setCursor(0, 1);
      lcd.print("Kd:");
      lcd.print(Kd, 3);
      lcd.print(" Pos:");
      lcd.print(position);
      lcd.print("   "); // Clear any remaining characters
    }
  }
}
void moveCar(int leftSpeed, int rightSpeed)
{
  digitalWrite(leftMotor1, leftSpeed > 0);
  digitalWrite(leftMotor2, leftSpeed < 0);
  analogWrite(leftMotorPWM, abs(leftSpeed));

  digitalWrite(rightMotor1, rightSpeed > 0);
  digitalWrite(rightMotor2, rightSpeed < 0);
  analogWrite(rightMotorPWM, abs(rightSpeed));
}

void wait()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}