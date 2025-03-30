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

// Motor Pins (adjust to your wiring)
#define  rightMotor1  16
#define  rightMotor2  17
#define  rightMotorPWM  18
#define  leftMotor1 19
#define  leftMotor2 23 
#define  leftMotorPWM 5

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

  // ESP32 PWM Configuration
  ledc_timer_config_t timer_conf;
  timer_conf.duty_resolution = LEDC_TIMER_8_BIT;
  timer_conf.freq_hz = 5000;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_0;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf;
  channel_conf.gpio_num = rightMotorPWM;
  channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  channel_conf.channel = LEDC_CHANNEL_0;
  channel_conf.timer_sel = LEDC_TIMER_0;
  channel_conf.duty = 0;
  channel_conf.hpoint = 0;
  ledc_channel_config(&channel_conf);

  channel_conf.gpio_num = leftMotorPWM;
  channel_conf.channel = LEDC_CHANNEL_1;
  ledc_channel_config(&channel_conf);

 
  // Sensor Calibration
  lcd.clear();
  lcd.print("Calibrating...");
  for (int i = 0; i < 100; i++)
  {
    setMotor(rightMotor1, rightMotor2, 0, 100);
    setMotor(leftMotor1, leftMotor2, 1, -100);
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
  setMotor(rightMotor1, rightMotor2, 0, rightMotorSpeed);
  setMotor(leftMotor1, leftMotor2, 1, leftMotorSpeed);

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

void setMotor(int in1, int in2, int pwmChannel, int speed)
{
  // Set motor direction and speed
  if (speed < 0)
  {
    speed = -speed;
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  ledcWrite(pwmChannel, speed);
}

void wait()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}