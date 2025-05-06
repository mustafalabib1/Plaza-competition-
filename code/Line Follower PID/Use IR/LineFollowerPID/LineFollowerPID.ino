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
#define FarLeft_SENSOR A4
#define LEFT_SENSOR A3
#define MIDDLE_SENSOR A2
#define Right_SENSOR A1
#define FarRight_SENSOR A0

// PD Control Parameters
float Kp = 128;  // Increased proportional gain for sharp corrections
float Kd = 165;  // Adjusted derivative gain
int previousError = 0;
const int BASE_SPEED = 130;
const int MAX_SPEED = 255;
const int RECOVERY_DURATION = 350;  // Milliseconds for sharp turns

const float weights[5] = { -2.0, -1.0, 0.0, 1.0, 2.0 };
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

  bool FarLeft = digitalRead(FarLeft_SENSOR);
  bool LEFT = digitalRead(LEFT_SENSOR);
  bool MIDDLE = digitalRead(MIDDLE_SENSOR);
  bool Right = digitalRead(Right_SENSOR);
  bool FarRight = digitalRead(FarRight_SENSOR);


  float error = calculateError(FarLeft, LEFT, MIDDLE, Right, FarRight);

  // PD Control
  int motorAdjust = Kp * error + Kd * (error - previousError);
  previousError = error;

  int leftSpeed = BASE_SPEED + motorAdjust;
  int rightSpeed = BASE_SPEED - motorAdjust;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  moveCar(leftSpeed, rightSpeed);
  debugOutput(FarLeft, LEFT, MIDDLE, Right, FarRight, error, leftSpeed, rightSpeed);
}

float calculateError(bool fl, bool l, bool m, bool r, bool fr) {
  // Sensor positions with weights:
  // Far Left (-2) | Left (-1) | Middle (0) | Right (+1) | Far Right (+2)
  float weightedSum = 0;
  int activeSensors = 0;

  // Acute angle detection
  if (fl && !l && !r && !fr) return -3.0;  // Extreme left
  if (fr && !fl && !m && !r) return 3.0;   // Extreme right

  // Standard line following
  if (fl) {
    weightedSum += weights[0];
    activeSensors++;
  }
  if (l) {
    weightedSum += weights[1];
    activeSensors++;
  }
  if (m) {
    weightedSum += weights[2];
    activeSensors++;
  }
  if (r) {
    weightedSum += weights[3];
    activeSensors++;
  }
  if (fr) {
    weightedSum += weights[4];
    activeSensors++;
  }

  // Line lost handling
  if (activeSensors == 0||activeSensors==5) return previousError * 1.1;  // Boost last known error

  // Normalized error (-2.0 to +2.0)
  float error = weightedSum / activeSensors;

  // Edge case boosters
  if (fl && l) error *= 1.5;  // Strong left curve
  if (fr && r) error *= 1.5;  // Strong right curve

  if (l && m) error *= 3;
  if (r && m) error *= 3;

  return constrain(error, -3.0, 3.0);
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

void debugOutput(bool fl, bool l, bool m, bool r, bool fr, float error, int lSpeed, int rSpeed) {
  Serial.print("S: ");
  Serial.print(fl);
  Serial.print(" ");
  Serial.print(l);
  Serial.print(" ");
  Serial.print(m);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(fr);
  Serial.print(" | E: ");
  Serial.print(error);
  Serial.print(" | L: ");
  Serial.print(lSpeed);
  Serial.print(" R: ");
  Serial.println(rSpeed);
}