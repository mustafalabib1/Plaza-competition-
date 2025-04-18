#include <Servo.h>

Servo myServo;
int servoPin = 9;
int angle, pre = 0;
void setup() {
  // put your setup code here, to run once:
  myServo.attach(servoPin);
  Serial.begin(9600);
  myServo.write(0);
}
void loop() {

  if (Serial.available()) {
    int angle = Serial.parseInt();
    if (angle!=0)
    pre=angle;
    else if (angle<0)
    pre=0;
  }
    myServo.write(pre);
}
