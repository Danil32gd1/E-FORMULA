#include <QTRSensors.h>

#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             11

#define motorPower 6
#define button_pin 2

#define rightMaxSpeed 255
#define rightBaseSpeed 245
#define rightMotor1 7
#define rightMotor2 8
#define rightMotorPWM 9

#define leftMaxSpeed 255
#define leftBaseSpeed 245
#define leftMotor1 4
#define leftMotor2 5
#define leftMotorPWM 3

float Kp = 0.1;
float Kd = 0.85;

bool butt_flag = 0;
bool butt;
unsigned long last_press;
int clicks = 0;

int lastError = 0;

QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); 
  for (int i = 0; i < 150; i++)
  {
    qtra.calibrate(); 
  }
  digitalWrite(13, LOW);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);  
}


void loop()
{
  butt = !digitalRead(button_pin);
  unsigned int position = qtra.readLine(sensorValues);
  float error = position - 3500;
  int P = error;
  int D = error - lastError;
  lastError = error;

  int motorSpeedChange = P*Kp + D*Kd;

  int motorSpeedRight = rightBaseSpeed + motorSpeedChange;
  int motorSpeedLeft = leftBaseSpeed - motorSpeedChange;

  if (motorSpeedRight > rightMaxSpeed) {
    motorSpeedRight = rightMaxSpeed;
  }
  if (motorSpeedLeft > leftMaxSpeed) {
    motorSpeedLeft = leftMaxSpeed;
  }
  if (motorSpeedRight < 0) {
    motorSpeedRight = 0;
  }
  if (motorSpeedLeft < 0) {
    motorSpeedLeft = 0;
  }

  if (butt_flag == 0 && butt == 1 && clicks == 0 && millis() - last_press > 400) {
    delay(1000);
    butt_flag = 1;
    clicks += 1;
    digitalWrite(motorPower, HIGH);
    last_press = millis();
  }

  {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, motorSpeedRight);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, motorSpeedLeft);
  }

  if (butt == 0 && butt_flag == 1 && clicks == 1) {
    butt_flag = 0;
  }

  if (butt_flag == 0 && butt == 1 && clicks == 1 && millis() - last_press > 400) {
    digitalWrite(motorPower, LOW);
    last_press = millis();
    butt_flag = 1;
    clicks = 0;
  }
  
  if (butt == 0 && butt_flag == 1 && clicks == 0) {
    butt_flag = 0;
  }
}
