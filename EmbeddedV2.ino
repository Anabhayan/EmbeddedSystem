"""
Written by Anabhayan September 7th/8th
Edited by Adhithya September 8th

Core code structure (PID) adapted from CurioRES https://youtu.be/dTGITLnYAY0
"""

#include <NewPing.h>

const float SET_POINT = 3.0;  // Desired water level in centimeters
const float USS_POS = 10.19;  // Height of USS from the bottom of the beaker (without water)

// PID constants
const float KP = 1.0;  // Proportional gain
const float KI = 0.0;  // Integral gain
const float KD = 0.0;  // Derivative gain
const float K = 100;    // Scaling factor

#define PWM_OUT 5
const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;
const int MAX_DISTANCE = 100;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float distance ;
long prevMicros = 0;
float prevError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PWM_OUT, OUTPUT);
}

void loop() {
  long currentMicros = micros();
  float deltaTime = (float)(currentMicros - prevMicros) / 1.0e6;
  prevMicros = currentMicros;

  distance = USS_POS - measureDistance();
  float error = SET_POINT - distance;
  
  integral += error * deltaTime;
  float derivative = (error - prevError) / deltaTime;
  
  float output = KP * error + KI * integral + KD * derivative;
  
  int pwmOutput = int(output * K);
  pwmOutput = constrain(pwmOutput, 0, 255);

  prevError = error;

  controlPump(pwmOutput);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" Setpoint: ");
  Serial.print(SET_POINT);
  Serial.print(" Distance: ");
  Serial.println(distance);

}

float measureDistance() {
  float cm = (sonar.ping() / 2.0) * 0.0343;  // Convert ping time to centimeters
  return cm;
}

void controlPump(int pwm) {
  analogWrite(PWM_OUT, pwm);
}
