#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "helpers.h"
#include <BasicLinearAlgebra.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long startMillis;
unsigned long currentMillis;
float total_roll;
float total_pitch;
float total_yaw;
BLA::Matrix<3> angles = {0, 0, 0};
BLA::Matrix<6> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
BLA::Matrix<6, 6> R;
BLA::Matrix<6, 6> Q;
BLA:: Matrix<3, 3> A;


void setup() {
  Serial.begin(9600);
  bno.begin();
  bno.setExtCrystalUse(true);
  startMillis = millis();
  total_roll = 0;
  total_pitch = 0;
  total_yaw = 0;
  R = calculate_R();
  Q = calculate_Q();
  A = calculate_A();
  
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - startMillis >= 100) {
    startMillis = roll_pitch_yaw(startMillis, currentMillis, bno, total_roll, total_pitch, total_yaw);
  }

  BLA::Matrix<3> angles_prev = angles;
  angles = f(angles_prev);
  // Serial.print("angles: ");
  // Serial.println(angles);

  y = g(angles_prev, angles, 0.1);
  // Serial.print("y: ");
  // Serial.println(y);

  
}
