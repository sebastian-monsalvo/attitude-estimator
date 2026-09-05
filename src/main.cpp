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
float dt;
BLA::Matrix<3> angles = {0, 0, 0};
BLA::Matrix<6> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
BLA::Matrix<6, 6> R;
BLA::Matrix<3, 3> Q;
BLA:: Matrix<3, 3> A;
BLA:: Matrix<3, 3> A_T;
BLA:: Matrix<6, 3> C;
BLA:: Matrix<3, 6> C_T;

// Initialization
BLA::Matrix<3> x_0 = {-2.0, -180.0, 2200.0};
BLA::Matrix<3, 3> P_0 = calculate_A(); // 3x3 identity for now

BLA::Matrix<3> x_10;
BLA::Matrix<3> x_11;
BLA::Matrix<3, 3> P_10;
BLA::Matrix<3, 3> P_11;
BLA::Matrix<3, 6> K;


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
  A_T = ~A;
  dt = 0.001;
  K.Fill(0);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - startMillis >= 100) {
    startMillis = roll_pitch_yaw(startMillis, currentMillis, bno, total_roll, total_pitch, total_yaw);
  }

  BLA::Matrix<3> angles_prev = angles;
  angles = f(angles_prev);

  y = g(angles_prev, angles, 0.1);
  
  C = calculate_C(angles_prev, angles, 0.1);

  // Prediction
  x_10 = f(x_0);
  P_10 = A * P_0 * A_T + Q;

  // Update
  C = calculate_C(x_0, x_10, dt);
  C_T = ~C;
  K = P_10 * C_T * Inverse(C * P_10 * C_T + R);
  y = get_y(bno);
  x_11 = x_10 + K * (y - g(x_0, x_10, dt));
  P_11 = P_10 - K * C * P_10;

  // for t + 1
  P_0 = P_11;
  x_0 = x_11;

  Serial.print(">Phi: ");
  Serial.println(x_11(0));

  Serial.print(">Theta: ");
  Serial.println(x_11(1));

  Serial.print(">Psi: ");
  Serial.println(x_11(2));
}
