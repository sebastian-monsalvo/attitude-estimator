#pragma once
// #include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>


// big bug was you were not passing by reference so in void loop() total_roll would reset to 0 in every iteration lol
unsigned long roll_pitch_yaw(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float& total_roll, float& total_pitch, float& total_yaw);

BLA::Matrix<3> f(BLA::Matrix<3> angles);

BLA::Matrix<6> g(BLA::Matrix<3> angles_prev, BLA::Matrix<3> angles, float dt);

BLA::Matrix<6, 6> R();