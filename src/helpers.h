#pragma once
#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Teleplot.h>

std::vector<float> f(std::vector<float> angles);

// big bug was you were not passing by reference so in void loop() total_roll would reset to 0 in every iteration lol
unsigned long roll(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float& total_roll, Teleplot tele);