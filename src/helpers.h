#pragma once
#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

std::vector<float> f(std::vector<float> angles);

unsigned long test(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno);