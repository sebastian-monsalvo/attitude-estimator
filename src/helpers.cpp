#pragma once
#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long test(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno) {

    float dt = (float) currentMillis - startMillis;
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.print("   ");
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.println(gyro.x());
    return millis();
}