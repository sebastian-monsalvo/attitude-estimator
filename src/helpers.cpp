#pragma once
#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long roll(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float total_roll) {

    float dt = (float) currentMillis - startMillis;
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.print("   ");
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    total_roll = total_roll + dt * gyro.x();
    Serial.println(total_roll);
    return millis();
}