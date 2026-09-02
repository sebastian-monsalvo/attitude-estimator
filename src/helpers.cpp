#pragma once
#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Teleplot.h>

unsigned long roll(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float& total_roll, Teleplot tele) {

    float dt = (float) currentMillis - startMillis;

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float total_roll_prev = total_roll;
    total_roll = total_roll + (dt / 1000.0) * gyro.x(); // dt in ms
    
    if (abs(total_roll - total_roll_prev) < 0.5) { // low pass filter, if innovation too small: dont count it
        total_roll = total_roll_prev;
    } else {
        total_roll = total_roll;
    }
    Serial.print(">raw_gyro_x:");
    Serial.println(gyro.x());

    Serial.print(">total_roll:");
    Serial.println(total_roll);

    Serial.print(">dt:");
    Serial.println(dt);

    return millis();
}