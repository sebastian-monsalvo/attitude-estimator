#include "StandardCplusplus.h"
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long roll_pitch_yaw(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float& total_roll, float& total_pitch, float& total_yaw) {

    float dt = (float) currentMillis - startMillis;

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float total_roll_prev = total_roll;
    float total_pitch_prev = total_pitch;
    float total_yaw_prev = total_yaw;

    total_roll = total_roll + (dt / 1000.0) * gyro.x(); // dt in ms
    total_pitch = total_pitch + (dt / 1000.0) * gyro.y();
    total_yaw = total_yaw + (dt / 1000.0) * gyro.z();
    
    if (abs(total_roll - total_roll_prev) < 0.5) { // low pass filter, if innovation too small: dont count it
        total_roll = total_roll_prev;
    } else {
        total_roll = total_roll;
    }

    if (abs(total_pitch - total_pitch_prev) < 0.5) { // low pass filter, if innovation too small: dont count it
        total_pitch = total_pitch_prev;
    } else {
        total_pitch = total_pitch;
    }

    if (abs(total_yaw - total_yaw_prev) < 0.5) { // low pass filter, if innovation too small: dont count it
        total_yaw = total_yaw_prev;
    } else {
        total_yaw = total_yaw;
    }

    Serial.print(">dt:");
    Serial.println(dt);

    Serial.print(">total_roll:");
    Serial.println(total_roll);

    Serial.print(">total_pitch:");
    Serial.println(total_pitch);

    Serial.print(">total_yaw:");
    Serial.println(total_yaw);

    return millis();
}