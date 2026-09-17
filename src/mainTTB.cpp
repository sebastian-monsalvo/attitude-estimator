#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long startMillis;
unsigned long currentMillis;
float total_roll;
float total_pitch;
float total_yaw;
float dt;
float phi_accel;
float theta_accel;
float phi_gyro = 0;
float theta_gyro = 0;
float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;
float mag_x;
float mag_y;
float mag_z;

void setup() {
  Serial.begin(115200);
  bno.begin();
  bno.setExtCrystalUse(true);
  startMillis = millis();
  total_roll = 0;
  total_pitch = 0;
  total_yaw = 0;
}

void loop() {
    uint8_t system, gyroscope, accelerometer, magnetometer = 0;
    bno.getCalibration(&system, &gyroscope, &accelerometer, &magnetometer);
    currentMillis = millis();
    dt = (float) currentMillis - startMillis;
    startMillis = currentMillis;
    dt = dt / 1000.0;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro_x = -gyro.x();
    gyro_y = -gyro.y();
    gyro_z = gyro.z();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accel_x = accel.x();
    accel_y = accel.y();
    accel_z = -accel.z();
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mag_x = mag.x();
    mag_y = mag.y();
    mag_z = mag.z();


    phi_accel = atan2(accel_y, accel_z) * 180.0 / 3.1415; 
    if (phi_accel <= 0) {
        phi_accel = phi_accel + 180.0;
    } else {
        phi_accel = phi_accel -180;
    }
    theta_accel = atan2(accel_x, accel_z) * 180.0 / 3.1415;

    phi_gyro = total_roll + gyro_x * dt;
    theta_gyro = total_pitch + gyro_y * dt;

    total_roll = 0.0 * phi_gyro + 1.0 * phi_accel;
    total_pitch = 0.95 * theta_gyro + 0.05 * theta_accel;

    // Serial.print(accelerometer);
    // Serial.print(",");
    // Serial.print(gyroscope);
    // Serial.print(",");
    // Serial.print(magnetometer);
    // Serial.print(",");
    // Serial.print(system);
    // Serial.print(",");
    // Serial.print(total_roll);
    // Serial.print(",");
    // Serial.println(total_pitch);

    // Teleplot
    Serial.print(">A_cal: ");
    Serial.println(accelerometer);
    Serial.print(">G_cal: ");
    Serial.println(gyroscope);
    Serial.print(">M_Cal: ");
    Serial.println(magnetometer);
    Serial.print(">Sys: ");
    Serial.println(system);
    Serial.print(">Roll: ");
    Serial.println(total_roll);
    Serial.print(">Pitch: ");
    Serial.println(total_pitch);
    Serial.print(">Accel_x: ");
    Serial.println(accel_x);
    Serial.print(">Accel_y: ");
    Serial.println(accel_y);
    Serial.print(">Accel_z: ");
    Serial.println(accel_z);
    Serial.print(">Arg: ");
    float arg = accel_y / accel_z;
    Serial.println(arg);
    delay(100);
}