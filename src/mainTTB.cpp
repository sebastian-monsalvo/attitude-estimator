#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

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
float total_roll_rad = 0;
float total_pitch_rad = 0;
float mag_x_proj = 0;
float mag_y_proj = 0;
float psi_mag = 0;
float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;
float mag_x;
float mag_y;
float mag_z;
float norm_phi;

void setup() {
  Serial.begin(115200);
  bno.begin();
  bno.setExtCrystalUse(true);
  startMillis = millis();
  total_roll = 0.0;
  total_pitch = 0.0;
  total_yaw = 0.0;
}

void loop() {
    delay(100);
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
    mag_x = -mag.x();
    mag_y = -mag.y();
    mag_z = mag.z();

    if (accel_x == 0.0) { // necessary because we cant have atan(0)
        return;
    }

    phi_accel = atan2(-accel_y, -accel_z) * 180.0 / 3.1415;
    theta_accel = -atan2(-accel_x, -accel_z) * 180.0 / 3.1415;

    phi_gyro = total_roll + gyro_x * dt;
    theta_gyro = total_pitch + gyro_y * dt;

    total_roll = 0.0 * phi_gyro + 1.0 * phi_accel;
    total_pitch = 0.95 * theta_gyro + 0.05 * theta_accel;

    total_roll_rad = total_roll * 3.1415 / 180.0;
    total_pitch_rad = total_pitch * 3.1415 / 180.0;
    // mag_x_proj = cos(total_pitch_rad) * mag_x;
    mag_y_proj = mag_y * cos(total_roll_rad) - mag_z * sin(total_roll_rad);
    mag_x_proj = mag_x * cos(total_pitch_rad) + mag_z * sin(total_pitch_rad);
    psi_mag = -atan2(mag_y_proj, mag_x_proj) * 180.0 / 3.1415;

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
    // Serial.print(">A_cal: ");
    // Serial.println(accelerometer);
    // Serial.print(">G_cal: ");
    // Serial.println(gyroscope);
    // Serial.print(">M_Cal: ");
    // Serial.println(magnetometer);
    // Serial.print(">Sys: ");
    // Serial.println(system);
    // Serial.print(">Roll: ");
    // Serial.println(total_roll);
    // Serial.print(">Pitch: ");
    // Serial.println(total_pitch);
    // Serial.print(">Accel_x: ");
    // Serial.println(accel_x);
    // Serial.print(">Accel_y: ");
    // Serial.println(accel_y);
    // Serial.print(">Accel_z: ");
    // Serial.println(accel_z);
    Serial.print(">Magx: ");
    Serial.println(mag_x);
    Serial.print(">Magy: ");
    Serial.println(mag_y);
    Serial.print(">Magz: ");
    Serial.println(mag_z);
    Serial.print(">Psi: ");
    Serial.println(psi_mag);
    Serial.print(">mag_y_proj: ");
    Serial.println(mag_y_proj);
    Serial.print(">mag_x_proj: ");
    Serial.println(mag_x_proj);

}