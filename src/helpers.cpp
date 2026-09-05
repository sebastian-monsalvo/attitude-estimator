#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>

unsigned long roll_pitch_yaw(unsigned long startMillis, unsigned long currentMillis, Adafruit_BNO055 bno, float& total_roll, float& total_pitch, float& total_yaw) {

    float dt = (float) currentMillis - startMillis;

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
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

    // Serial.print(">dt:");
    // Serial.println(dt);

    // Serial.print(">gyro_x:");
    // Serial.println(accel.x());

    // Serial.print(">gyro_y:");
    // Serial.println(accel.y());

    // Serial.print(">gyro_z:");
    // Serial.println(accel.z());

    return millis();
}

BLA::Matrix<3> f(BLA::Matrix<3> angles) {
    BLA::Matrix<3> result = angles;
    BLA::Matrix<3,3> identity;
    identity.Fill(0.0);
    identity(0,0) = 1.0;
    identity(1,1) = 1.0;
    identity(2,2) = 1.0;
    result = identity * result;
    return result;
}

BLA::Matrix<6> g(BLA::Matrix<3> angles_prev, BLA::Matrix<3> angles, float dt) {
    BLA::Matrix<6> result;
    result.Fill(0.0);

    dt = dt / 1000.0;
    float phi = angles(0) * PI / 180.0; //convert to radians so sin and cos work
    float theta = angles(1) * PI / 180.0;
    float psi = angles(2) * PI / 180.0;
    float over_dt = 1.0 / dt;

    BLA::Matrix<3, 3> B_inv = {cos(theta) * cos(psi), sin(psi), 0.0, -cos(theta) * sin(psi), cos(psi), 0.0, sin(theta), 0.0, 1.0};
    B_inv = B_inv * over_dt;

    BLA::Matrix<3, 3> R;
    R(0, 0) = cos(psi) * cos(theta);
    R(0, 1) = cos(psi) * sin(theta) * sin(phi) + sin(psi) * cos(phi);
    R(0, 2) = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
    R(1, 0) = -sin(psi) * cos(theta);
    R(1, 1) = -sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
    R(1, 2) = sin(psi) * sin(theta) * cos(phi) + cos(psi) * sin(phi);
    R(2, 0) = sin(theta);
    R(2, 1) = -cos(theta) * sin(phi);
    R(2, 2) = cos(theta) * cos(phi);

    BLA::Matrix<3> angular_velocities = B_inv * angles - B_inv * angles_prev;
    BLA::Matrix<3> gravity = {0.0, 0.0, -9.81};
    BLA::Matrix<3> accelerations = R * gravity;

    result = angular_velocities && accelerations;

    return result;
}

BLA::Matrix<6, 6> calculate_R(){
    BLA::Matrix<6, 6> result;
    result.Fill(0);

    result(0, 0) = sq(0.6);
    result(1, 1) = sq(0.6);
    result(2, 2) = sq(0.6);
    result(3, 3) = sq(0.01);
    result(4, 4) = sq(0.01);
    result(5, 5) = sq(0.025);

    return result;
}

BLA::Matrix<3, 3> calculate_Q() {
    BLA::Matrix<3, 3> result;
    result.Fill(0);

    result(0, 0) = sq(1.0); // like maybe i rotate the IMU and it is 1 degree off
    result(1, 1) = sq(1.0);
    result(2, 2) = sq(1.0);

    return result;
}

BLA::Matrix<3, 3> calculate_A() {
    BLA::Matrix<3, 3> result;
    result.Fill(0);

    result(0, 0) = 1.0;
    result(1, 1) = 1.0;
    result(2, 2) = 1.0;

    return result;
}

BLA::Matrix<6, 3> calculate_C(BLA::Matrix<3> angles_prev, BLA::Matrix<3> angles, float dt) {
    BLA::Matrix<3, 3> top_half;
    top_half.Fill(0);

    dt = dt / 1000.0;
    float phi = angles(0) * PI / 180.0; //convert to radians so sin and cos work
    float theta = angles(1) * PI / 180.0;
    float psi = angles(2) * PI / 180.0;

    BLA::Matrix<3> d_angles_dt = (angles - angles_prev) / dt;
    float phi_dot = d_angles_dt(0);
    float theta_dot = d_angles_dt(1);

    top_half(0, 1) = phi_dot * cos(psi) * (-sin(theta));
    top_half(0, 2) = phi_dot * cos(theta) * (-sin(psi)) + theta_dot * cos(psi);
    top_half(1, 1) = -phi_dot * sin(psi) * (-sin(theta));
    top_half(1, 2) = -phi_dot * cos(theta) * cos(psi) + phi_dot * (-sin(psi));
    top_half(2, 1) = phi_dot * cos(theta);
    
    BLA::Matrix<3, 3> bottom_half;
    bottom_half.Fill(0);
    bottom_half(0, 0) = -9.81 * (-cos(psi) * sin(theta) * (-sin(theta)) + sin(psi) * cos(phi));
    bottom_half(0, 1) = -9.81 * (-cos(psi) * cos(theta) * cos(phi));
    bottom_half(0, 2) = -9.81 * (sin(psi) * sin(theta) * cos(phi) + (-sin(psi) * sin(phi)));
    bottom_half(1, 0) = -9.81 * (sin(psi) * sin(theta) * (-sin(phi)) + cos(psi) * cos(phi));
    bottom_half(1, 1) = -9.81 * (sin(psi) * cos(theta) * cos(phi));
    bottom_half(1, 2) = -9.81 * (cos(psi) * sin(theta) * cos(phi) + (-sin(psi)) * sin(phi));
    bottom_half(1, 3) = -9.81 * (cos(psi) * sin(theta) * cos(phi) + (-sin(psi)) * sin(phi));
    bottom_half(2, 0) = -9.81 * (cos(theta) * (-sin(phi)));
    bottom_half(2, 1) = -9.81 * (-sin(theta) * cos(phi));

    return top_half && bottom_half;
    
}

BLA::Matrix<6> get_y(Adafruit_BNO055 bno) {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY); // change to accel after calibrating IMU

    BLA::Matrix<6> y = {gyro.x(), gyro.y(), gyro.z(), accel.x(), accel.y(), accel.z()};
    return y;
}