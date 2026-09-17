// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>
// #include "helpers.h"
// #include <BasicLinearAlgebra.h>

// Adafruit_BNO055 bno = Adafruit_BNO055(55);

// unsigned long startMillis;
// unsigned long currentMillis;
// float total_roll;
// float total_pitch;
// float total_yaw;
// float dt;
// BLA::Matrix<6> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// BLA::Matrix<6, 6> R = calculate_R();
// BLA::Matrix<3, 3> Q = calculate_Q();
// BLA:: Matrix<3, 3> A = calculate_A();
// BLA:: Matrix<3, 3> A_T = ~A;
// BLA:: Matrix<6, 3> C;
// BLA:: Matrix<3, 6> C_T;

// // Initialization
// BLA::Matrix<3> x_neg = {0.0, 0.0, 0.0};
// BLA::Matrix<3> x_0 = {0.0, 0.0, 0.0};
// BLA::Matrix<3, 3> P_0 = calculate_A(); // 3x3 identity for now

// BLA::Matrix<3> x_10;
// BLA::Matrix<3> x_11;
// BLA::Matrix<3, 3> P_10;
// BLA::Matrix<3, 3> P_11;
// BLA::Matrix<3, 6> K;


// void setup() {
//   Serial.begin(9600);
//   bno.begin();
//   bno.setExtCrystalUse(true);
//   startMillis = millis();
//   total_roll = 0;
//   total_pitch = 0;
//   total_yaw = 0;
// }

// void loop() {

//   currentMillis = millis();
//   dt = (float) currentMillis - startMillis;

//   if (dt >= 10) {
    
//     // Prediction
//     x_10 = f(x_0);
//     P_10 = A * P_0 * A_T + Q;

//     // Update
//     C = calculate_C(x_neg, x_0, dt);
//     C_T = ~C;   
//     K = P_10 * C_T * Inverse(C * P_10 * C_T + R);
//     y = get_y(bno);
//     x_11 = x_10 + K * (y - g(x_neg, x_0, dt));
//     BLA::Matrix<6, 1> test = y - g(x_neg, x_0, dt);
//     P_11 = P_10 - K * C * P_10;

//     // for t + 1
//     x_neg = x_0;
//     x_0 = x_11;
//     P_0 = P_11;

//     // debugging
//     Serial.print(">x: ");
//     Serial.println(x_11(0));


//     Serial.print(">y: ");
//     Serial.println(x_11(1));


//     Serial.print(">z: ");
//     Serial.println(x_11(2));


//     Serial.print(">dt: ");
//     Serial.println(dt);

//     startMillis = millis();
//   }
// }
