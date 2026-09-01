#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long startMillis;
unsigned long currentMillis;


void setup() {
  Serial.begin(9600);
  bno.begin();
  bno.setExtCrystalUse(true);
  startMillis = millis();
}

void loop() {

  currentMillis = millis();

  if (currentMillis - startMillis > 1000) {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print(gyro.x());
    Serial.println("");
    float dt = (float) currentMillis - startMillis;


    startMillis = millis();
  }

}
