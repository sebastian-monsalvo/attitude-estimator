#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "helpers.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long startMillis;
unsigned long currentMillis;
float total_roll;
float total_pitch;
float total_yaw;

void setup() {
  Serial.begin(9600);
  bno.begin();
  bno.setExtCrystalUse(true);
  startMillis = millis();
  total_roll = 0;
  total_pitch = 0;
  total_yaw = 0;

}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - startMillis >= 100) {
    startMillis = roll_pitch_yaw(startMillis, currentMillis, bno, total_roll, total_pitch, total_yaw);
  }

}
