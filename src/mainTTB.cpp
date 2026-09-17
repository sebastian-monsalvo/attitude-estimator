#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.print(3);
    Serial.print(",");
    Serial.println(5);
    delay(100);
}