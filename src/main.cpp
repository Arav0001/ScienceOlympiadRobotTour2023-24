#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "DCMotor.h"
#include <SPI.h>

Adafruit_MotorShield shield = Adafruit_MotorShield();

DCMotor leftMotor = DCMotor(shield.getMotor(1), 2, 4);
const float totalDistanceInch = 100; // Inch
const float totalDistanceCm = 100; // Cm
const float targetTime = 60; // Sec
void setup() {
    Serial.begin(9600);

    shield.begin();
    leftMotor.setPIDcoefficients(8.0f, 0.1f, 0.5f, 0.005f);
}

void loop() {
    leftMotor.runToPos(sin(millis() / 100.0f) * 100.0f);

    Serial.println(leftMotor.getEncoderPos());
}