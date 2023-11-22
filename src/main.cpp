#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "DCMotor.h"
#include <SPI.h>

#include <string.h>

Adafruit_MotorShield shield = Adafruit_MotorShield();

DCMotor leftMotor(shield.getMotor(1), 2, 4);

void setup() {
    Serial.begin(9600);

    shield.begin();
    leftMotor.setPIDcoefficients(8.0f, 0.1f, 0.5f, 0.005f);
}

void loop() {
    //leftMotor.setAngularVelocity(1000);

    Serial.println(leftMotor.getAngularVelocity());
}