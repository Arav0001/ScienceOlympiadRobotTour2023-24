#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "DCMotor.h"
#include "VelocityProfileSequence.h"
#include <SPI.h>

#include <string.h>

#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 4
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 5

#define WHEEL_DIAMETER 65.0f / 10.0f
#define TICKS_PER_REVOLUTION 300
#define TRACK_WIDTH 0.0f

long startTime = 0;

Adafruit_MotorShield shield = Adafruit_MotorShield();

DCMotor leftMotor(shield.getMotor(LEFT_MOTOR), LEFT_ENCODER_A, LEFT_ENCODER_B, TICKS_PER_REVOLUTION);

VelocityProfileSequence* vpLeft;

int cmToTicks(float cm);
float ticksToCm(int ticks);

void setup() {
    Serial.begin(9600);

    shield.begin();
    leftMotor.setPosPIDcoefficients(3.5f, 0.05f, 0.17f, 0.005f);
    leftMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    startTime = millis() / 1000.0f;

    vpLeft = new VelocityProfileSequence(startTime, WHEEL_DIAMETER / 2.0f);

    vpLeft->addVelocityProfile(100.0f, 3.0f);
    vpLeft->addVelocityProfile(-25.0f, 1.0f);
}

void loop() {
    delay(1);
    
    leftMotor.update();

    float time = millis() / 1000.0f;
    
    float vel = vpLeft->getTrigonometricAngularVelocity(time);
    
    Serial.print(leftMotor.getAngularVelocity());
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.println(time - startTime);
    
    leftMotor.setAngularVelocity(vel);
}

int cmToTicks(float cm) {
    return cm / (M_PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
}

float ticksToCm(int ticks) {
    return (float)ticks / TICKS_PER_REVOLUTION * M_PI * WHEEL_DIAMETER;
}