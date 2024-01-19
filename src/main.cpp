#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "DCMotor.h"
#include "PositionProfileSequence.h"
#include <Wire.h>
#include <SPI.h>

#include <string.h>

#define LEFT_MOTOR 3
#define RIGHT_MOTOR 2

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 4
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 5

#define WHEEL_DIAMETER 62.4f / 10.0f
#define TICKS_PER_REVOLUTION 300
#define TRACK_WIDTH 13.0f

long startTime = 0;

Adafruit_MotorShield shield = Adafruit_MotorShield();

DCMotor leftMotor(shield.getMotor(LEFT_MOTOR), LEFT_ENCODER_A, LEFT_ENCODER_B, TICKS_PER_REVOLUTION);
DCMotor rightMotor(shield.getMotor(RIGHT_MOTOR), RIGHT_ENCODER_A, RIGHT_ENCODER_B, TICKS_PER_REVOLUTION);

void updateLeftEncoder() { leftMotor.readEncoder(); }
void updateRightEncoder() { rightMotor.readEncoder(); }

PositionProfileSequence leftSequence;
PositionProfileSequence rightSequence;

int cmToTicks(float cm);
float ticksToCm(int ticks);

void update();
void outputTelemetry(float time, float leftPos, float rightPos);

void straight(float distance, float time);
void turnRight(float time);
void turnLeft(float time);
void pause(float time);

void setup() {
    Serial.begin(9600);

    shield.begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, HIGH);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, HIGH);

    leftMotor.setPosPIDcoefficients(3.5f, 0.05f, 0.17f, 0.005f);
    leftMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    rightMotor.setPosPIDcoefficients(3.5f, 0.05f, 0.17f, 0.005f);
    rightMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    startTime = millis() / 1000.0f;
    
    leftSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);
    rightSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);

    straight(100.0f, 4.0f);
    pause(1.5f);
    turnLeft(0.5f);
}

void loop() {
    update();
}

void straight(float distance, float time) {
    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(-distance, time);
}

void turnLeft(float time) {
    float distance = M_PI / 4.0f * WHEEL_DIAMETER;

    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(distance, time);
}

void turnRight(float time) {
    float distance = M_PI / 4.0f * TRACK_WIDTH;

    leftSequence.addPositionProfile(-distance, time);
    rightSequence.addPositionProfile(-distance, time);
}

void pause(float time) {
    leftSequence.addPositionProfile(0.0f, time);
    rightSequence.addPositionProfile(0.0f, time);
}

void update() {
    delay(1);

    leftMotor.update();
    rightMotor.update();

    float time = millis() / 1000.0f;

    float leftPos = leftSequence.getTrigonometricPosition(time);
    float rightPos = rightSequence.getTrigonometricPosition(time);

    outputTelemetry(time, leftPos, rightPos);
    
    leftMotor.setPosition(leftPos);
    rightMotor.setPosition(rightPos);
}

void outputTelemetry(float time, float leftPos, float rightPos) {
    Serial.print((float)leftMotor.getPosition() / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print(leftPos / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print((float)rightMotor.getPosition() / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print(rightPos / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.println(time - startTime);
}

int cmToTicks(float cm) {
    return cm / (M_PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
}

float ticksToCm(int ticks) {
    return (float)ticks / TICKS_PER_REVOLUTION * M_PI * WHEEL_DIAMETER;
}