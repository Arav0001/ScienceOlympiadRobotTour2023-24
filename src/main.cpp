#include <Arduino.h>
#include <Wire.h>

#include "DCMotor.h"
#include "PositionProfileSequence.h"

#define WHEEL_DIAMETER 63.0f / 10.0f
#define TICKS_PER_REVOLUTION 385
#define TRACK_WIDTH 12.875f

long startTime = 0;

DCMotor leftMotor(A4, A5, 6, 10, 9, TICKS_PER_REVOLUTION);
DCMotor rightMotor(A2, A3, 5, 11, 12, TICKS_PER_REVOLUTION);

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

    attachInterrupt(10, updateLeftEncoder, RISING);
    attachInterrupt(11, updateRightEncoder, RISING);

    leftMotor.setPosPIDcoefficients(2.5f, 0.0f, 0.15f, 0.005f);
    leftMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    rightMotor.setPosPIDcoefficients(2.5f, 0.0f, 0.15f, 0.005f);
    rightMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    startTime = millis() / 1000.0f;
    
    leftSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);
    rightSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);

    // add code here
}

void loop() {
    update();
}

void straight(float distance, float time) {
    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(distance, time);
}

void turn(float radians, float time) {
    float distance = radians * TRACK_WIDTH / 2;

    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(-distance, time);
}

void turnLeft(float time) {
    turn(-M_PI / 2.0f, time);
}

void turnRight(float time) {
    turn(M_PI / 2.0f, time);
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

    float leftPos = leftSequence.getPosition(time);
    float rightPos = rightSequence.getPosition(time);

    //outputTelemetry(time, leftPos, rightPos);
    
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