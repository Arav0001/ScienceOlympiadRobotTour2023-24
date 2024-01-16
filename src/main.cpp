#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "DCMotor.h"
#include "DifferentialDrive.h"
#include <SPI.h>

#include <string.h>

#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 4
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 5

#define WHEEL_DIAMETER 65.0f / 10.0f / 2.54f
#define TICKS_PER_REVOLUTION 300
#define TRACK_WIDTH 6.0f

DifferentialDrive drive(LEFT_MOTOR, RIGHT_MOTOR, LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

void setup() {
    Serial.begin(9600);

    drive.setDriveConstants(WHEEL_DIAMETER, TICKS_PER_REVOLUTION, TRACK_WIDTH);

    drive.init();

    drive.turnOnArc(1.2, 10, 8);
}

void loop() {
    // drive.stop();
}