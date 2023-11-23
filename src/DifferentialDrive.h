#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include <util/atomic.h>

#include "DCMotor.h"
#include "PID.h"
#include "Pose2D.h"

class DifferentialDrive {
private:
    Adafruit_MotorShield shield;
    DCMotor leftMotor;
    DCMotor rightMotor;

    float WHEEL_DIAMETER;
    int TICKS_PER_REVOLUTION;
    float TRACK_WIDTH;

    Pose2D pose;

public:
    DifferentialDrive(int leftMotor, int rightMotor, int leftEncA, int leftEncB, int rightEncA, int rightEncB) {
       this->shield = Adafruit_MotorShield();
       this->leftMotor = DCMotor(shield.getMotor(leftMotor), leftEncA, leftEncB, TICKS_PER_REVOLUTION);
       this->rightMotor = DCMotor(shield.getMotor(rightMotor), rightEncA, leftEncB, TICKS_PER_REVOLUTION);

       pose = Pose2D();
    }

    void setDriveConstants(float wheelDiameter, int ticksPerRevolution, float trackWidth) {
        this->WHEEL_DIAMETER = wheelDiameter;
        this->TICKS_PER_REVOLUTION = ticksPerRevolution;
        this->TRACK_WIDTH = trackWidth;
    }

    bool init() {
        return shield.begin() && WHEEL_DIAMETER != NULL && TICKS_PER_REVOLUTION != NULL && TRACK_WIDTH != NULL;
    }

    void drive(float leftSpeed, float rightSpeed) {
        // rad/s
        leftMotor.setAngularVelocity(leftSpeed);
        rightMotor.setAngularVelocity(rightSpeed);
    }
};