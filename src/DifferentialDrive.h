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

    const float WHEEL_DIAMETER = 65.0f / 10.0f / 2.54f;
    const int TICKS_PER_REVOLUTION = 300;
    const float TRACK_WIDTH = 0.0f;

    Pose2D pose;

public:
    DifferentialDrive(int leftMotor, int rightMotor, int leftEncA, int leftEncB, int rightEncA, int rightEncB) {
       this->shield = Adafruit_MotorShield();
       this->leftMotor = DCMotor(shield.getMotor(leftMotor), leftEncA, leftEncB);
       this->rightMotor = DCMotor(shield.getMotor(rightMotor), rightEncA, leftEncB);

       pose = Pose2D();

       shield.begin();
    }

    
};