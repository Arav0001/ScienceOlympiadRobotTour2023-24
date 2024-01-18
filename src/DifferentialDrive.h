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
    DCMotor* leftMotor;
    DCMotor* rightMotor;

    float WHEEL_DIAMETER = NULL;
    int TICKS_PER_REVOLUTION = NULL;
    float TRACK_WIDTH = NULL;
    float WHEEL_TO_CENTER = NULL;

    float MAX_ANGULAR_VELOCITY = 5.0;

    Pose2D pose;

public:
    DifferentialDrive(int leftMotor, int rightMotor, int leftEncA, int leftEncB, int rightEncA, int rightEncB) {
       this->shield = Adafruit_MotorShield();
       this->leftMotor = new DCMotor(shield.getMotor(leftMotor), leftEncA, leftEncB, TICKS_PER_REVOLUTION);
       this->rightMotor = new DCMotor(shield.getMotor(rightMotor), rightEncA, leftEncB, TICKS_PER_REVOLUTION);

       pose = Pose2D();
    }

    void setDriveConstants(float wheelDiameter, int ticksPerRevolution, float trackWidth) {
        this->WHEEL_DIAMETER = wheelDiameter;
        this->TICKS_PER_REVOLUTION = ticksPerRevolution;
        this->TRACK_WIDTH = trackWidth;
        this->WHEEL_TO_CENTER = TRACK_WIDTH / 2;
    }

    bool init() {
        return shield.begin() && WHEEL_DIAMETER != NULL && TICKS_PER_REVOLUTION != NULL && TRACK_WIDTH != NULL;
    }

    void drive(float leftSpeed, float rightSpeed, float time) {
        long start = micros();

        while((micros() - start) / 1e6 <= time) {
            leftMotor->setAngularVelocity(leftSpeed);
            rightMotor->setAngularVelocity(rightSpeed);
        }
        
        stop();
    }

    void stop() {
        leftMotor->setPower(0.0f);
        rightMotor->setPower(0.0f);
    }

    void straight(float d, float time) {
        const float v = d / (WHEEL_DIAMETER * M_PI) / time;

        if (abs(v) <= MAX_ANGULAR_VELOCITY) {
            drive(v, v, time);

            pose = pose + Pose2D(d * cos(pose.theta), d * sin(pose.theta), 0.0f);
        } else {
            Serial.println("ERROR: invalid time, velocity exceeds maximum permitted by motor");
        }
    }

    void turnOnPoint(float theta, float time) {
        const float dl = theta * WHEEL_TO_CENTER;
        const float dr = -theta * WHEEL_TO_CENTER;

        const float vl = dl / (WHEEL_DIAMETER * M_PI) / time;
        const float vr = dr / (WHEEL_DIAMETER * M_PI) / time;

        if (abs(vl) <= MAX_ANGULAR_VELOCITY && abs(vr) <= MAX_ANGULAR_VELOCITY) {
            drive(vl, vr, time);

            pose = pose + Pose2D(0.0f, 0.0f, theta);
        } else {
            Serial.println("ERROR: invalid time, velocity exceeds maximum permitted by motor");
        }
    }
    
    void turnOnArc(float theta, float d, float time) {
        const int sign = signbit(theta);
        theta = abs(theta);
        const float htheta = sign * theta / 2;

        const float radius = d / theta;
        const float dl = (radius - sign * WHEEL_TO_CENTER) * theta;
        const float dr = (radius + sign * WHEEL_TO_CENTER) * theta;

        const float vl = dl / (WHEEL_DIAMETER * M_PI) / time;
        const float vr = dr / (WHEEL_DIAMETER * M_PI) / time;

        if (abs(vl) <= MAX_ANGULAR_VELOCITY && abs(vr) <= MAX_ANGULAR_VELOCITY) {
            drive(vl, vr, time);

            const float dxy = 2 * radius * sin(htheta);
            const float dx = dxy * cos(pose.theta - htheta);
            const float dy = dxy * sin(pose.theta - htheta);

            pose = pose + Pose2D(theta, dx, dy);
        } else {
            Serial.println("ERROR: invalid time, velocity exceeds maximum permitted by motor");
        }
    }

    void delayAndUpdate(float seconds) {
        long start = micros();
        while((micros() - start) / 1e6 <= seconds) {}
    }
};