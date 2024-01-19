#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <util/atomic.h>
#include <math.h>

#include "PID.h"

class DCMotor {
private:
    Adafruit_DCMotor *motor;
    int encA_pin;
    int encB_pin;
    
    float power = 0.0f;

    volatile int pos = 0;
    volatile int ppos = 0;

    volatile float vel = 0.0f;
    const float alpha = 0.075f;
    volatile float fvel = 0.0f;

    volatile long t = 0;
    volatile long pt = 0;

    int TICKS_PER_REVOLUTION = 0.0f;

    PID posPID;
    PID velPID;
public:
    DCMotor() = default;
    DCMotor(Adafruit_DCMotor *motor, int encA, int encB, int TPR) {
        this->motor = motor;
        this->encA_pin = encA;
        this->encB_pin = encB;

        this->TICKS_PER_REVOLUTION = TPR;
        
        pinMode(encA_pin, INPUT_PULLUP);
        pinMode(encB_pin, INPUT_PULLUP);

        this->posPID = PID();
    }

    void setPosPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        posPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void setVelPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        velPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void readEncoder() {
        if (digitalRead(encA_pin)) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) !digitalRead(encB_pin) ? pos++ : pos--;
        }
    }
    
    void update() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            t = millis();
            long dt = t - pt;
            int dp = pos - ppos;

            vel = dp / (dt / 1.0e3f) / (float)TICKS_PER_REVOLUTION * 2.0f * M_PI;

            fvel = alpha * vel + (1.0f - alpha) * fvel;

            pt = t;
            ppos = pos;
        }
    }

    void setPower(float power) {
        int dir;

        if (power > 0.0f) {
            dir = FORWARD;
        } else if (power < 0.0f) {
            dir = BACKWARD;
        } else {
            dir = RELEASE;
        }

        this->motor->run(dir);
        this->motor->setSpeed((int)abs(power * 255.0f));
        
        this->power = power;
    }

    void setPosition(float target) {
        setPower(posPID.update(getPosition(), target, 255.0f));
    }

    // rad/s
    void setAngularVelocity(float velocity) {
        setPower(velPID.update(getAngularVelocity(), velocity, 255.0f));
    }

    float getPower() {
        return power;
    }

    int getPosition() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) return pos;
    }

    void resetPosition() {
        pos = 0;
    }

    // rad/s
    float getAngularVelocity() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) return fvel;
        return NULL;
    }
};