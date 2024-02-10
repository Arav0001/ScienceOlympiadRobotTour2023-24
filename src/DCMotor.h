#pragma once

#include <Arduino.h>

#include "PID.h"

class DCMotor {
private:
    int dir1_pin;
    int dir2_pin;

    int pwm_pin;
    
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

    int TPR = 0.0f;

    PID posPID;
    PID velPID;
public:
    DCMotor() = default;
    DCMotor(int dir1, int dir2, int pwm, int encA, int encB, int TPR) {
        this->dir1_pin = dir1;
        this->dir2_pin = dir2;
        this->pwm_pin = pwm;

        this->encA_pin = encA;
        this->encB_pin = encB;

        pinMode(dir1_pin, OUTPUT);
        pinMode(dir2_pin, OUTPUT);
        pinMode(pwm_pin, OUTPUT);

        this->TPR = TPR;
        
        pinMode(encA_pin, INPUT);
        pinMode(encB_pin, INPUT);

        this->posPID = PID();

        setPower(0.0f);
    }

    void setPosPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        posPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void setVelPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        velPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void readEncoder() {
        if (digitalRead(encA_pin)) {
            -digitalRead(encB_pin) ? pos++ : pos--;
        }
    }
    
    void update() {
        t = millis();
        long dt = t - pt;
        int dp = pos - ppos;

        vel = dp / (dt / 1.0e3f) / (float)TPR * 2.0f * M_PI;

        fvel = alpha * vel + (1.0f - alpha) * fvel;

        pt = t;
        ppos = pos;
    }

    void setPower(float power) {
        if (power > 0.0f) {
            digitalWrite(dir1_pin, LOW);
            digitalWrite(dir2_pin, HIGH);
        } else if (power < 0.0f) {
            digitalWrite(dir1_pin, HIGH);
            digitalWrite(dir2_pin, LOW);
        } else {
            digitalWrite(dir1_pin, LOW);
            digitalWrite(dir2_pin, LOW);
        }
        
        analogWrite(pwm_pin, (int)abs(power * 255.0f));
        
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
        return pos;
    }

    void resetPosition() {
        pos = 0;
    }

    // rad/s
    float getAngularVelocity() {
        return fvel;
    }
};