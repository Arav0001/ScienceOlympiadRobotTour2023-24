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
    float direction = 1.0f;

    volatile int pos = 0;
    volatile float aVel = 0.0f;
    volatile long pt = 0;

    int TICKS_PER_REVOLUTION = 0.0f;

    static DCMotor *instance;

    static void isr() {
        instance->readEncoder();
    }

    void readEncoder() {
        if (digitalRead(encA_pin)) {
            !digitalRead(encB_pin) ? pos++ : pos--;
            
            long t = micros();
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) aVel = 2.0f * PI * 1.0e6f / (t - pt) / (float)TICKS_PER_REVOLUTION;

            pt = t;
        }
    }

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

        attachInterrupt(digitalPinToInterrupt(encA_pin), isr, RISING);

        instance = this;

        this->posPID = PID();
    }

    void reverse() {
        direction = -direction;
    }

    void setPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        posPID.setCoefficients(Kp, Ki, Kd, Ka);
        velPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void setPower(float power) {
        int dir;
        float _power = power * direction;

        if (_power > 0.0f) {
            dir = FORWARD;
        } else if (_power < 0.0f) {
            dir = BACKWARD;
        } else {
            dir = BRAKE;
        }

        this->motor->run(dir);
        this->motor->setSpeed((int)abs(_power * 255.0f));
        
        this->power = power;
    }

    void setPosition(float target) {
        setPower(posPID.update(getPosition(), target, 255.0f));
    }

    // rad/s
    void setAngularVelocity(float velocity) {
        setPower(velPID.update(getAngularVelocity(), velocity, 1.0f));
    }

    float getPower() {
        return power;
    }
    
    int getPosition() {
        int currentPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) currentPos = pos;
        return currentPos;
    }

    void resetPosition() {
        pos = 0;
    }

    // rad/s
    float getAngularVelocity() {
        float cVel;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) cVel = aVel;
        return cVel;
    }
};

DCMotor *DCMotor::instance;