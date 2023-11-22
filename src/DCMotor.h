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
    volatile float aVel = 0.0f;
    volatile long pt = 0;

    static DCMotor *instance;

    static void isr() {
        instance->readEncoder();
    }

    void readEncoder() {
        bool a = digitalRead(encA_pin);
        
        a && !digitalRead(encB_pin) ? pos++ : pos--;

        if (a) {
            long t = micros();
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) aVel = 2.0f * PI / ((t - pt) / 1.0e6f);

            pt = t;
        }
    }

    PID posPID;
    PID velPID;
public:
    DCMotor() = default;
    DCMotor(Adafruit_DCMotor *motor, int encA, int encB) {
        this->motor = motor;
        this->encA_pin = encA;
        this->encB_pin = encB;

        pinMode(encA_pin, INPUT_PULLUP);
        pinMode(encB_pin, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(encA_pin), isr, RISING);

        instance = this;

        this->posPID = PID();
    }

    void setPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        posPID.setCoefficients(Kp, Ki, Kd, Ka);
        velPID.setCoefficients(Kp, Ki, Kd, Ka);
    }

    void setPower(float power) {
        int dir;

        if (power > 0.0f) {
            dir = FORWARD;
        } else if (power < 0.0f) {
            dir = BACKWARD;
        } else {
            dir = BRAKE;
        }

        this->motor->run(dir);
        this->motor->setSpeed((int)abs(power * 255.0f));
        
        this->power = power;
    }

    void setPosition(float target) {
        setPower(posPID.update(getPosition(), target, 255.0f));
    }

    void setAngularVelocity(float speed) {
        setPower(velPID.update(getAngularVelocity(), speed, 1.0f));
    }

    float getPower() {
        return power;
    }

    float getAngularVelocity() {
        float cVel;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) cVel = aVel;
        return cVel;
    }

    void resetPosition() {
        pos = 0;
    }
    
    int getPosition() {
        int currentPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) currentPos = pos;
        return currentPos;
    }
};

DCMotor *DCMotor::instance;