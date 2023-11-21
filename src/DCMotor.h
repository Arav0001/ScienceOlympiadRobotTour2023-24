#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <util/atomic.h>
#include <math.h>

#include "PID.h"

class DCMotor {
private:
    Adafruit_DCMotor *motor;
    int encoderA_pin;
    int encoderB_pin;
    
    float power = 0.0f;

    volatile int pos = 0;

    static DCMotor *instance;

    static void isr() {
        instance->readEncoder();
    }

    void readEncoder() {
        digitalRead(encoderA_pin) && !digitalRead(encoderB_pin)? pos++ : pos--;
    }

    PID pid;
    bool usingPID = false;
public:
    DCMotor(Adafruit_DCMotor *motor, int encoderA, int encoderB) {
        this->motor = motor;
        this->encoderA_pin = encoderA;
        this->encoderB_pin = encoderB;

        pinMode(encoderA_pin, INPUT_PULLUP);
        pinMode(encoderB_pin, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(encoderA_pin), isr, RISING);

        instance = this;

        this->pid = PID();
    }

    void setPIDcoefficients(float Kp, float Ki, float Kd, float Ka) {
        pid.setCoefficients(Kp, Ki, Kd, Ka);
        usingPID = true;
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

    void runToPos(float target) {
        int currentPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) currentPos = pos;

        if (usingPID) setPower(pid.update(currentPos, target, 255.0f));
    }

    float getPower() {
        return power;
    }

    void resetEncoderPos() {
        pos = 0;
    }
    
    int getEncoderPos() {
        int currentPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) currentPos = pos;
        return currentPos;
    }
};

DCMotor *DCMotor::instance;