#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <util/atomic.h>
#include <math.h>
#include <SPI.h>
#include "PID.h"
#include "DCMotor.h"
#include <Adafruit_MotorShield.h>


class Drive{
private:
    Adafruit_MotorShield shield;
    DCMotor leftMotor; 
    DCMotor rightMotor; 
    const int ticksPerRevolution = 600;
    const float inchesPerRevolution = 8.0395f;
    const float cmPerRevolution = 20.420f;
    const float robotWidthInch = 100.0f;
    const float robotWidthCm = 100.0f;
public:
    Drive(int leftMotorNumber, int rightMotorNumber, int encoderA1, int encoderB1, int encoderA2, int encoderB2){
       shield = Adafruit_MotorShield();
       leftMotor = DCMotor(shield.getMotor(leftMotorNumber), encoderA1, encoderB1);
       rightMotor = DCMotor(shield.getMotor(rightMotorNumber), encoderA2, encoderB2);
    }
    // unit 1 is inch anything else is cm
    void moveForward(float distance, int unit){
        float numRevolutions = 0.0f;
        if(unit == 1){
            numRevolutions = distance / inchesPerRevolution;
        } else {
            numRevolutions = distance / cmPerRevolution;
        }
        rightMotor.runToPos(numRevolutions * ticksPerRevolution);
        leftMotor.runToPos(numRevolutions * ticksPerRevolution);
        rightMotor.resetEncoderPos();
        leftMotor.resetEncoderPos();
    }
    void moveBackward(float distance, int unit){
        float numRevolutions = 0.0f;
        if(unit == 1){
            numRevolutions = distance / inchesPerRevolution;
        } else {
            numRevolutions = distance / cmPerRevolution;
        }
        rightMotor.runToPos(-numRevolutions * ticksPerRevolution);
        leftMotor.runToPos(-numRevolutions * ticksPerRevolution);
        rightMotor.resetEncoderPos();
        leftMotor.resetEncoderPos();
    }
    void turnLeft(int unit){
        float numRevolutions = 0.0f;
        if(unit == 1){
            numRevolutions = robotWidthInch / (2 * inchesPerRevolution);
        } else {
            numRevolutions = robotWidthCm / (2 * cmPerRevolution);
        }
        rightMotor.runToPos(numRevolutions * ticksPerRevolution);
        leftMotor.runToPos(-numRevolutions * ticksPerRevolution);
        rightMotor.resetEncoderPos();
        leftMotor.resetEncoderPos();
    }
    void turnRight(int unit){
        float numRevolutions = 0.0f;
        if(unit == 1){
            numRevolutions = robotWidthInch / (2 * inchesPerRevolution);
        } else {
            numRevolutions = robotWidthCm / (2 * cmPerRevolution);
        }
        rightMotor.runToPos(-numRevolutions * ticksPerRevolution);
        leftMotor.runToPos(numRevolutions * ticksPerRevolution);
        rightMotor.resetEncoderPos();
        leftMotor.resetEncoderPos();
    }
    void turnAround(int unit){
        turnLeft(unit);
        turnLeft(unit);
    }
    void moveRight(float distance, int unit){
        turnRight(unit);
        moveForward(distance, unit);
    }
    void moveLeft(float distance, int unit){
        turnLeft(unit);
        moveForward(distance, unit);
    }
};