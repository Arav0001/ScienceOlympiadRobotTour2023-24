#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <SPI.h>
#include <util/atomic.h>
#include <StandardCplusplus.h>
#include <vector>

#define LEFT_MOTOR 3
#define RIGHT_MOTOR 2

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 4
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 5

#define WHEEL_DIAMETER 62.4f / 10.0f
#define TICKS_PER_REVOLUTION 300
#define TRACK_WIDTH 13.1f

long startTime = 0;

class PID {
private:
    float Kp;
    float Ki;
    float Kd;
    float Ka;

    long pt = 0;
    int pe = 0;
    
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    float cfe = 0.0f;
    float pfe = 0.0f;
public:
    PID(float Kp, float Ki, float Kd, float Ka) {
        setCoefficients(Kp, Ki, Kd, Ka);
    }

    PID() {
        setCoefficients(1.0f, 0.0f, 0.0f, 0.0f);
    }

    void setCoefficients(float Kp, float Ki, float Kd, float Ka) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->Ka = Ka;
    }

    float update(int current, int target, float scalar) {
        long t = micros();

        float dt  = ((float)(t - pt)) / 1.0e6f;
        pt = t;

        int error = target - current;

        cfe = (Ka * pfe) + (1 - Ka) * (error - pe); 
        pfe = cfe;

        pe = error;

        p = error;
        i = i + error * dt;
        d = cfe / dt;

        float signal = (Kp * p + Ki * i + Kd * d) / scalar;

        return constrain(signal, -1.0f, 1.0f);
    }
};

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

    int TPR = 0.0f;

    PID posPID;
    PID velPID;
public:
    DCMotor() = default;
    DCMotor(Adafruit_DCMotor *motor, int encA, int encB, int TPR) {
        this->motor = motor;
        this->encA_pin = encA;
        this->encB_pin = encB;

        this->TPR = TPR;
        
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

            vel = dp / (dt / 1.0e3f) / (float)TPR * 2.0f * M_PI;

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

class PositionProfile {
private:
    float distance;
    float ticks; // cm
    float vel;
    float totalTime;
    float flatTime;

    float startPos;
    float startTime;

    float WHEEL_RADIUS;
    int TPR;

    float b1;
    float b2;

    float quadratic(float t, float offset) {
        return vel * (1 - pow((t - offset) / b1 - 1, 2));
    }

    float trigonometric(float t, float offset) {
        return vel * ((t + offset) / 2 - b1 / (2 * M_PI) * sin(M_PI / b1 * (t - offset)));
    }

public:
    PositionProfile() = default;
    
    PositionProfile(float distance, float startPos, float startTime, float totalTime, float WHEEL_RADIUS, int TPR) {
        this->distance = distance;
        this->ticks = distance / WHEEL_RADIUS;

        this->totalTime = totalTime;
        this->flatTime = totalTime - 1.0f;

        this->startPos = startPos;
        this->startTime = startTime;

        this->WHEEL_RADIUS = WHEEL_RADIUS;
        this->TPR = TPR;

        b1 = (totalTime - flatTime) / 2.0f;
        b2 = (totalTime + flatTime) / 2.0f;
    }

    float getQuadraticPosition(float time) {
        if (distance == 0.0f) return 0.0f;
        
        const float t = time - startTime;
        
        const float I = (2 * totalTime + flatTime) / 3;
        vel = distance / WHEEL_RADIUS / I;
        
        if (0 <= t && t < b1) {
            return quadratic(t, 0.0f);
        } else if (b1 <= t && t < b2) {
            return vel;
        } else if (b2 <= t && t <= totalTime) {
            return quadratic(t, flatTime);
        } else {
            return 0.0f;
        }
    }

    float getTrigonometricPosition(float time) {
        if (distance == 0.0f) return startPos;
        
        const float t = time - startTime;

        const float I = (totalTime + flatTime) / 2;
        vel = distance / WHEEL_RADIUS / I;

        float dp;
        
        if (0 <= t && t < b1) {
            dp = trigonometric(t, 0.0f);
        } else if (b1 <= t && t < b2) {
            dp = vel * (t - b1 / 2.0f);
        } else if (b2 <= t && t <= totalTime) {
            dp = trigonometric(t, flatTime);
        } else {
            dp =  0.0f;
        }

        dp *= TPR / 2.0f / M_PI;

        return startPos + dp;
    }

    float getStartTime() {
        return startTime;
    }

    float getEndTime() {
        return startTime + totalTime;
    }
    
    float getStartPos() {
        return startPos;
    }
    
    float getDeltaPos() {
        return distance / (M_PI * WHEEL_RADIUS * 2.0f) * TPR; //
    }
    
    float getTotalTime() {
        return totalTime;
    }
};

class PositionProfileSequence {
private:
    std::vector<PositionProfile> profiles;
    
    float startPos;
    float startTime;

    float WAIT_TIME;
    float WHEEL_RADIUS;
    int TPR;
public:
    PositionProfileSequence() = default;

    PositionProfileSequence(float startPos, float startTime, float WAIT_TIME, float WHEEL_RADIUS, int TPR) {
        this->startPos = startPos;
        this->startTime = startTime;

        this->WAIT_TIME = WAIT_TIME;
        this->WHEEL_RADIUS = WHEEL_RADIUS;
        this->TPR = TPR;

        profiles = std::vector<PositionProfile>();
    }

    void addPositionProfile(float distance, float totalTime) {
        profiles.push_back(PositionProfile(distance, startPos + getTotalPos(), startTime + getTotalTime(), totalTime, WHEEL_RADIUS, TPR));
        if (WAIT_TIME > 0.0f) profiles.push_back(PositionProfile(0.0f, startPos + getTotalPos(), startTime + getTotalTime(), WAIT_TIME, WHEEL_RADIUS, TPR));
    }

    float getTotalPos() {
        float totalPos = 0.0f;
        for (PositionProfile profile : profiles) totalPos += profile.getDeltaPos();
        return totalPos;
    }
    
    float getTotalTime() {
        float totalTime = 0.0f;
        for (PositionProfile profile : profiles) totalTime += profile.getTotalTime();
        return totalTime;
    }

    int findProfile(float time) {
        int profile = 0;

        for (unsigned int i = 0; i < profiles.size(); i++) {
            if (profiles.at(i).getStartTime() <= time && time <= profiles.at(i).getEndTime()) {
                profile = i;
                break;
            }
        }

        return profile;
    }
    
    float getQuadraticPosition(float time) {
        return profiles.at(findProfile(time)).getQuadraticPosition(time);
    }
    
    float getTrigonometricPosition(float time) {
        if (time >= getTotalTime()) {
            PositionProfile lastProfile = profiles.at(profiles.size() - 1);
            return lastProfile.getStartPos() + lastProfile.getDeltaPos();
        }
        
        return profiles.at(findProfile(time)).getTrigonometricPosition(time);
    }
};

Adafruit_MotorShield shield = Adafruit_MotorShield();

DCMotor leftMotor(shield.getMotor(LEFT_MOTOR), LEFT_ENCODER_A, LEFT_ENCODER_B, TICKS_PER_REVOLUTION);
DCMotor rightMotor(shield.getMotor(RIGHT_MOTOR), RIGHT_ENCODER_A, RIGHT_ENCODER_B, TICKS_PER_REVOLUTION);

void updateLeftEncoder() { leftMotor.readEncoder(); }
void updateRightEncoder() { rightMotor.readEncoder(); }

PositionProfileSequence leftSequence;
PositionProfileSequence rightSequence;

int cmToTicks(float cm);
float ticksToCm(int ticks);

void update();
void outputTelemetry(float time, float leftPos, float rightPos);

void straight(float distance, float time);
void turnRight(float time);
void turnLeft(float time);
void pause(float time);

void setup() {
    Serial.begin(9600);
    
    shield.begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, HIGH);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, HIGH);

    leftMotor.setPosPIDcoefficients(3.5f, 0.05f, 0.17f, 0.005f);
    leftMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    rightMotor.setPosPIDcoefficients(3.5f, 0.05f, 0.17f, 0.005f);
    rightMotor.setVelPIDcoefficients(12.5f, 32.5f, 0.75f, 0.0f);

    startTime = millis() / 1000.0f;
    
    leftSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);
    rightSequence = PositionProfileSequence(0.0f, startTime, 0.1f, WHEEL_DIAMETER / 2.0f, TICKS_PER_REVOLUTION);

    straight(50.0f, 1.5f);
    turnLeft(1.0f);
}

void loop() {
    update();
}

void straight(float distance, float time) {
    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(-distance, time);
}

void turn(float radians, float time) {
    float distance = radians * TRACK_WIDTH / 2;

    leftSequence.addPositionProfile(distance, time);
    rightSequence.addPositionProfile(distance, time);
}

void turnLeft(float time) {
    turn(M_PI / 2.0f, time);
}

void turnRight(float time) {
    turn(-M_PI / 2.0f, time);
}

void pause(float time) {
    leftSequence.addPositionProfile(0.0f, time);
    rightSequence.addPositionProfile(0.0f, time);
}

void update() {
    delay(1);

    leftMotor.update();
    rightMotor.update();

    float time = millis() / 1000.0f;

    float leftPos = leftSequence.getTrigonometricPosition(time);
    float rightPos = rightSequence.getTrigonometricPosition(time);

    outputTelemetry(time, leftPos, rightPos);
    
    leftMotor.setPosition(leftPos);
    rightMotor.setPosition(rightPos);
}

void outputTelemetry(float time, float leftPos, float rightPos) {
    Serial.print((float)leftMotor.getPosition() / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print(leftPos / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print((float)rightMotor.getPosition() / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.print(rightPos / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * M_PI);
    Serial.print(" ");
    Serial.println(time - startTime);
}

int cmToTicks(float cm) {
    return cm / (M_PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
}

float ticksToCm(int ticks) {
    return (float)ticks / TICKS_PER_REVOLUTION * M_PI * WHEEL_DIAMETER;
}