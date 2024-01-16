#pragma once

#include <Arduino.h>

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