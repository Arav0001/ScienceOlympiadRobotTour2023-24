#include <Arduino.h>

class PID {
private:
    float Kp;
    float Ki;
    float Kd;
    float Ka;

    long previousTime = 0;
    int previousError = 0;
    
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    float currentFilterEstimate = 0.0f;
    float previousFilterEstimate = 0.0f;
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
        long currentTime = micros();

        float deltaTime  = ((float)(currentTime - previousTime)) / 1.0e6f;
        previousTime = currentTime;

        int error = target - current;

        currentFilterEstimate = (Ka * previousFilterEstimate) + (1 - Ka) * (error - previousError); 
        previousFilterEstimate = currentFilterEstimate;

        previousError = error;

        p = error;
        i = i + error * deltaTime;
        d = currentFilterEstimate / deltaTime;

        float correction = (Kp * p + Ki * i + Kd * d) / scalar;
        
        if (correction > 1.0f) correction = 1.0f;
        else if (correction < -1.0f) correction = -1.0f;

        return correction;
    }
};