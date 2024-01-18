#include <math.h>

class VelocityProfile {
private:
    float distance; // cm
    float vel;
    float totalTime;
    float flatTime;

    float startTime;

    float WHEEL_RADIUS;

    float b1;
    float b2;

    float quadratic(float t, float offset) {
        return vel * (1 - pow((t - offset) / b1 - 1, 2));
    }

    float trigonometric(float t, float offset) {
        return vel * pow(sin(M_PI / (totalTime - flatTime) * (t - offset)), 2);
    }

public:
    VelocityProfile() = default;
    
    VelocityProfile(float distance, float startTime, float totalTime, float WHEEL_RADIUS) {
        this->distance = distance;
        this->totalTime = totalTime;
        this->flatTime = totalTime - 1.0f;

        this->startTime = startTime;

        this->WHEEL_RADIUS = WHEEL_RADIUS;

        b1 = (totalTime - flatTime) / 2.0f;
        b2 = (totalTime + flatTime) / 2.0f;
    }

    float getQuadraticAngularVelocity(float time) {
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

    float getTrigonometricAngularVelocity(float time) {
        if (distance == 0.0f) return 0.0f;
        
        const float t = time - startTime;

        const float I = (totalTime + flatTime) / 2;
        vel = distance / WHEEL_RADIUS / I;

        if (0 <= t && t < b1) {
            return trigonometric(t, 0.0f);
        } else if (b1 <= t && t < b2) {
            return vel;
        } else if (b2 <= t && t <= totalTime) {
            return trigonometric(t, flatTime);
        } else {
            return 0.0f;
        }
    }

    float getStartTime() {
        return startTime;
    }

    float getEndTime() {
        return startTime + totalTime;
    }
    
    float getTotalTime() {
        return totalTime;
    }
};