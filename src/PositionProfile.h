#pragma once

#include <math.h>

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
    int TICKS_PER_REVOLUTION;

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
    
    PositionProfile(float distance, float startPos, float startTime, float totalTime, float WHEEL_RADIUS, int TICKS_PER_REVOLUTION) {
        this->distance = distance;
        this->ticks = distance / WHEEL_RADIUS;

        this->totalTime = totalTime;
        this->flatTime = totalTime - 1.0f;

        this->startPos = startPos;
        this->startTime = startTime;

        this->WHEEL_RADIUS = WHEEL_RADIUS;
        this->TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;

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

        dp *= TICKS_PER_REVOLUTION / 2.0f / M_PI;

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
        return distance / (M_PI * WHEEL_RADIUS * 2.0f) * TICKS_PER_REVOLUTION; //
    }
    
    float getTotalTime() {
        return totalTime;
    }
};