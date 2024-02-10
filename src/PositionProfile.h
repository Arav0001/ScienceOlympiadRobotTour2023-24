#pragma once

#include <Arduino.h>
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
    int TPR;

    float b1;
    float b2;

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

    float getPosition(float time) {
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