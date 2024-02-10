#pragma once

#include <Arduino.h>
#include <math.h>
#include <vector>

#include "PositionProfile.h"

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
    
    float getPosition(float time) {
        if (time >= getTotalTime()) {
            PositionProfile lastProfile = profiles.at(profiles.size() - 1);
            return lastProfile.getStartPos() + lastProfile.getDeltaPos();
        }
        
        return profiles.at(findProfile(time)).getPosition(time);
    }
};