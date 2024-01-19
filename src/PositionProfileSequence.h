#pragma once

#include "PositionProfile.h"

#include <StandardCplusplus.h>
#include <vector>

class PositionProfileSequence {
private:
    std::vector<PositionProfile> profiles;
    
    float startPos;
    float startTime;

    float WAIT_TIME;
    float WHEEL_RADIUS;
    int TICKS_PER_REVOLUTION;
public:
    PositionProfileSequence() = default;

    PositionProfileSequence(float startPos, float startTime, float WAIT_TIME, float WHEEL_RADIUS, int TICKS_PER_REVOLUTION) {
        this->startPos = startPos;
        this->startTime = startTime;

        this->WAIT_TIME = WAIT_TIME;
        this->WHEEL_RADIUS = WHEEL_RADIUS;
        this->TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;

        profiles = std::vector<PositionProfile>();
    }

    void addPositionProfile(float distance, float totalTime) {
        profiles.push_back(PositionProfile(distance, startPos + getTotalPos(), startTime + getTotalTime(), totalTime, WHEEL_RADIUS, TICKS_PER_REVOLUTION));
        if (WAIT_TIME > 0.0f) profiles.push_back(PositionProfile(0.0f, startPos + getTotalPos(), startTime + getTotalTime(), WAIT_TIME, WHEEL_RADIUS, TICKS_PER_REVOLUTION));
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