#include "VelocityProfile.h"

#include <StandardCplusplus.h>
#include <vector>

class VelocityProfileSequence {
private:
    const float WAIT_TIME = 3.0f;
    
    std::vector<VelocityProfile> profiles;
    
    float startTime;

    float WHEEL_RADIUS;
public:
    VelocityProfileSequence() = default;

    VelocityProfileSequence(float startTime, float WHEEL_RADIUS) {
        this->startTime = startTime;
        this->WHEEL_RADIUS = WHEEL_RADIUS;

        profiles = std::vector<VelocityProfile>();
    }

    void addVelocityProfile(float distance, float totalTime) {
        profiles.push_back(VelocityProfile(distance, startTime + getTotalTime(), totalTime, WHEEL_RADIUS));
        profiles.push_back(VelocityProfile(0.0f, startTime + getTotalTime(), WAIT_TIME, WHEEL_RADIUS));
    }

    float getTotalTime() {
        float totalTime = 0.0f;
        for (VelocityProfile profile : profiles) totalTime += profile.getTotalTime();
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
    
    float getQuadraticAngularVelocity(float time) {
        return profiles.at(findProfile(time)).getQuadraticAngularVelocity(time);
    }
    
    float getTrigonometricAngularVelocity(float time) {
        return profiles.at(findProfile(time)).getTrigonometricAngularVelocity(time);
    }
};