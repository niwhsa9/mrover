#include "motion_profile.hpp"

double TrapezoidalMotionProfile::velocity(double t) {
    // TODO
    double totalDistance = (mDesiredPosition - mInitialPosition);

    double accelerationTime = mMaxVelocity / mMaxAcceleration;
    double totalTime = accelerationTime + totalDistance / mMaxVelocity;

    return 0.0;
}