#include "BBController.hpp"

/*
TODO:
- Implement minimum on/off time logic using elapsedMillis
- Persistance/Hysteresis logic
*/
BBController::BBController(double lowerDeadband_, double upperDeadband_, double minOffTime_, double minOnTime_) {
    lowerDeadband = lowerDeadband_;
    upperDeadband = upperDeadband_;
    minOffTime = minOffTime_;
    minOnTime = minOnTime_;
}

void BBController::determineState(float measurement) {
    // if (desiredState == valveState) return;  // already in desired state

    // const uint32_t required_ms = valveState ? minOnTime : minOffTime;

    // if (timer >= required_ms) {
    //     valveState = desiredState;
    //     timer = 0;              // reset timer at each transition
    // }

    bool over = currentPV > (targetPV + upperDeadband);
    bool under = currentPV < (targetPV - lowerDeadband);

    if (over) {
        state = false;
    }
    else if (under) {
        state = true;
    }
}

bool BBController::getState() {return state;}

void BBController::setTargetPV(double targetPV_) {targetPV = targetPV_;}

void BBController::setLowerDeadband(double lowerDeadband_) {lowerDeadband = lowerDeadband_;}
void BBController::setUpperDeadband(double upperDeadband_) {upperDeadband = upperDeadband_;}

void BBController::setMinOffTime(double minOffTime_) {minOffTime = minOffTime_;}
void BBController::setMinOnTime(double minOnTime_) {minOnTime = minOnTime_;}


