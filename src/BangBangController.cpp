#include "BangBangController.hpp"

/*
TODO:
- Implement minimum on/off time logic using elapsedMillis
- Persistance/Hysteresis logic
*/
BangBangController::BangBangController(double lowerDeadband_, double upperDeadband_, double minOffTime_, double minOnTime_) {
    lowerDeadband = lowerDeadband_;
    upperDeadband = upperDeadband_;
    minOffTime = minOffTime_;
    minOnTime = minOnTime_;
}

void BangBangController::updateController(double currentPV ) {

    bool over = currentPV > (targetPV + upperDeadband);
    bool under = currentPV < (targetPV - lowerDeadband);
    bool insideDeadband = !over && !under;

    // Based SpaceX logic
    if (!isActive) {
        return;
    }
    if (over) {
        requestState(false);
    }
    else if (under) {
       requestState(true);
    }


}

void BangBangController::setState(bool state) {isActive = state;}
bool BangBangController::getState(void) {return valveState;}

void BangBangController::setTargetPV(double targetPV_) {targetPV = targetPV_;}

void BangBangController::setLowerDeadband(double lowerDeadband_) {lowerDeadband = lowerDeadband_;}
void BangBangController::setUpperDeadband(double upperDeadband_) {upperDeadband = upperDeadband_;}

void BangBangController::requestState(bool desiredState) {
    if (desiredState == valveState) return;  // already in desired state

    const uint32_t required_ms = valveState ? minOnTime : minOffTime;
    if (timer >= required_ms) {
        valveState = desiredState;
        timer = 0;              // reset timer at each transition
    }
}

void BangBangController::setMinOffTime(double minOffTime_) {minOffTime = minOffTime_;}
void BangBangController::setMinOnTime(double minOnTime_) {minOnTime = minOnTime_;}


