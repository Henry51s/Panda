#include "BangBangController.hpp"

/*
TODO:
- Implement minimum on/off time logic using elapsedMillis
- Persistance/Hysteresis logic
*/
BangBangController::BangBangController(const BangBangConfig& config_)
    : config(config_) {} // All controllers should be initialized after initializing all DC channels

void BangBangController::update() {

    bool overPressure = currentPressure > (targetPressure + config.upperDeadband);
    bool underPressure = currentPressure < (targetPressure - config.lowerDeadband);
    bool insideDeadband = !overPressure && !underPressure;

    // Based SpaceX logic
    if (!isActive) {
        return;
    }
    if (overPressure) {
        valveState = false;
    }
    else if (underPressure) {
       valveState = true;
    }


}

void BangBangController::setState(bool state) {isActive = state;}
bool BangBangController::getState(void) {return valveState;}

void BangBangController::setCurrentPressure(double current) {currentPressure = current;}
void BangBangController::setTargetPressure(double target) {targetPressure = target;}

