#include "BangBangController.hpp"

/*
TODO:
- Implement minimum on/off time logic using elapsedMillis
- Consider hysteresis handling if needed
*/
BangBangController::BangBangController(DCChannel& channel_, const BangBangConfig& config_)
    : config(config_), channel(channel_) {} // All controllers should be initialized after initializing all DC channels

void BangBangController::update() {

    bool overPressure = currentPressure > (targetPressure + config.upperDeadband);
    bool underPressure = currentPressure < (targetPressure - config.lowerDeadband);

    // Based SpaceX logic
    if (!isActive) {
        return;
    }
    if (overPressure) {
        channel.setState(false);
    }
    else if (underPressure) {
        channel.setState(true);
    }

    // else valveState = true;

}

void BangBangController::setState(bool state) {
    isActive = state;
    channel.setControlled(state);
}

void BangBangController::setCurrentPressure(double current) {currentPressure = current;}
void BangBangController::setTargetPressure(double target) {targetPressure = target;}

