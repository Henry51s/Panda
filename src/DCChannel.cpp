#include "DCChannel.hpp"

DCChannel::DCChannel(int pin_, BangBangController* controllerPtr_): pin(pin_), controllerPtr(controllerPtr_) {}

void DCChannel::setState(bool state_) {
    if (!isControlled) { // Only allow state change if not controlled by BangBang
        state = state_;
        digitalWrite(pin, state ? HIGH : LOW);
    }
}

// void DCChannel::setControlled(bool controlled) {isControlled = controlled;}
bool DCChannel::isBangBangActive(void) {return isControlled;}
bool DCChannel::getState(void) {return state;}
bool DCChannel::hasBangBang(void) {return controllerPtr != nullptr;}