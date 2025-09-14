#include "DCChannel.hpp"

DCChannel::DCChannel(int pin_): pin(pin_) {}

void DCChannel::setState(bool state_) {
    if (!isControlled) { // Only allow state change if not controlled by BangBang
        state = state_;
        digitalWrite(pin, state ? HIGH : LOW);
    }
}

void DCChannel::setControlled(bool controlled) {isControlled = controlled;}
bool DCChannel::isControlledByBangBang() {return isControlled;}
bool DCChannel::getState() {return state;}