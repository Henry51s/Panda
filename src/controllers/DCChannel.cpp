#include "DCChannel.hpp"

DCChannel::DCChannel(int pin_, InterfaceController* ctrl_, bool hasBB_): pin(pin_), ctrl(ctrl_), hasBB(hasBB_) {}

void DCChannel::setState(bool state_) {
    digitalWrite(pin, state_ ? HIGH : LOW);
}

bool DCChannel::getState(void) {return state;}
bool DCChannel::hasBangBang(void) {return hasBB;}