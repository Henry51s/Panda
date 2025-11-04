#include "DCChannel.hpp"

DCChannel::DCChannel(int pin_) : pin(pin_) {}

void DCChannel::setState(bool state_) {state = state_;}

bool DCChannel::getState(void) {return state;}