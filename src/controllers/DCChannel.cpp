#include "DCChannel.hpp"

DCChannel::DCChannel(int pin_, InterfaceController* ctrl_, bool hasBB_): pin(pin_), ctrl(ctrl_), hasBB(hasBB_) {}

void DCChannel::setState(bool state_) {state = state_;}
void DCChannel::setMode(Modes mode_) {mode = mode_;}

bool DCChannel::getState(void) {return state;}
DCChannel::Modes DCChannel::getMode(void) {return mode;}
// bool DCChannel::hasBangBang(void) {return hasBB;}

void DCChannel::update(){
    if (mode == MANUAL) {

        digitalWrite(pin, state ? HIGH : LOW);

    }
    else if (mode == AUTO && hasBB) {
        // Get state from controller, then digitalwrite....
        bool state = ctrl->getState();
    }
}