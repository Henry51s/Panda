#pragma once
#include "ArmingController.hpp"

void ArmingController::setup() {
    pinMode(armPin, OUTPUT);
    pinMode(disarmPin, OUTPUT);

    activeLine = NONE;

    // Optional pulse to disarm
    digitalWrite(armPin, LOW);
    digitalWrite(disarmPin, LOW);

    active = false;
}

// void ArmingController::setState(State state_) {
//     state = state_;
//     timer = 0;
// }

void ArmingController::start(State s) {

    cancel();

    digitalWrite(armPin, LOW);
    digitalWrite(disarmPin, LOW);

    digitalWrite((s == ARM) ? armPin : disarmPin, HIGH);

    activeLine = (s == ARM) ? LINE_ARM : LINE_DISARM;

    timer = 0;
    active = true;
}

void ArmingController::cancel() {
    if (!active) {return;}

    switch(activeLine) {

        default: break;

        case LINE_ARM:    digitalWrite(armPin, LOW);    break;
        case LINE_DISARM: digitalWrite(disarmPin, LOW); break;

    }

    activeLine = NONE;
    active = false;

}

void ArmingController::update() {
    if (!active) {return;}
    if (timer > PULSE_DURATION) {
        active = false;
        digitalWrite((state == ARM) ? armPin : disarmPin, LOW);
    }
}

void ArmingController::arm() {
    start(ARM);
}

void ArmingController::disarm() {
    start(DISARM);
}