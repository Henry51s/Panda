#pragma once

#include <Arduino.h>
#include "hardware-configs/BoardConfig.hpp"

class ArmingController {



    private:

    enum State {
        DISARM = 0,
        ARM = 1
    };

    enum ActiveLine : uint8_t {
        NONE,
        LINE_ARM,
        LINE_DISARM
    };

    uint8_t armPin, disarmPin;
    elapsedMillis timer = 0;
    bool active = false;
    State state = DISARM;
    ActiveLine activeLine;

    void start(State s);
    void cancel();

    public:

    ArmingController(uint8_t armPin_, uint8_t disarmPin_) : armPin(armPin_), disarmPin(disarmPin_) {}

    void setup();
    // void setState(State state_);
    void arm();
    void disarm();

    void update();

};