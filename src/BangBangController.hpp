#pragma once
// #include <Arduino.h>
#include "elapsedMillis.h"

struct BangBangConfig {
    const double lowerDeadband = 0; // Make sure units are consistent
    const double upperDeadband = 0; // Make sure units are consistent
    // const double minOnTime = 0;
    // const double minOffTime = 0;
    const double overHysMs = 10; // Time to remain on after exceeding upper deadband
    const double underHysMs = 10; // Time to remain off after going below lower deadband
};

class BangBangController {
    private:

    const BangBangConfig config;

    bool isActive = false;
    double currentPressure = 0, targetPressure = 0; // Make sure units are consistent
    bool valveState = false;
    // bool inHys = false;

    elapsedMillis timer = 0;

    public: 

    BangBangController(const BangBangConfig& config_); // All controllers should be initialized after initializing all DC channels

    void setState(bool state);
    bool getState(void);
    
    void setTargetPressure(double target);
    void setCurrentPressure(double current);

    void update(void);

};