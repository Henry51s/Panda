#pragma once
#include "DCChannel.hpp"

struct BangBangConfig {
    const double lowerDeadband = 0; // Make sure units are consistent
    const double upperDeadband = 0; // Make sure units are consistent
    const double minOnTime = 0;
    const double minOffTime = 0;
};

class BangBangController {
    private:

    const BangBangConfig config;
    DCChannel& channel; // Assuming all controllers should be initialized after initializing all DC channels

    bool isActive = false;
    double currentPressure = 0, targetPressure = 0; // Make sure units are consistent

    public: 

    BangBangController(DCChannel& channel_, const BangBangConfig& config_); // All controllers should be initialized after initializing all DC channels

    void setState(bool state);
    void setTargetPressure(double target);
    void setCurrentPressure(double current);

    void update();

};