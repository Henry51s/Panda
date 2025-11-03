#pragma once
// #include <Arduino.h>
#include "elapsedMillis.h"
#include "InterfaceController.hpp"

class BBController : public InterfaceController {
    private:

    double lowerDeadband = 0;
    double upperDeadband = 0;

    double minOffTime = 0;
    double minOnTime = 0;

    // bool isActive = false;
    double currentPV = 0, targetPV = 0; // Make sure units are consistent
    bool state = false;
    // bool inHys = false;

    // elapsedMillis timer = 0;

    public: 

    BBController(double lowerDeadband_, double upperDeadband_, double minOffTime_, double minOnTime_); // All controllers should be initialized after initializing all DC channels
 
    // void updateController(double currentPV_);
    void determineState(float measurement);
    bool getState();

    void setUpperDeadband(double upperDeadband_);
    void setLowerDeadband(double lowerDeadband_);

    void setMinOffTime(double minOffTime_);
    void setMinOnTime(double minOnTime_);

    void setTargetPV(double targetPV_);

};