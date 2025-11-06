#pragma once
#include "controllers/DCChannel.hpp"

class BBChannel : public DCChannel {
    
    private:

    double lowerDeadband = 0;
    double upperDeadband = 0;

    double minOffTime = 0;
    double minOnTime = 0;

    // bool isActive = false;
    double currentPV = 0, targetPV = 0; // Make sure units are consistent
    bool state = false;

};