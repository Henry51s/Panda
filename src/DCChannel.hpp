#pragma once
#include <Arduino.h>

class DCChannel {
  
    private:

    int pin; // Assuming all DC channels are already initialized as digital output pins
    bool state = false;
    bool isControlled = false;

    public:

    DCChannel(int pin_);
    void setState(bool state_);
    bool getState();

    // Bang-Bang control integration
    void setControlled(bool controlled);
    bool isControlledByBangBang();
    
};