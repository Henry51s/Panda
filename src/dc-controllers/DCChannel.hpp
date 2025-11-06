#pragma once
#include <Arduino.h>

class DCChannel {
  
    private:

    uint8_t pin; // Assuming all DC channel pins are already initialized as digital output pins
    bool state = false;

    public:

    DCChannel(int pin_); // Constructor with optional BangBangController pointer

    void setState(bool state_);
    bool getState(void);
};