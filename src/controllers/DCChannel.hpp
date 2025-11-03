#pragma once
#include <Arduino.h>
#include "InterfaceController.hpp"

class DCChannel {
  
    private:

    enum Modes {
        AUTO = 0,
        MANUAL = 1
    };

    uint8_t pin; // Assuming all DC channel pins are already initialized as digital output pins
    bool state = false;
    bool hasBB = false;

    Modes mode = MANUAL;

    // BangBangController* controllerPtr = nullptr; // Pointer to the BangBangController if it exists, otherwise nullptr

    InterfaceController* ctrl;

    public:

    DCChannel(int pin_, InterfaceController* ctrl_, bool hasBB_); // Constructor with optional BangBangController pointer

    void setState(bool state_);
    void setMode(Modes mode_);

    Modes getMode(void);
    bool getState(void);

    void update(void);

    // Bang-Bang control integration
    // void setControlled(bool controlled);
    // void setControllerPtr(BangBangController* controllerPtr_) {controllerPtr = controllerPtr_;} // Sets the pointer to the BangBangController
    // BangBangController* getControllerPtr(void) {return controllerPtr;} // Returns the pointer to the BangBangController if it exists
    // bool isBangBangActive(void); // returns true if bang bang control active
    // bool hasBangBang(void); // returns true if the dc channel has a bang bang controller associated with it
    
};