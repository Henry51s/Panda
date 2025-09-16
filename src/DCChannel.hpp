#pragma once
#include <Arduino.h>
#include <BangBangController.hpp>

class DCChannel {
  
    private:

    int pin; // Assuming all DC channels are already initialized as digital output pins
    bool state = false;
    bool isControlled = false;

    BangBangController* controllerPtr = nullptr; // Pointer to the BangBangController if it exists, otherwise nullptr

    public:

    DCChannel(int pin_, BangBangController* controllerPtr_); // Constructor with optional BangBangController pointer
    void setState(bool state_);
    bool getState(void);

    // Bang-Bang control integration
    // void setControlled(bool controlled);
    // void setControllerPtr(BangBangController* controllerPtr_) {controllerPtr = controllerPtr_;} // Sets the pointer to the BangBangController
    BangBangController* getControllerPtr(void) {return controllerPtr;} // Returns the pointer to the BangBangController if it exists
    bool isBangBangActive(void); // returns true if bang bang control active
    bool hasBangBang(void); // returns true if the dc channel has a bang bang controller associated with it
    
};