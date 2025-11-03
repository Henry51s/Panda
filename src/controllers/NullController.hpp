#pragma once
#include "InterfaceController.hpp"

class NullController : public InterfaceController {
    private:
    bool state = false;
    public:
    void determineState(float measurement) {state = false;}
};