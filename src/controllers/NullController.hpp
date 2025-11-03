#pragma once
#include "InterfaceController.hpp"

class NullController : public InterfaceController {

    public:

    bool step(float measurement) override {return false;}


};