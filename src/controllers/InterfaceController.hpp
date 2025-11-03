#pragma once

class InterfaceController {
    public:

    virtual ~InterfaceController() = default;  
    virtual bool step(float measurement) = 0;
};