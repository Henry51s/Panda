#pragma once

class InterfaceController {
    private:
    bool state = false;
    public:
    virtual ~InterfaceController() = default;  
    virtual bool getState() = 0;
    virtual void determineState(float measurement) = 0;
};