#pragma once
#include <Arduino.h>
#include "drivers/MCP3561.hpp"
#include "hardware-configs/BoardConfig.hpp"

class Scanner {

    protected:

    enum State : uint8_t {
        IDLE,
        WAIT_MUX,
        WAIT_CONV
    };

    

    public:

    struct Bank {
        MCP3561 &adc;
        MuxSettings adcIn;
        const uint8_t* muxPins;
        float* data;
        uint8_t numChannel;
        uint8_t irqPin;
    };
    
    virtual ~Scanner() = default;
    virtual void update() = 0;
    virtual void setup() = 0;

};