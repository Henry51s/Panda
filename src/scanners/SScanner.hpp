#if 0
#pragma once

#include "Scanner.hpp"
#include "hardware-configs/pins.hpp"

class SScanner : public Scanner {

    private:

    MCP3561 &adc;
    MuxSettings adcCh = MuxSettings::CH0;

    State state = IDLE;
    // uint8_t bank = 0;
    uint8_t channel = 0;
    elapsedMicros timer;

    public:

    float adcOutput[NUM_DC_CHANNELS] = {0};

    SScanner(MCP3561 &adc_) : adc(adc_) {}

    void setup() override;
    void update() override;


};

#endif