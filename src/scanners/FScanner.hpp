#pragma once
#include "Scanner.hpp"
#include "hardware-configs/pins.hpp"

class FScanner : public Scanner {

    private:

    struct ChannelSettings {
        MuxSettings adcChannel;
        const uint8_t* muxPins;
        const uint8_t numChannels;
        float* out;
    };

    MCP3561 &adc;
    State state = IDLE;
    uint8_t channel = 0;
    uint8_t index = 0;
    elapsedMicros timer;
    


    ChannelSettings settingsArr[2] = {lctcSettings, ptSettings};
    uint8_t numSettings = 2;

    public:

    float ptOutput[NUM_PT_CHANNELS] = {0}, lctcOutput[NUM_TC_CHANNELS + NUM_LC_CHANNELS] = {0};

    ChannelSettings ptSettings = {
        MuxSettings::CH1,
        ptMux,
        NUM_PT_CHANNELS,
        ptOutput
    };

    ChannelSettings lctcSettings = {
        MuxSettings::CH0,
        lctcMux,
        NUM_LC_CHANNELS + NUM_TC_CHANNELS,
        lctcOutput
    };

    FScanner(MCP3561 &adc_) : adc(adc_) {}
    void setup() override;
    void update() override;

    void getPTOutput(float* out);
    void getLCTCOutput(float* out);

};