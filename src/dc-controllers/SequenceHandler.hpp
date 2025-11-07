#include <elapsedMillis.h>
#include "hardware-configs/BoardConfig.hpp"
#include "hardware-configs/pins.hpp"

struct SequenceItem {
    uint8_t channel;
    bool state;
    int delay;
};

struct DCChannel{
    uint8_t pin;
    bool state;

    DCChannel() : pin(0), state(false) {}
    DCChannel(uint8_t pin_) : pin(pin_) {}
    void setState(bool state_) {
        state = state_;
        digitalWrite(pin, state);
    }
};


class SequenceHandler {

    public:

    void setup();
    void update(void);
    void setCommand(char* command);
    void resetCommand(void);
    bool pollCommand(void);
    void execute(bool sequenceState);

    void printCurrentCommand(void);

    DCChannel channelArr[NUM_DC_CHANNELS];

    private:

    elapsedMillis sTimer;

    int currentIndex;
    int numCommands;

    bool isActive = false;
    bool inDelay = false;

    SequenceItem sequenceArr[NUM_MAX_COMMANDS];


    // int solenoidChannels[MAX_NUM_COMMANDS];
    // int solenoidStates[MAX_NUM_COMMANDS];
    // int solenoidDelays[MAX_NUM_COMMANDS];

};