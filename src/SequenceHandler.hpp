#include <elapsedMillis.h>

#define MAX_NUM_COMMANDS 32

class SequenceHandler {

    public:

    int solenoidChannels[MAX_NUM_COMMANDS];
    int solenoidStates[MAX_NUM_COMMANDS];
    int solenoidDelays[MAX_NUM_COMMANDS];

    void update(void);
    void setCommand(char* command);
    void resetCommand(void);
    bool pollCommand(void);
    void setState(bool sequenceState);

    void printCurrentCommand(void);

    private:

    elapsedMillis sTimer;
    int currentIndex;

    int numCommands;

    private:

    bool isActive = false;
    bool inDelay = false;

};