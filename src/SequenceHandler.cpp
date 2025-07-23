#include "SequenceHandler.hpp"

void SequenceHandler::resetCommand() {
    memset(solenoidChannels, 0, sizeof(solenoidChannels));
    memset(solenoidStates,   0, sizeof(solenoidStates));
    memset(solenoidDelays,   0, sizeof(solenoidDelays));
}

void SequenceHandler::printCurrentCommand(void) {

    uint8_t dcChannels[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};

    for (int i = 0; i < numCommands; i++) {
        Serial.print("  ["); Serial.print(i); Serial.print("] ");
        Serial.print("chan=");  Serial.print(solenoidChannels[i]);
        Serial.print(", pin="); Serial.print(dcChannels[solenoidChannels[i] - 1]);
        Serial.print(", state=");  Serial.print(solenoidStates[i]);
        Serial.print(", delay=");  Serial.println(solenoidDelays[i]);
    } 
    
}

bool SequenceHandler::pollCommand(void) {
    if (numCommands < 0) {
        return true;
    }
    else {
        return false;
    }

}

void SequenceHandler::setCommand(char* command) {

    /*
    Command example: s11.01000,s10.00000,sA1.02000,sA0.00000
    Function: Channel 1 is on for one second, off's, then Channel 10 is on for 2 seconds, then off's

    **** Using milliseconds here. microseconds are cringe

    */

    numCommands    = 0;
    currentIndex   = 0;
    isActive       = false;
    sTimer         = 0;

    memset(solenoidChannels, 0, sizeof(solenoidChannels));
    memset(solenoidStates,   0, sizeof(solenoidStates));
    memset(solenoidDelays,   0, sizeof(solenoidDelays));

    char buf[256];
    strcpy(buf, command);
    buf[sizeof(buf) - 1] = '\0';

    size_t lenCommand = strlen(command);

    for (int i = 0; i < lenCommand; i++) {
        if (buf[i] == 's') {
            numCommands++;
        }
    }

    unsigned channel, state, delay;
    char* token = strtok(buf, ",");

    if (token && sscanf(token + 1, "%1x%1u.%5u", &channel, &state, &delay) == 3) {
        solenoidChannels[0] = channel;
        solenoidStates[0] = state;
        solenoidDelays[0] = delay;

        Serial.print("Token "); Serial.print("0"); Serial.print(": "); Serial.println(token);
        Serial.print("  ["); Serial.print("0"); Serial.print("] ");
        Serial.print("chan=");  Serial.print(channel);
        // Serial.print(", pin="); Serial.print(dcChannels[solenoidChannels[i] - 1]);
        Serial.print(", state=");  Serial.print(state);
        Serial.print(", duration=");  Serial.println(delay);
    }

    for (int i = 1; i < numCommands; i++) {
        token = strtok(nullptr, ",");
        Serial.print("Token "); Serial.print(i); Serial.print(": "); Serial.println(token);
        if (token && sscanf(token + 1, "%1x%1u.%5u", &channel, &state, &delay) == 3) {
            solenoidChannels[i] = channel;
            solenoidStates[i] = state;
            solenoidDelays[i] = delay;

            Serial.print("  ["); Serial.print(i); Serial.print("] ");
            Serial.print("chan=");  Serial.print(channel);
            // Serial.print(", pin="); Serial.print(dcChannels[solenoidChannels[i] - 1]);
            Serial.print(", state=");  Serial.print(state);
            Serial.print(", duration=");  Serial.println(delay);


        }
    }


    // int commandLength = strlen(command);
    // int commandArrayIndex = 0;
    // for (int i = 0; i < commandLength; i++) {
    // if (command[i] == 's') {
    //     if (command[i+1] >= '0' && command[i+1] <= '9') {
    //     solenoidChannels[commandArrayIndex] = command[i + 1] - '0';
    //     }
    //     if (command[i+1] >= 'A' && command[i+1] <= 'F') {
    //     solenoidChannels[commandArrayIndex] = command[i + 1] - 'A' + 10;
    //     }

    // numCommands++;

    //     solenoidStates[commandArrayIndex] = command[i + 2] - '0';
    //     //Serial.println(command[i + 2]);
    // }
    // if (command[i] == '.') {
    //     solenoidDelays[commandArrayIndex] = 10000 * (command[i+1] - '0') + 1000 * (command[i+2] - '0') + 100 * (command[i+3] - '0') + 10 * (command[i+4] - '0') + 1 * (command[i+5] - '0');
    //     //Serial.println(solenoidDelays[commandArrayIndex]);
    // }
    // if (command[i] == ',') {
    //     commandArrayIndex++;
    // }
    // }


}

void SequenceHandler::setState(bool sequenceState) {
    if (sequenceState == true) {
        isActive = true;
        // sTimer = solenoidDelays[0] + 1;
        sTimer = 0;
        currentIndex = 0;
    }
    else {
        isActive = false;
    }
}

// TODO: Solenoid delays is shifted right 1
void SequenceHandler::update() {

    uint8_t dcChannels[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};

    // No commands and not firing
    if (numCommands == 0 || isActive == false) {
        sTimer = 0;
        return;
    }

    if (inDelay == false) {
        digitalWrite(dcChannels[solenoidChannels[currentIndex] - 1], solenoidStates[currentIndex]);

        Serial.print("Firing idx=");
        Serial.print(currentIndex);
        Serial.print(" chan=");
        Serial.print(solenoidChannels[currentIndex]);
        Serial.print(" pin=");
        Serial.print(dcChannels[solenoidChannels[currentIndex] - 1]);
        Serial.print(" state=");
        Serial.println(solenoidStates[currentIndex]);
        Serial.print(" duration=");
        Serial.println(solenoidDelays[currentIndex]);

        sTimer = 0;
        inDelay = true;
    }

    else if (sTimer >= solenoidDelays[currentIndex]) {
        inDelay = false;
        currentIndex++;

        if (currentIndex >= numCommands) {
            currentIndex = 0;
            isActive = false;
            Serial.println("Sequence complete");
            resetCommand();
        }
    }



    // // Performing first action, then waiting first delay
    // if (currentIndex == 0 && isActive == true) {
    //     digitalWrite(dcChannels[solenoidChannels[currentIndex] - 1], solenoidStates[currentIndex]);
    //     sTimer = 0;
    //     currentIndex++;
    // }

    // if (currentIndex >= 1 && sTimer >= solenoidDelays[currentIndex] && isActive == true) {
    //     //
    // Serial.print("Firing idx=");
    // Serial.print(currentIndex);
    // Serial.print(" chan=");
    // Serial.print(solenoidChannels[currentIndex]);
    // Serial.print(" pin=");
    // Serial.print(dcChannels[solenoidChannels[currentIndex] - 1]);
    // Serial.print(" state=");
    // Serial.println(solenoidStates[currentIndex]);
    // Serial.print(" duration=");
    // Serial.println(solenoidDelays[currentIndex]);
    // digitalWrite(dcChannels[solenoidChannels[currentIndex]], solenoidStates[currentIndex]);
    //     currentIndex++;
    //     sTimer = 0;
    // }



    // if () {
    //     // debug print everything:
    //     Serial.print("Firing idx=");
    //     Serial.print(currentIndex);
    //     Serial.print(" chan=");
    //     Serial.print(solenoidChannels[currentIndex]);
    //     Serial.print(" pin=");
    //     Serial.print(dcChannels[solenoidChannels[currentIndex] - 1]);
    //     Serial.print(" state=");
    //     Serial.println(solenoidStates[currentIndex]);
    //     Serial.print(" duration=");
    //     Serial.println(solenoidDelays[currentIndex]);
    //     digitalWrite(dcChannels[solenoidChannels[currentIndex] - 1], solenoidStates[currentIndex]);
    //     // currentIndex++;
    //     sTimer = 0;
    //     return;
    // }
    
    // if (sTimer < solenoidDelays[currentIndex]) {
    //     currentIndex++;
    //     sTimer = 0;
    // }



}