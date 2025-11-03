#pragma once
#include <Wire.h>

// Driver header file for the MCP9802A0 temperature sensor
class MCP9802A0 {

    public:

    void configureTempSensor(void);
    float getBoardTemp(void);
    uint16_t temperature;

};