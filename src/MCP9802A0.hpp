#pragma once
#include <Wire.h>
class MCP9802A0 {

    public:

    void configureTempSensor(void);
    float getBoardTemp(void);
    uint16_t temperature;


};