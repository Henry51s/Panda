#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "DCChannel.hpp"

// =================== Pins ================= //
struct ADCPins {
  uint8_t irq, cs, mosi, miso, sck;
};

struct UARTPins {
  uint8_t rx, tx;
};

static constexpr UARTPins UART1Pins = {8, 7};
static constexpr UARTPins UART2Pins = {20, 21};

// Arming pins
static constexpr uint8_t PIN_ARM = 32;
static constexpr uint8_t PIN_DISARM = 33;

// Solenoid current ADC SPI pins
static constexpr ADCPins sADCPins = {9, 10, 11, 12, 13};

// PT ADC SPI pins
static constexpr ADCPins ptADCPins = {2, 0, 26, 1, 27};

static constexpr uint8_t ptMux[4] = {31, 30, 29, 28}; // Mux pins for PTs
static constexpr uint8_t lctcMux[4] = {3, 4, 6, 5}; // Mux pins for both LCs and TCs
static constexpr uint8_t sMux[4] = {18, 19, 23, 22}; // Solenoid current mux pins

// DC Channel pins. In order of channels 1 to 12
static constexpr uint8_t PINS_DC_CHANNELS[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};

// =================== SPI & ADC configurations ================= //

static const SPISettings SPISettingsDefault(20000000, MSBFIRST, SPI_MODE0);
static constexpr unsigned int T_MUX_SETTLE_US = 500;
static constexpr unsigned int T_CONV_US = 1000;

// =================== Serial configurations ================= //

static constexpr unsigned int SERIAL_BAUD_RATE = 460800;
static constexpr unsigned int SERIAL_TIMEOUT = 2000;

static constexpr unsigned int PACKET_IDLE_MS = 100;
static constexpr unsigned int PULSE_DURATION = 500;

static constexpr uint8_t NUM_MAX_COMMANDS = 32;
static constexpr uint8_t DATA_DECIMALS = 4;

static constexpr uint8_t NUM_DC_CHANNELS = 12;
static constexpr uint8_t NUM_PT_CHANNELS = 16;
static constexpr uint8_t NUM_LC_CHANNELS = 6;
static constexpr uint8_t NUM_TC_CHANNELS = 6;

static constexpr uint8_t PACKET_SIZE = NUM_DC_CHANNELS + NUM_PT_CHANNELS + NUM_LC_CHANNELS + NUM_TC_CHANNELS;

// =================== BangBangControllers configurations ================= //

// Manually declare bang-bang controllers here if needed
BangBangController testController({.lowerDeadband = 2.0, .upperDeadband = 3.0, .minOnTime = 500, .minOffTime = 500});

// nullptr means no BangBangController associated with the DCChannel (BangBangControllers are hard wired per channel)
static DCChannel dcChannels[NUM_DC_CHANNELS] = {
  DCChannel(PINS_DC_CHANNELS[0], nullptr),
  DCChannel(PINS_DC_CHANNELS[1], nullptr),
  DCChannel(PINS_DC_CHANNELS[2], nullptr),
  DCChannel(PINS_DC_CHANNELS[3], nullptr),
  DCChannel(PINS_DC_CHANNELS[4], nullptr),
  DCChannel(PINS_DC_CHANNELS[5], nullptr),
  DCChannel(PINS_DC_CHANNELS[6], nullptr),
  DCChannel(PINS_DC_CHANNELS[7], nullptr),
  DCChannel(PINS_DC_CHANNELS[8], nullptr),
  DCChannel(PINS_DC_CHANNELS[9], nullptr),
  DCChannel(PINS_DC_CHANNELS[10], nullptr),
  DCChannel(PINS_DC_CHANNELS[11], nullptr)
};
