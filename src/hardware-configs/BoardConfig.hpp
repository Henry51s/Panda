#pragma once
#include <SPI.h>
#include <Arduino.h>

/**
 * Configuration file
 * Includes DC channel and Bang-Bang implementation, time values, and conversion constants
 */

 // =================== SPI & ADC configurations ================= //

static const SPISettings SPISettingsDefault(20000000, MSBFIRST, SPI_MODE0);
static constexpr unsigned int T_MUX_SETTLE_US = 5000;
static constexpr unsigned int T_CONV_US = 10000;

// =================== Serial configurations ================= //

static constexpr bool DEBUG_F_ADC = false;
static constexpr bool DEBUG_BB = false;
static constexpr bool DEBUG_PACKET = false;

static constexpr unsigned int SERIAL_BAUD_RATE = 115200;
static constexpr unsigned int SERIAL_TIMEOUT = 2000;
static constexpr unsigned int SERIAL_WRITE_DELAY = 100000; // Microseconds to wait between writes to prevent overwhelming the serial bus

static constexpr unsigned int PACKET_IDLE_MS = 100;
static constexpr unsigned int PULSE_DURATION = 500;

static constexpr uint8_t NUM_MAX_COMMANDS = 32;
static constexpr uint8_t DATA_DECIMALS = 6; // Number of decimal places in telemetry data

static constexpr uint8_t NUM_DC_CHANNELS = 12;
static constexpr uint8_t NUM_PT_CHANNELS = 16;
static constexpr uint8_t NUM_LC_CHANNELS = 6;
static constexpr uint8_t NUM_TC_CHANNELS = 6;

static constexpr uint8_t PACKET_SIZE = NUM_DC_CHANNELS + NUM_PT_CHANNELS + NUM_LC_CHANNELS + NUM_TC_CHANNELS;

// =================== BangBangControllers configurations ================= //
/**
 * BangBang pressures will be hardcoded, deadbands and target pressures will be set in real time. Each PT and solenoid slot is hard-wired.
 * BB Channel 1: Fuel-side slow press (controllerFuel)
 * BB Channel 2: LOX-side slow press (controllerLOX)
 */

// static constexpr double BBPTShuntResistor[4] = {47.2, 47.1, 46.9, 47.1}; // Nominal value should be 47 ohms, actual values are 5% off

// // Manually declare bang-bang controllers here if needed
// BangBangController controllerFuel(0, 0, 0, 0);
// BangBangController controllerLOX(0, 0, 0, 0);

// // nullptr means no BangBangController associated with the DCChannel (BangBangControllers are hard wired per channel)
// static DCChannel dcChannels[NUM_DC_CHANNELS] = {
//   DCChannel(PINS_DC_CHANNELS[0], nullptr),
//   DCChannel(PINS_DC_CHANNELS[1], nullptr),
//   DCChannel(PINS_DC_CHANNELS[2], nullptr),
//   DCChannel(PINS_DC_CHANNELS[3], nullptr),
//   DCChannel(PINS_DC_CHANNELS[4], nullptr),
//   DCChannel(PINS_DC_CHANNELS[5], nullptr),
//   DCChannel(PINS_DC_CHANNELS[6], nullptr),
//   DCChannel(PINS_DC_CHANNELS[7], nullptr),
//   DCChannel(PINS_DC_CHANNELS[8], nullptr),
//   DCChannel(PINS_DC_CHANNELS[9], nullptr),
//   DCChannel(PINS_DC_CHANNELS[10], &controllerFuel),
//   DCChannel(PINS_DC_CHANNELS[11], &controllerLOX)
// };
