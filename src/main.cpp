#include <FlexSerial.h>
#include <SPI.h>
#include "SequenceHandler.hpp"
#include <Wire.h>
#include "MCP3561.hpp" // ADC
#include "DCChannel.hpp"
#include "Constants.hpp"
// #include "BangBangController.hpp"

/* 
TODO:
- Replace while(Serial.available()) with non-blocking reads
- Clean up SPI and ADC initialization
- Implement Bang-Bang controllers and DCChannel object *WIP*
- Add channel state to data packet
- Implement telemetry on/off control

*/




// Cursed UART pin config
// FlexSerial UART1(UART1Pins.rx, UART1Pins.tx); // RX, TX
FlexSerial UART2(UART2Pins.rx, UART2Pins.tx); // RX, TX

MCP3561 sADC(sADCPins.cs, SPI);
MCP3561 ptADC(ptADCPins.cs, SPI1);

SequenceHandler sh;

bool toCSVRow(const float* data, char identifier, size_t n,
              char* out, size_t outSize,
              uint8_t decimals = DATA_DECIMALS) { // Generates a CSV row from an array of floats for serial transmission
  size_t used = 0;

  for (size_t i = 0; i < n ; ++i) {
    char digits[32];
    digits[0] = identifier;                                // scratch for one number
    dtostrf(data[i], 0, decimals, digits + 1);         // <-- write INTO digits
    size_t len = strlen(digits);

    // need = number chars + 1 delimiter (',' or '\n') + 1 final '\0' reserve
    size_t need = len + 1 + 1;
    if (used + need > outSize) return false;        // prevent overflow

    memcpy(out + used, digits, len);                // append number
    used += len;

    out[used++] = (i + 1 < n) ? ',' : '\n';         // delimiter
  }

  out[used] = '\0';                                  // null-terminate
  return true;
}

// ========== State machine ==========

enum State : uint8_t {
  IDLE,
  WAIT_MUX,
  WAIT_CONV
};

struct Bank {
  MCP3561 &adc;
  MuxSettings adcIn;
  const uint8_t* muxPins;
  float* data;
  uint8_t numChannel;
};

static float sData[NUM_DC_CHANNELS] = {0}, ptData[NUM_PT_CHANNELS] = {0}, lctcData[NUM_TC_CHANNELS + NUM_LC_CHANNELS] = {0};

/*
bank[0] - Solenoid channel current reading
bank[1] - PT channel reading
bank[2] - LC & TC channel reading (LC and TC data arrays are combined because they share the same mux)
*/
static constexpr Bank banks[] = {

  {sADC, MuxSettings::CH0, sMux, sData, NUM_DC_CHANNELS},
  {ptADC, MuxSettings::CH1, ptMux, ptData, NUM_PT_CHANNELS},
  {ptADC, MuxSettings::CH0, lctcMux, lctcData, NUM_TC_CHANNELS + NUM_LC_CHANNELS}

};

static constexpr uint8_t NUM_BANKS = sizeof(banks) / sizeof(banks[0]);

struct ADCMuxSystem {

  private:

  // State
  State state = IDLE;
  uint8_t bank = 0;
  uint8_t channel = 0;
  elapsedMicros timer;

  // Outputs
  // uint8_t mapping[NUM_DC_CHANNELS] = {7, 6, 5, 4, 3, 2, 1, 0, 11, 10, 9, 8}; // mapping[i] = m | i is the mux channel, m is the corresponding solenoid channel on the board

  public:
  // This should be called after all pins have been initialized in the regular setup()
  void setup() {
    for (auto b : banks) {
      for (int i = 0; i < 4; i++) {
        pinMode(b.muxPins[i], OUTPUT);
        digitalWrite(b.muxPins[i], LOW);
      }
    }
  }

  void update() {

    const Bank& currentBank = banks[bank];

    switch(state) {
      case IDLE:
        // Select mux channel and ADC input channel, reset timer and proceed to WAIT_MUX state
        currentBank.adc.setMuxInputs(currentBank.adcIn, MuxSettings::AGND);

        for (int i = 0; i < 4; i++) {
          digitalWrite(currentBank.muxPins[i], (channel >> i) & 1);
        }

        timer = 0;
        state = WAIT_MUX;
        break;
      case WAIT_MUX:
        // wait for T_MUX_SETTLE_US, then start one-shot reading from ADC
        // reset timer, then proceed to WAIT_CONV
        if (timer >= T_MUX_SETTLE_US) {
          // Tell the ADC to perform one-shot
          currentBank.adc.trigger();
          timer = 0;
          state = WAIT_CONV;
        }
        break;
      case WAIT_CONV:
        // wait for T_CONV_US, then read the ADC output into the respective samples index
        // Increment channel (If it's at 15, wrap back to zero, then we have a full sample array and we're good to process)
        // return to IDLE
        if (timer >= T_CONV_US) {
          // Read ADC output
          // Set samples[channel] to what the ADC outputs
          float res = currentBank.adc.getOutput();
          currentBank.data[channel] = res;

          // Advancing channel and/or bank
          channel++;
          if (channel >= currentBank.numChannel) {
            channel = 0;
            bank++;
            if (bank >= NUM_BANKS) {
              bank = 0;
            }
          }
          timer = 0;
          state = IDLE;
        }
        break;
    }
  }
};

// Handling non-blocking pulses for arming and safing
struct Pulse {

  private:
    uint8_t pin;
    elapsedMillis timer = 0;
    bool active = false;

  public:
    Pulse(uint8_t pin_): pin(pin_) {}

    void start() {
      digitalWrite(pin, HIGH);
      timer = 0;
      active = true;
    }

    void cancel() {
      if (active) {
        active = false;
        digitalWrite(pin, LOW);
      }
    }

    void update() {
      if (active && timer > PULSE_DURATION) {
        active = false;
        digitalWrite(pin, LOW);
      }
    }

};


ADCMuxSystem scanner;
Pulse armPulse(PIN_ARM);
Pulse disarmPulse(PIN_DISARM);

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(SERIAL_BAUD_RATE); //RS-485 bus 1
  Serial2.setTimeout(100);
  Serial.begin(SERIAL_BAUD_RATE); // Debugging via serial monitor
  Wire2.begin();
  // I2CScanner();

  pinMode(PIN_ARM, OUTPUT);
  pinMode(PIN_DISARM, OUTPUT);

  digitalWrite(PIN_DISARM, HIGH);
  delay(PULSE_DURATION);
  digitalWrite(PIN_DISARM, LOW);

  for (int i = 0; i < NUM_DC_CHANNELS; i++) {
    pinMode(PINS_DC_CHANNELS[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(sMux[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(ptMux[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    digitalWrite(ptMux[i], LOW);
  }

  pinMode(sADCPins.cs, OUTPUT);
  pinMode(sADCPins.irq, INPUT);

  pinMode(sADCPins.miso, INPUT);
  pinMode(sADCPins.mosi, OUTPUT);

  pinMode(ptADCPins.cs, OUTPUT);
  pinMode(ptADCPins.irq, INPUT);
 
  pinMode(ptADCPins.miso, INPUT);
  pinMode(ptADCPins.mosi, OUTPUT);


  SPI.begin();
  SPI.setClockDivider(4);
  
  SPI.setMISO(sADCPins.miso);
  SPI.setMOSI(sADCPins.mosi);
  SPI.setSCK(sADCPins.sck);

  sADC.setSettings(SPISettingsDefault);
  delay(100);
  sADC.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
  delay(100);
  sADC.writeRegisterDefaults();
  sADC.setGain(GainSettings::GAIN_1);
  sADC.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
  sADC.setVREF(3.3f);
  sADC.setBiasCurrent(BiasCurrentSettings::I_0);

  SPI1.begin();
  SPI1.setClockDivider(4);

  // SPI1.setMISO(SPI_PT_MISO);
  // SPI1.setMOSI(SPI_PT_MOSI);
  SPI1.setMISO(ptADCPins.miso);
  SPI1.setMOSI(ptADCPins.mosi);
  // SPI.setCS(CS_PIN);
  // SPI1.setSCK(SPI_PT_SCK);
  SPI1.setSCK(ptADCPins.sck);

  ptADC.setSettings(SPISettingsDefault);
  delay(100);
  ptADC.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
  delay(100);
  ptADC.writeRegisterDefaults();
  delay(100);
  ptADC.setGain(GainSettings::GAIN_1);
  ptADC.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
  ptADC.setVREF(1.25f);

  scanner.setup();

}

void loop() {
  // put your main code here, to run repeatedly:

  // ========== Serial IO/Solenoid Sequence and command handling ==========

  /*
  Single-letter commands:
  'a' - Arm
  'r' - Safe
  'f' - Fire active sequence

  Solenoid command: S<Channel identifier in hex><State (1 or 0)> 
  [Example: S11 turns channel 1 on]
  Solenoid sequence: s<Channel identifier in hex><State>.<Delay after in milliseconds (5 digits)>,... 
  [Example: s11.01000,s10.00000 turns channel 1 on for 1000ms, then turns channel 1 off]

  "S" = Command
  "s" = Sequence
  */

  static char rxBuffer[128] = {0}; // Keep buffer between calls
  static uint8_t readIndex = 0;    // Keep track of how full

  static elapsedMillis lastByteTimer = 0;
  
  // static bool collecting = true;
  static bool packetReady = false;

  while (Serial2.available() > 0) {
      char c = Serial2.read();
      lastByteTimer = 0;

      if (c == '\r' || c == '\n') {
        if (readIndex > 0) {
          rxBuffer[readIndex] = '\0';
          packetReady = true;
        }
        readIndex = 0;
        break;
      }

      if (readIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[readIndex++] = c;
      }

      else {

        rxBuffer[sizeof(rxBuffer) - 1] = '\0';
        packetReady = true;
        readIndex = 0;
        break;

      }
      
  }

  if (!packetReady && readIndex > 0 && lastByteTimer > PACKET_IDLE_MS) {
        rxBuffer[readIndex] = '\0';
        packetReady = true;
        readIndex = 0;
  }

  if (packetReady) {
    // Serial.println(rxBuffer);

    char idChar = rxBuffer[0];
    size_t rxBufferLen = strlen(rxBuffer);
  

    if (idChar == 's') {

      sh.setCommand(rxBuffer);

    }

    else if (idChar == 'S') {

      char channelChar = rxBuffer[1];
      char stateChar = rxBuffer[2];

      unsigned channel, state;

      if (channelChar >= '0' && channelChar <= '9') channel = channelChar - '0';
      else channel = 10 + (toupper(channelChar) - 'A');     // A-F

      state = stateChar - '0';

      // digitalWrite(dcChannels[channel - 1], state);
      if (channel >= 1 && channel <= NUM_DC_CHANNELS) {
        dcChannels[channel - 1].setState(state);
      }



    }

    else if (idChar == 'a') {
      // Arm
      disarmPulse.cancel();
      armPulse.start();

    }

    else if (idChar == 'r') {
      // Disarm
      armPulse.cancel();
      disarmPulse.start();
    }

    else if (idChar == 'f') {
      sh.setState(true);
    }

    memset(rxBuffer, 0, sizeof(rxBuffer));
    packetReady = false;
  }


  armPulse.update();
  disarmPulse.update();
  sh.update();
  // UART1.println("Hello World!");

  // =========== Data Acquisition ===========

  scanner.update();
  
  Bank sBank = banks[0];
  Bank ptBank = banks[1];
  Bank lctcBank = banks[2];

  // =========== Bang-Bang Control ==========

 /*
 Manually update each BangBangController here. For example,
 testController.update(<Some PT from ptBank>);

 ** These PT channels and DC Channels are hard-coded. DO NOT SWITCH ANY BANG BANG PT/VALVE CONNECTORS ON THE BOARD **
 */

  // // Update all BangBangControllers
  // for (int i = 0; i < NUM_DC_CHANNELS; i++) {
  //   BangBangController* controllerPtr = dcChannels[i].getControllerPtr();
  //   if (controllerPtr != nullptr) {
  //     controllerPtr->update();
  //     bool newState = controllerPtr->getState();
  //     dcChannels[i].setState(newState);
  //   }
  // }

  // =========== Packet ==========

  /*

  Format: <Identifier letter><Data>,...

  Solenoid packet: s<Solenoid Current Data>
  PT packet: p<PT Current Data>
  TC and LC packet: t<TC and LC Voltage Data>

  TC and LC data is combined into one packet; the first 6 entries are LC, the last 6 are TC
  Packet length depends on decimal places defined in DATA_DECIMALS (Currently 5). All entries have the same decimal place. Each packet is sent separately
  
  */

  char sPacket[512], ptPacket[512], lctcPacket[512];
  if (toCSVRow(sBank.data,'s', NUM_DC_CHANNELS, sPacket, sizeof(sPacket), DATA_DECIMALS)) {
    // Serial2.println(sPacket);
    // Serial.println(sPacket);
  }
  if (toCSVRow(ptBank.data,'p', NUM_PT_CHANNELS, ptPacket, sizeof(ptPacket), DATA_DECIMALS)) {
    // Serial2.println(ptPacket);
    // Serial.println(ptPacket); 
  }
  if (toCSVRow(lctcBank.data,'t', NUM_LC_CHANNELS + NUM_TC_CHANNELS, lctcPacket, sizeof(lctcPacket), DATA_DECIMALS)) {
    //  Serial2.println(lctcPacket);
    //  Serial.println(lctcPacket);
  }
}

// void I2CScanner() {
//   for (uint8_t i = 0; i < 127; i++) {
//     Wire2.beginTransmission(i);
//     if (Wire2.endTransmission() == 0) {
//       Serial.print("Device at Address: 0x");
//       Serial.print(i, HEX);
//       Serial.print("\n");
//     }
//   }
// }



