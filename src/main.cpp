#include <FlexSerial.h>
#include <SPI.h>
#include "SequenceHandler.hpp"
#include <Wire.h>
#include "MCP3561.hpp" // ADC
// #include "Mux.hpp"

#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 2000

#define MAX_NUM_COMMANDS 32

// Arming pins
#define PIN_ARM 32
#define PIN_DISARM 33

// Solenoid current ADC SPI pins
#define SPI_S_IRQ 9
#define SPI_S_CS 10
#define SPI_S_MOSI 11
#define SPI_S_MISO 12
#define SPI_S_SCK 13

// PT ADC SPI pins
#define SPI_PT_IRQ 2
#define SPI_PT_CS 0
#define SPI_PT_MOSI 26
#define SPI_PT_MISO 1
#define SPI_PT_SCK 27

#define R_CS_S 0.1f // Solenoid current sense resistance
#define R_CS_PT 47.f // PT current sense resistance

static constexpr uint8_t NUM_DC_CHANNELS = 12;
static constexpr uint8_t NUM_PT_CHANNELS = 16;
static constexpr uint8_t NUM_LC_CHANNELS = 6;
static constexpr uint8_t NUM_TC_CHANNELS = 6;

static constexpr int T_MUX_SETTLE_US = 100000;
static constexpr int T_CONV_US = 1000000;

static constexpr bool SERIAL_DEBUG_S = true;
static constexpr bool SERIAL_DEBUG_F = true;

static constexpr uint8_t NUM_BANKS = 2;
static constexpr uint8_t BANK_PT = 0;
static constexpr uint8_t BANK_TC_LC = 1;


const uint8_t ptMux[4] = {31, 30, 29, 28};
const uint8_t lctcMux[4] = {3, 4, 6, 5};

// DC Channel pins. In order of channels 1 to 12
const uint8_t dcChannels[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};
// Solenoid current mux pins
const uint8_t sMux[4] = {18, 19, 23, 22};

// Cursed UART pin config
FlexSerial UART1(8, 7);

// Ability to turn on and off telemetry
bool sendState = false;


MCP3561 sADC(SPI_S_CS, SPI);
MCP3561 ptADC(SPI_PT_CS, SPI1);

SequenceHandler sh;

enum State : uint8_t {
  IDLE,
  WAIT_MUX,
  WAIT_CONV
};

// Separate structs because the F (fluids) ADC has two muxes connected to it
struct ADCMuxSystem_S {

  private:

  void selectChannel(uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(muxPins[i], (ch >> i) & 0x1);
    }
  }

  // Pins
  uint8_t muxPins[4];
  MCP3561 &adc;

  // State
  State state = IDLE;
  uint8_t channel = 0;
  elapsedMicros timer;

  // Outputs
  uint8_t mapping[NUM_DC_CHANNELS] = {7, 6, 5, 4, 3, 2, 1, 0, 11, 10, 9, 8}; // mapping[i] = m | i is the mux channel, m is the corresponding solenoid channel on the board

  public:

  float currentArr[NUM_DC_CHANNELS] = {0};

  ADCMuxSystem_S(const uint8_t pins[4], MCP3561 &mcp3561) : adc(mcp3561) {
    for (int i = 0; i < 4; i++) {
      muxPins[i] = pins[i];
    }
  }

  // This should be called after all pins have been initialized in the regular setup()
  void setup() {
    for (uint8_t i = 0; i < 4; i++) {
      pinMode(muxPins[i], OUTPUT);
      digitalWrite(muxPins[i], LOW);
    }
  }

  void update() {

    switch(state) {
      case IDLE:
        // Select mux channel, reset timer and proceed to WAIT_MUX state
        Serial.println("==========");
        Serial.print("Mux channel now set to: "); // Debugging serial messages
        Serial.println(channel);

        selectChannel(channel);
        timer = 0;
        state = WAIT_MUX;
        break;
      case WAIT_MUX:
        // wait for T_MUX_SETTLE_US, then start one-shot reading from ADC
        // reset timer, then proceed to WAIT_CONV
        if (timer >= T_MUX_SETTLE_US) {
          Serial.print("Sending one-shot command. Current time since IDLE: ");
          Serial.println(timer);
          // Tell the ADC to perform one-shot
          adc.trigger();
          timer = 0;
          state = WAIT_CONV;
        }
        break;
      case WAIT_CONV:
        // wait for T_CONV_US, then read the ADC output into the respective samples index
        // Increment channel (If it's at 15, wrap back to zero, then we have a full sample array and we're good to process)
        // return to IDLE
        if (timer >= T_CONV_US) {
          Serial.print("Reading from ADC. Current time since WAIT_MUX: ");
          Serial.println(timer);
          // Read ADC output
          // Set samples[channel] to what the ADC outputs
          float res = adc.getOutput();
          Serial.print("ADC output: ");
          Serial.println(res, 5);
          if (channel < NUM_DC_CHANNELS - 1 && channel >= 0) {channel++;}
          else {channel = 0;}
          timer = 0;
          state = IDLE;
        }
        break;
    }
  }
};


struct ADCMuxSystem_F {

  private:

  void selectChannel(uint8_t ch, const uint8_t muxPins[4]) {
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(muxPins[i], (ch >> i) & 0x1);
    }
  }

  // pointers into the above static arrays:
  const uint8_t numChArr[NUM_BANKS] = {
    NUM_PT_CHANNELS,
    NUM_TC_CHANNELS + NUM_LC_CHANNELS
  };

  float ptArr[NUM_PT_CHANNELS] = {0}, lctcArr[NUM_LC_CHANNELS + NUM_TC_CHANNELS] = {0};

  float* dataArr[NUM_BANKS]  = {ptArr, tcArr};

  uint8_t bank = 0;
  MCP3561 &adc;

  // State
  State state = IDLE;
  uint8_t channel = 0;
  elapsedMicros timer;

  // Outputs
  // uint8_t mapping[NUM_PT_CHANNELS] = {7, 6, 5, 4, 3, 2, 1, 0, 11, 10, 9, 8}; // mapping[i] = m | i is the mux channel, m is the corresponding solenoid channel on the board

  public:

  float ptArr[NUM_PT_CHANNELS] = {0};
  float lcArr[NUM_LC_CHANNELS] = {0};
  float tcArr[NUM_TC_CHANNELS] = {0};

  ADCMuxSystem_F(MCP3561 &mcp3561) : adc(mcp3561) {

    // Inititalize mux pins for fluids

  }

  // This should be called after all pins have been initialized in the regular setup()
  void setup() {
    for (uint8_t i = 0; i < 4; i++) {
      pinMode(mux1[i], OUTPUT);
      pinMode(mux2[i], OUTPUT);
      digitalWrite(mux1[i], LOW);
      digitalWrite(mux2[i], LOW);
    }
  }

  void update() {

    switch(state) {
      case IDLE:
        // Select mux channel, reset timer and proceed to WAIT_MUX state
        if (SERIAL_DEBUG_F) {
          Serial.println("==========");
          Serial.print("Mux channel now set to: "); // Debugging serial messages
          Serial.println(channel);
        }
      
        selectChannel(channel);
        timer = 0;
        state = WAIT_MUX;
        break;
      case WAIT_MUX:
        // wait for T_MUX_SETTLE_US, then start one-shot reading from ADC
        // reset timer, then proceed to WAIT_CONV
        if (timer >= T_MUX_SETTLE_US) {
          if (SERIAL_DEBUG_F) {
            Serial.print("Sending one-shot command. Current time since IDLE: ");
            Serial.println(timer);
          }
          
          // Tell the ADC to perform one-shot
          adc.trigger();
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
          float res = adc.getOutput();
          if (SERIAL_DEBUG_F) {
            Serial.print("Reading from ADC. Current time since WAIT_MUX: ");
            Serial.println(timer);  
            Serial.print("ADC output: ");
            Serial.println(res, 5);
          }
          if (channel < NUM_DC_CHANNELS - 1 && channel >= 0) {channel++;}
          else {channel = 0;}
          timer = 0;
          state = IDLE;
        }
        break;
    }
  }
};

ADCMuxSystem_S sSystem(sMux, sADC);
// ADCMuxSystem ptSystem(ptMux, ptADC);

void setup() {
  // put your setup code here, to run once:
  UART1.begin(BAUD_RATE); //RS-485 bus 1
  UART1.setTimeout(100);
  Serial.begin(BAUD_RATE); // Debugging via serial monitor
  Wire2.begin();
  // I2CScanner();

  pinMode(PIN_ARM, OUTPUT);
  pinMode(PIN_DISARM, OUTPUT);

  digitalWrite(PIN_DISARM, HIGH);
  delay(50);
  digitalWrite(PIN_DISARM, LOW);

  // digitalWrite(PIN_ARM, HIGH);
  // delay(50);
  // digitalWrite(PIN_ARM, LOW);

  for (int i = 0; i < 12; i++) {
    pinMode(dcChannels[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(sMux[i], OUTPUT);
  }

  digitalWrite(sMux[0], HIGH);
  digitalWrite(sMux[1], HIGH);
  digitalWrite(sMux[2], HIGH);
  digitalWrite(sMux[3], LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(ptMux[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    digitalWrite(ptMux[i], LOW);
  }

  pinMode(SPI_S_CS, OUTPUT);
  pinMode(SPI_S_IRQ, INPUT);
  // pinMode(MCLK_PIN, OUTPUT);
  pinMode(SPI_S_MISO, INPUT);
  pinMode(SPI_S_MOSI, OUTPUT);

  pinMode(SPI_PT_CS, OUTPUT);
  pinMode(SPI_PT_IRQ, INPUT);
  // pinMode(MCLK_PIN, OUTPUT);
  pinMode(SPI_PT_MISO, INPUT);
  pinMode(SPI_PT_MOSI, OUTPUT);


  SPI.begin();
  SPI.setClockDivider(4);
  
  SPI.setMISO(SPI_S_MISO);
  SPI.setMOSI(SPI_S_MOSI);
  // SPI.setCS(CS_PIN);
  SPI.setSCK(13);

  SPISettings sSettings(20000000, MSBFIRST, SPI_MODE0);
  sADC.setSettings(sSettings);
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

  SPI1.setMISO(SPI_PT_MISO);
  SPI1.setMOSI(SPI_PT_MOSI);
  // SPI.setCS(CS_PIN);
  SPI1.setSCK(SPI_PT_SCK);

  ptADC.setSettings(sSettings);
  delay(100);
  ptADC.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
  delay(100);
  ptADC.writeRegisterDefaults();
  delay(100);
  ptADC.setGain(GainSettings::GAIN_1);
  ptADC.setMuxInputs(MuxSettings::CH1, MuxSettings::AGND);
  ptADC.setVREF(1.25f);

  sSystem.setup();
  // ptSystem.setup();
  // sh.setCommand("s11.01000,s10.02000,sA1.02000,sA0.00010");

  // while(true) {
  //   delay(500);
  //   sh.setCommand("s11.00000,s10.00010,sA1.02000,sA0.00010");
  //   // sh.printCurrentCommand();
  // }

}

void loop() {
  // put your main code here, to run repeatedly:

  // ========== Serial IO/Solenoid Sequence and command handling ==========
  static char rxBuffer[128] = {0}; // Keep buffer between calls
  static uint8_t readIndex = 0;    // Keep track of how full

  static elapsedMillis serialTimer = 0;
  
  static bool collecting = true;
  static bool packetReady = false;

  while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\r') continue;
      
      if (c == '\n') { // End of command
          rxBuffer[readIndex] = '\0'; // Null-terminate string
          packetReady = true; 
          collecting = false;
          readIndex = 0; // Reset for next command
          break;
      }

      if (readIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[readIndex++] = c;
      }

      else {

        rxBuffer[sizeof(rxBuffer) - 1] = '\0';
        packetReady = true;
        collecting = false;
        readIndex = 0;
        break;

      }

      // if (collecting && serialTimer >= SERIAL_TIMEOUT) {
      //   collecting = false;
      //   readIndex = 0; // Reset for next comman
      //   break;
      // }
  }

  if (packetReady) {

    //   if (rxBuffer[0] == 's' && sh.pollCommand() == false) {
      
    //     sh.setCommand(rxBuffer); // Sets the command once
    //     sh.setState(true);

    //     memset(rxBuffer, 0, sizeof(rxBuffer)); // Resets rxBuffer
    //     packetReady = false;

    // }

    char idChar = rxBuffer[0];
    size_t rxBufferLen = strlen(rxBuffer);

    if (idChar == 's') {
      bool isSequence = (strchr(rxBuffer, '.') != nullptr);
      switch (isSequence) {
        case true:

        if (sh.pollCommand() == false) {
          sh.setCommand(rxBuffer);
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }

        break;

        case false:

        if (strlen(rxBuffer) != 3) {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }

        char channelChar = rxBuffer[1];
        char stateChar = rxBuffer[2];
        char end = rxBuffer[3];

        unsigned channel, state;

        if (isxdigit(channelChar) == false || isdigit(stateChar) == false) {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }

        if (end != '\0') {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }

        if (channelChar >= '0' && channelChar <= '9') channel = channelChar - '0';
        else channel = 10 + (toupper(channelChar) - 'A');     // A-F

        state = stateChar - '0';

        if (channel < 1 || channel > 12) {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }
        
        if (state != 0 && state != 1) {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          packetReady = false;
        }

        digitalWrite(dcChannels[channel - 1], state);

        memset(rxBuffer, 0, sizeof(rxBuffer));
        packetReady = false;

        break;
      }

    }

    else if (idChar == 'a') {
      // Arm
      digitalWrite(PIN_ARM, HIGH);
      delay(50);
      digitalWrite(PIN_ARM, LOW);

      Serial.println("Arming...");

      // Reset
      memset(rxBuffer, 0, sizeof(rxBuffer));
      packetReady = false;
    }

    else if (idChar == 'r') {
      // Disarm
      digitalWrite(PIN_DISARM, HIGH);
      delay(50);
      digitalWrite(PIN_DISARM, LOW);

      Serial.println("Reseting...");

      // Reset
      memset(rxBuffer, 0, sizeof(rxBuffer));
      packetReady = false;
    }

  }
  
  sh.update();

  // =========== Solenoid Current Data Acquisition ===========

  // Reading solenoid current data 
  // Mapping multiplexer channels to the corresponding DC channel (Extremely cursed)
  // sSystem.update();

  // Debugging
  // for (int i = 0; i < NUM_DC_CHANNELS; i++) {
  //   if (i != 11) {
  //     Serial.print(sSystem.currentArr[i], 5);
  //     Serial.print("|");
  //   }
  //   else {
  //     Serial.println(sSystem.currentArr[i], 5);
  //   }
  // }

  // ========== PT Data Acquisition ==========

  
  // ptSystem.update();
    // Debugging
  // for (int i = 0; i < 16; i++) {
  //   if (i != 15) {
  //     Serial.print(ptSystem.currentArr[i], 5);
  //     Serial.print("|");
  //   }
  //   else {
  //     Serial.println(ptSystem.currentArr[i], 5);
  //   }
  // }

  // ========== TC/LC Data Acquisition ==========

  /*
  Complete Solenoid Status packet format:
  <Solenoid Packet Identifier>\n
  <Solenoid Channel #>,<State>,<Current e.g. 0900 = 900mA>\n
  .
  .
  .
  <Solenoid Channel #>,<State>,<Current>\n
  <Checksum>
  */

  
  
 

  // sh.printCurrentCommand();
  // delay(500);
  
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



