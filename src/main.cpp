#include <FlexSerial.h>
#include <SPI.h>
#include "SequenceHandler.hpp"
#include <Wire.h>
#include "MCP3561.hpp" // ADC
#include "DCChannel.hpp"
#include "Constants.hpp"
#include <SD.h>
// #include "BangBangController.hpp"

/* 
TODO:
- Replace while(Serial.available()) with non-blocking reads
- Clean up SPI and ADC initialization
- Implement Bang-Bang controllers and DCChannel object *NOT BEING IMPLEMENTED 10/7/2025*
- Add channel state to data packet
- Implement telemetry on/off control
- Rewrite SequenceHandler to use dcChannels array
- Overhaul telemetry
- Parallelize ADC readings

*/

/**
 * SERIAL PROTOCOL (Baud rate: 115200)
 * 
 * Command Format:
 * 'a' - Arm
 * 'd' - Disarm
 * 's<Channel Identifier in Hex><State>.<Five digit delay in milliseconds>' - Solenoid Sequence
 * 'S<Channel Identifier in Hex><State>' - Solenoid command
 * 
 * 'B'<Channel Identifier in Hex><State> - Bang-Bang channel activation/deactivation
 * 'b'<Channel Identifier in Hex>.<Lower Deadband in raw voltage>.<Upper Deadband in raw voltage>.<Min off time in milliseconds>.<Min on time in milliseconds>
 * 
 * Telemetry Format:
 * 'p<Voltage across shunt resistor>' x16 - Pressure Transducer Packet
 * 's<Voltage across shunt resistor>' x12 - Solenoid Current Packet
 * 't<Conditioned thermocouple voltage>' x6 - Thermocouple Voltage Packet
 * 'l<Conditioned load cell voltage>' x6 - Load Cell Voltage Packet
 * 
 */

// Cursed UART pin config
// FlexSerial UART1(UART1Pins.rx, UART1Pins.tx); // RX, TX
FlexSerial UART2(UART2Pins.rx, UART2Pins.tx); // RX, TX

MCP3561 sADC(sADCPins.cs, SPI);
MCP3561 ptADC(ptADCPins.cs, SPI1);

SequenceHandler sh;

// ========== Telemetry Functions =========

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


// // ========== State machine ==========

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
  uint8_t irqPin;
};

static float sData[NUM_DC_CHANNELS] = {0}, ptData[NUM_PT_CHANNELS] = {0}, lctcData[NUM_TC_CHANNELS + NUM_LC_CHANNELS] = {0};

/*
bank[0] - Solenoid channel current reading
bank[1] - PT channel reading
bank[2] - LC & TC channel reading (LC and TC data arrays are combined because they share the same mux)
*/
static constexpr Bank banks[] = {

  {sADC, MuxSettings::CH0, sMux, sData, NUM_DC_CHANNELS, 9},
  {ptADC, MuxSettings::CH1, ptMux, ptData, NUM_PT_CHANNELS, 2},
  {ptADC, MuxSettings::CH0, lctcMux, lctcData, NUM_TC_CHANNELS + NUM_LC_CHANNELS, 2}

};

static constexpr uint8_t NUM_BANKS = sizeof(banks) / sizeof(banks[0]);

// Bank sBank = {sADC, MuxSettings::CH0, sMux, sData, NUM_DC_CHANNELS};
// Bank ptBank = {ptADC, MuxSettings::CH1, ptMux, ptData, NUM_PT_CHANNELS};
// Bank lctcBank = {ptADC, MuxSettings::CH0, lctcMux, lctcData, NUM_TC_CHANNELS + NUM_LC_CHANNELS};

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
        pinMode(b.irqPin, INPUT);
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
        if (timer >= T_CONV_US || digitalRead(currentBank.irqPin) == LOW) {
          // Read ADC output
          // Set samples[channel] to what the ADC outputs
          float res = currentBank.adc.getOutput();
          currentBank.data[channel] = res;

          if (bank == 0) {
            Serial.print("S Channel: ");
            Serial.print(channel + 1);
            Serial.print(" | Raw: ");
            Serial.println(res, HEX);
          }
          else if (bank == 1) {
            Serial.print("PT Channel: ");
            Serial.print(channel + 1);
            Serial.print(" | Raw: ");
            Serial.println(res, HEX);
          }
          else if (bank == 2) {
            Serial.print("LCTC Channel: ");
            Serial.print(channel + 1);
            Serial.print(" | Raw: ");
            Serial.println(res, HEX);
          }

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


// ADCMuxSystem scanner;
Pulse armPulse(PIN_ARM);
Pulse disarmPulse(PIN_DISARM);

elapsedMicros serialTimer;
elapsedMillis tareTimer = 0;

ADCMuxSystem scanner;

bool tared = false;
double offset1 = 0, offset2 = 0;
double sum1 = 0 , sum2 = 0;
int counter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(SERIAL_BAUD_RATE); //RS-485 bus 1
  Serial2.setTimeout(100);
  static uint8_t buf[300];
  Serial2.addMemoryForWrite(buf, sizeof(buf));
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
  // ptADC.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
  delay(100);
  // ptADC.writeRegisterDefaults();
  ptADC.setGain(GainSettings::GAIN_1);
  ptADC.setMuxInputs(MuxSettings::AGND, MuxSettings::AGND);
  ptADC.setVREF(1.25f);
  delay(100);
  ptADC.readAllRegisters();

  if (DEBUG_F_ADC) {ptADC.readAllRegisters(); ptADC.printRegisters();}

  scanner.setup();

}

void loop() {
  // put your main code here, to run repeatedly:

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

  // Serial.println(rxBuffer);

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
        Serial.print("Channel: "); Serial.print(channel - 1); Serial.print(" | State: "); Serial.println(state);
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

    else if (idChar == 'B') {

      char channelChar = rxBuffer[1];
      char stateChar = rxBuffer[2];

      unsigned state = stateChar - '0';

      if (channelChar == '1') {
        dcChannels[10].getControllerPtr()->setState(state);
      }

      if (channelChar == '2') {
        dcChannels[11].getControllerPtr()->setState(state);
      }

      Serial.print("Channel: ");
      Serial.println(channelChar);
      Serial.print("State: ");
      Serial.println(stateChar);

    }

    else if (idChar == 'b') {

      Serial.println(rxBuffer);
      // Configuring bang bang deadbands and minimum actuation times
      unsigned channelRx = 0;
      double targetPVRx = 0;
      double upperDeadbandRx = 0;
      double lowerDeadbandRx = 0;
      unsigned long minOffTimeRx = 0;
      unsigned long minOnTimeRx = 0;

      // char* token = strtok(rxBuffer, ".");

      if (sscanf(rxBuffer, "b%x,%lf,%lf,%lf,%u,%u", &channelRx, &targetPVRx, &lowerDeadbandRx, &upperDeadbandRx, &minOffTimeRx, &minOnTimeRx) == 6) {

        if (channelRx == 1) {

          dcChannels[10].getControllerPtr()->setTargetPV(targetPVRx);
          dcChannels[10].getControllerPtr()->setLowerDeadband(lowerDeadbandRx);
          dcChannels[10].getControllerPtr()->setUpperDeadband(upperDeadbandRx);
          dcChannels[10].getControllerPtr()->setMinOffTime(minOffTimeRx);
          dcChannels[10].getControllerPtr()->setMinOnTime(minOnTimeRx);

          if (DEBUG_BB) {

            Serial.println("Received Fuel BB configuration: ");
            Serial.print("Channel: ");
            Serial.println(channelRx);
            Serial.print("Target Pressure, raw value: ");
            Serial.println(targetPVRx, 5);
            Serial.print("Lower deadband, raw value: ");
            Serial.println(lowerDeadbandRx, 5);
            Serial.print("Upper deadband, raw value: ");
            Serial.println(upperDeadbandRx, 5);
            Serial.print("Min off time: ");
            Serial.println(minOffTimeRx);
            Serial.print("Min on time: ");
            Serial.println(minOnTimeRx);

          }
        }

        if (channelRx == 2) {

          dcChannels[11].getControllerPtr()->setTargetPV(targetPVRx);
          dcChannels[11].getControllerPtr()->setLowerDeadband(lowerDeadbandRx);
          dcChannels[11].getControllerPtr()->setUpperDeadband(upperDeadbandRx);
          dcChannels[11].getControllerPtr()->setMinOffTime(minOffTimeRx);
          dcChannels[11].getControllerPtr()->setMinOnTime(minOnTimeRx);

          if (DEBUG_BB) {

            Serial.print("Channel: ");
            Serial.println(channelRx);
            Serial.println("Received LOX BB configuration: ");
            Serial.print("Target Pressure, raw value: ");
            Serial.println(targetPVRx, 5);
            Serial.print("Lower deadband, raw value: ");
            Serial.println(lowerDeadbandRx, 5);
            Serial.print("Upper deadband, raw value: ");
            Serial.println(upperDeadbandRx, 5);
            Serial.print("Min off time: ");
            Serial.println(minOffTimeRx);
            Serial.print("Min on time: ");
            Serial.println(minOnTimeRx);

          }
        }
       }
    }

    memset(rxBuffer, 0, sizeof(rxBuffer));
    packetReady = false;
  }


  armPulse.update();
  disarmPulse.update();
  sh.update();



  // =========== Data Acquisition ===========

  scanner.update();

  // if (!tared && tareTimer > 1000) {

  //   sum1 += ptBank.data[14];
  //   sum2 += ptBank.data[15];
  //   counter++;

  //   if (counter >= 10000) {
  //     offset1 = sum1 / 10000.0;
  //     offset2 = sum2 / 10000.0;
  //     tared = true;
  //   }

  // }

  // ptBank.data[14] -= offset1;
  // ptBank.data[15] -= offset2;

  char sPacket[512], ptPacket[512], lctcPacket[512];

  toCSVRow(banks[0].data,'s', NUM_DC_CHANNELS, sPacket, sizeof(sPacket), DATA_DECIMALS);
  toCSVRow(banks[1].data,'p', NUM_PT_CHANNELS, ptPacket, sizeof(ptPacket), DATA_DECIMALS);
  toCSVRow(banks[2].data,'t', NUM_LC_CHANNELS + NUM_TC_CHANNELS, lctcPacket, sizeof(lctcPacket), DATA_DECIMALS);

  if (serialTimer > SERIAL_WRITE_DELAY) {

    int currentTime = serialTimer;
    // ========== Printing packets ==========
    Serial2.print(sPacket);
    Serial2.print(ptPacket);
    Serial2.print(lctcPacket);
    // Serial2.write(ptPacket, ptPacketLen);
    // ==========
    // Serial.println("Transmission time: " + String(serialTimer - currentTime) + "us");
    // Serial.println(Serial.availableForWrite());
    // Serial.println(ptPacket);
    serialTimer = 0;

  }

  // for (int i = 0; i < 12; i++) {
  // if (dcChannels[i].getControllerPtr() != nullptr) {
  //   if (i == 10) {
  //     dcChannels[i].getControllerPtr()->updateController(ptBank.data[14] + 0.4456);
  //     // dcChannels[i].setState(dcChannels[i].getControllerPtr()->getState());
  //   }
  //   else if (i == 11){
  //     dcChannels[i].getControllerPtr()->updateController(ptBank.data[15]);
  //   }
    
  //   dcChannels[i].setState(dcChannels[i].getControllerPtr()->getState());
  //  }
  // }
}


