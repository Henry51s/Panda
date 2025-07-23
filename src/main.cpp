#include <FlexSerial.h>
#include <SPI.h>
#include "SequenceHandler.hpp"
#include <Wire.h>
#include "MCP3561.hpp" // ADC
#include "Mux.hpp"

#define BAUD_RATE 9600
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

// DC Channel pins. In order of channels 1 to 12
uint8_t dcChannels[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};

// Solenoid current mux pins
uint8_t sMux[4] = {18, 19, 23, 22};

// PT mux pins
uint8_t ptMux[4] = {31, 30, 29, 28};

// TC mux pins
uint8_t tcMuxPins[4] = {3, 4, 6, 5};

// Cursed UART pin config
FlexSerial UART1(8, 7);

// Ability to turn on and off telemetry
bool sendState = false;


MCP3561 sADC(SPI_S_CS, SPI);
MCP3561 ptADC(SPI_PT_CS, SPI1);

SequenceHandler sh;

Multiplexer tcMux(tcMuxPins[0], tcMuxPins[1], tcMuxPins[2], tcMuxPins[3]);

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
  ptADC.setGain(GainSettings::GAIN_4);
  ptADC.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
  ptADC.setVREF(1.25f);

  // digitalWrite(dcChannels[0], HIGH);

  tcMux.setActiveChannel(5);

  // sh.setCommand("s11.01000,s10.02000,sA1.02000,sA0.00010");

  // while(true) {
  //   delay(500);
  //   sh.setCommand("s11.00000,s10.00010,sA1.02000,sA0.00010");
  //   // sh.printCurrentCommand();
  // }

}

void loop() {
  // put your main code here, to run repeatedly:

  static char rxBuffer[128] = {0}; // Keep buffer between calls
  static uint8_t readIndex = 0;    // Keep track of how full

  static elapsedMillis serialTimer = 0;
  
  static bool collecting = true;
  static bool packetReady = false;

  // while (Serial.available() > 0 && serialTimer <= SERIAL_TIMEOUT) {
  //     char c = Serial.read();
      
  //     if (c == '\n') { // End of command
  //         rxBuffer[readIndex] = '\0'; // Null-terminate string
  //         fullPacket = true;
  //         readIndex = 0; // Reset for next command
  //         break;
  //     }
  //     else {
  //         if (readIndex < sizeof(rxBuffer) - 1) { // Prevent buffer overflow
  //             rxBuffer[readIndex++] = c;
  //         }
  //     }
  // }

  // if (fullPacket) {

  //     if (rxBuffer[0] == 's' && sh.pollCommand() == false) {
      
  //     sh.setCommand(rxBuffer); // Sets the command once
  //     sh.setState(true);

  //     memset(rxBuffer, 0, sizeof(rxBuffer)); // Resets rxBuffer
  //     fullPacket = false;
  //   }

  // }



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

  // Solenoid packet format
  

  // float sCurrent = sADC.getOutput() / (20.0f * R_CS_S);
  // Serial.print("Solenoid reading: ");
  // Serial.print(sCurrent, 6);
  // Serial.print(" | ");

  // float ptCurrent = (ptADC.getOutput() / (4.0f));
  // Serial.print("PT reading: ");
  // Serial.println(ptCurrent, 10);

  // // Reading solenoid current data (Extremely cursed)
  // // Mapping multiplexer channels to the corresponding DC channel
  // uint8_t muxMap[12] = {7, 6, 5, 4, 3, 2, 1, 0, 11, 10, 9, 8};

  // for (int i = 0; i < 12; i++) {
  //   uint8_t activeChannelIndex = muxMap[i]; // Active multiplexer channel (0-index)
  //   // uint8_t activeChannel = activeChannelIndex + 1; // Active DC channel (1-index). Don't know if this is necessary

  //   // Converting activeChannelIndex to binary for muxing
  //   for (int j = 0; j < 4; i++) {
  //     uint8_t binaryDigit = (activeChannelIndex >> j) & 1;
  //     digitalWrite(sMux[j], binaryDigit);
  //   }
  // }
  // sADC.readRegisters();
  //UART1.println(adc.getADCData(), BIN);
  //UART1.println(ptADC.getADCData());
  //ptADC.printRegisters();
  //UART1.println(adc.adc_data[0]);
  // ptADC.getOutput();
  // delay(5);

  
  
 

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



