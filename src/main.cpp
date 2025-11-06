#include <FlexSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "drivers/MCP3561.hpp"
#include "hardware-configs/pins.hpp"

// #include "scanners/Scanner.hpp"
#include "scanners/FScanner.hpp" // Fluids DAQ
// #include "scanners/SScanner.hpp" // Solenoid current DAQ

#include "telemetry/TelemetryHandler.hpp"

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
// FlexSerial UART2(UART2Pins.rx, UART2Pins.tx); // RX, TX



// MCP3561 sADC(sADCPins.cs, SPI, SPISettingsDefault); // Solenoid current ADC
MCP3561 ptADC(ptADCPins.cs, SPI1, SPISettingsDefault); // Fluids data ADC
FScanner fScanner(ptADC);

TelemetryHandler th(Serial2, 256);



void setup() {
  // put your setup code here, to run once:
  Serial2.begin(SERIAL_BAUD_RATE); //RS-485 bus 1
  Serial2.setTimeout(100);

  static uint8_t txBuf[TX_BUF_SIZE]; // Literally downloading more memory
  Serial2.addMemoryForWrite(txBuf, TX_BUF_SIZE);

  Serial.begin(SERIAL_BAUD_RATE); // Debugging via serial monitor

  
  while(!Serial);


  // SPI.begin();
  // SPI.setClockDivider(4);
  
  // SPI.setMISO(sADCPins.miso);
  // SPI.setMOSI(sADCPins.mosi);
  // SPI.setSCK(sADCPins.sck);


  SPI1.begin();
  SPI1.setClockDivider(4);

  SPI1.setMISO(ptADCPins.miso);
  SPI1.setMOSI(ptADCPins.mosi);
  SPI1.setSCK(ptADCPins.sck);

  fScanner.setup();
  // I2CScanner();
  // Wire2.begin();

  // for (int i = 0; i < NUM_DC_CHANNELS; i++) {
  //   pinMode(PINS_DC_CHANNELS[i], OUTPUT);
  // }



  // Call scanner setup()'s here

}

void loop() {
  // // put your main code here, to run repeatedly:

  th.poll();
  if (th.isPacketReady()) {
    char* rxPacket = th.takePacket();
    // Feed rxPacket into command handler
  }




  
  // TODO: Overhaul commands
  //   if (idChar == 's') {
  //     Serial.println(rxBuffer);
  //     sh.setCommand(rxBuffer);
  //   }

  //   else if (idChar == 'S') {

  //     char channelChar = rxBuffer[1];
  //     char stateChar = rxBuffer[2];

  //     unsigned channel, state;

  //     if (channelChar >= '0' && channelChar <= '9') channel = channelChar - '0';
  //     else channel = 10 + (toupper(channelChar) - 'A');     // A-F

  //     state = stateChar - '0';

  //     // digitalWrite(dcChannels[channel - 1], state);
  //     if (channel >= 1 && channel <= NUM_DC_CHANNELS) {
  //       dcChannels[channel - 1].setState(state);
  //     }

  //   }

  //   else if (idChar == 'a') {
  //     // Arm
  //     disarmPulse.cancel();
  //     armPulse.start();

  //   }

  //   else if (idChar == 'r') {
  //     // Disarm
  //     armPulse.cancel();
  //     disarmPulse.start();
  //   }

  //   else if (idChar == 'f') {
  //     Serial.println(rxBuffer);
  //     sh.setState(true);
  //   }

  //   // Resetting all incoming buffers
  //   memset(rxBuffer, 0, sizeof(rxBuffer));
  //   packetReady = false;
  // }




  // =========== Data Acquisition ===========

  // fScanner.update();

  // float ptData[NUM_PT_CHANNELS], lctcData[NUM_LC_CHANNELS + NUM_TC_CHANNELS];

  // ========== Telemetry ===========
  // char sPacket[512], ptPacket[512], lctcPacket[512];

  
  // toCSVRow(banks[1].data,'p', NUM_PT_CHANNELS, ptPacket, sizeof(ptPacket), DATA_DECIMALS);
  // toCSVRow(banks[2].data,'t', NUM_LC_CHANNELS + NUM_TC_CHANNELS, lctcPacket, sizeof(lctcPacket), DATA_DECIMALS);

  // ========== Printing packets ==========
  // Serial2.print(sPacket);
  // Serial2.print(ptPacket);
  // Serial2.print(lctcPacket);

  // Serial.println(ptPacket);
  // Serial2.write(ptPacket, ptPacketLen);
  // ==========
  // Serial.println("Transmission time: " + String(serialTimer - currentTime) + "us");
  // Serial.println(Serial.availableForWrite());
  // Serial.println(ptPacket);
}


