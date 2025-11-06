#include "FScanner.hpp"

void FScanner::setup() {
    
    // Initializing pins
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 4; j++) {
        pinMode(settingsArr[i].muxPins[j], OUTPUT);
        digitalWrite(settingsArr[i].muxPins[j], LOW);
      }
    }

    // pinMode(ptADCPins.cs, OUTPUT);
    // pinMode(ptADCPins.irq, INPUT);
  
    // pinMode(ptADCPins.miso, INPUT);
    // pinMode(ptADCPins.mosi, OUTPUT);
    
    // Initializing ADC
    adc.setSettings(SPISettingsDefault);
    Serial.println("0");
    delay(100);
    adc.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
    Serial.println("1");
    delay(100);
    adc.writeRegisterDefaults();
    Serial.println("2");
    adc.setGain(GainSettings::GAIN_1);
    adc.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
    adc.setVREF(1.25f);
    adc.setBiasCurrent(BiasCurrentSettings::I_0);
    adc.readAllRegisters();
    adc.printRegisters();

}

void FScanner::update() {

  switch(state) {
    case IDLE:
      // Select mux channel and ADC input channel, reset timer and proceed to WAIT_MUX state
      adc.setMuxInputs(settingsArr[index].adcChannel, MuxSettings::AGND);

      for (int i = 0; i < 4; i++) {
        digitalWrite(settingsArr[index].muxPins[i], (channel >> i) & 1);
      }

      timer = 0;
      state = WAIT_MUX;
      break;
    case WAIT_MUX:
      // wait for T_MUX_SETTLE_US, then start one-shot reading from ADC
      // reset timer, then proceed to WAIT_CONV
      if (timer >= T_MUX_SETTLE_US) {
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
        Serial.println(res);
        settingsArr[index].out[channel] = res;

        // if (bank == 0) {
        //   Serial.print("S Channel: ");
        //   Serial.print(channel + 1);
        //   Serial.print(" | Raw: ");
        //   Serial.println(res, HEX);
        // }
        // else if (bank == 1) {
        //   Serial.print("PT Channel: ");
        //   Serial.print(channel + 1);
        //   Serial.print(" | Raw: ");
        //   Serial.println(res, HEX);
        // }
        // else if (bank == 2) {
        //   Serial.print("LCTC Channel: ");
        //   Serial.print(channel + 1);
        //   Serial.print(" | Raw: ");
        //   Serial.println(res, HEX);
        // }

        // Advancing channel and/or bank
        channel++;
        if (channel >= settingsArr[index].numChannels) {
          channel = 0;
          index++;
          if (index >= 2) {
            index = 0;
          }
        }
        timer = 0;
        state = IDLE;
      }
      break;
    }
  }

  void FScanner::getPTOutput(float* out) {out = ptOutput;}
  void FScanner::getLCTCOutput(float* out) {out = lctcOutput;}
