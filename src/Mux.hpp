#ifndef MUX_HPP
#define MUX_HPP

#include "Arduino.h"

class Multiplexer {
  private:
  uint8_t selection_pins[4];

  public:
  Multiplexer(uint8_t s0_pin, uint8_t s1_pin, uint8_t s2_pin, uint8_t s3_pin);
  void setActiveChannel(uint8_t active_channel);

};

#endif