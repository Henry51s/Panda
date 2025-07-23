#include "Mux.hpp"

Multiplexer::Multiplexer(uint8_t s0_pin, uint8_t s1_pin, uint8_t s2_pin, uint8_t s3_pin) {
  this->selection_pins[0] = s0_pin;
  this->selection_pins[1] = s1_pin;
  this->selection_pins[2] = s2_pin;
  this->selection_pins[3] = s3_pin;
}

void Multiplexer::setActiveChannel(uint8_t active_channel) {
  if (active_channel >= 16) {
    return;
  }
  uint8_t pin_state[4];
  pin_state[0] = active_channel & 0x01;
  pin_state[1] = (active_channel >> 1) & 0x01;
  pin_state[2] = (active_channel >> 2) & 0x01;
  pin_state[3] = (active_channel >> 3) & 0x01;

  for (int i = 0; i < 3; i++) {
    digitalWrite(selection_pins[i], pin_state[i]);
  }
}