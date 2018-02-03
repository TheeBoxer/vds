#include "pin_io.hh"

#include <cstdint>

// TODO: these
void analog_write(int address, uint8_t value) {}
void digital_write(int address, bool value) {}

void rcr::vds::io::analog::AnalogIoHandler::handle(const AnalogWrite& m) {
  analog_write(static_cast<int>(m.pin), m.value);
}

void rcr::vds::io::digital::DigitalIoHandler::handle(const DigitalWrite& m) {
  analog_write(static_cast<int>(m.pin), m.value);
}
