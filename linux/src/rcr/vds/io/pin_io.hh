#ifndef _RCR_VDS_IO_PINIO_HH_
#define _RCR_VDS_IO_PINIO_HH_

#include "../../../../lib/cpp-mediator/include/cpp_mediator/mediator.hpp"

#include <cstdint>

namespace rcr {
namespace vds {
namespace io {

enum class Pin {
  EncA,
  EncB,
  LimitIn,
  LimitOut,
  MotorA,
  MotorB,
  MotorPwm,
};

enum class InputPin {
  EncA   = static_cast<int>(Pin::EncA),
  EncB   = static_cast<int>(Pin::EncB),
  LimitIn  = static_cast<int>(Pin::LimitIn),
  LimitOut = static_cast<int>(Pin::LimitOut),
};

enum class OutputPin {
  MotorA   = static_cast<int>(Pin::MotorA),
  MotorB   = static_cast<int>(Pin::MotorB),
  MotorPwm = static_cast<int>(Pin::MotorPwm),
};

namespace analog {

class AnalogIoHandler;
struct AnalogWrite
  : public holden::request<void, AnalogIoHandler> {
  AnalogWrite(Pin p, uint8_t value) : pin(p), value(value) {}
  Pin pin;
  uint8_t value;
};

class AnalogIoHandler
  : public holden::request_handler<AnalogWrite> {
 public:
  void handle(const AnalogWrite& m);
};

} // namespace analog

namespace digital {

class DigitalIoHandler;
struct DigitalWrite
  : public holden::request<void, DigitalIoHandler> {
  DigitalWrite(Pin p, bool value) : pin(p), value(value) {}
  Pin pin;
  bool value;
};

class DigitalIoHandler
  : public holden::request_handler<DigitalWrite> {
 public:
  void handle(const DigitalWrite& m);
};

} // namespace digital

} // namespace io
} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_IO_PINIO_HH_
