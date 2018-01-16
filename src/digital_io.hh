#ifndef _RCR_DIGITALIO_HH_
#define _RCR_DIGITALIO_HH_

#include <mediator.hpp>

using mediator = holden::mediator::mediator;
template <typename T, typename U>
using request = holden::mediator::request<T, U>;
template <typename T>
using handler = holden::mediator::request_handler<T>;

namespace rcr {
namespace vds {
namespace digital_io {

enum class Pin {
  MOTOR_A,
  MOTOR_B,
  MOTOR_PWM,
  ENC_A,
  ENC_B,
  LIM_IN,
  LIM_OUT,
};

enum class Logic { Low, High };

class DigitalIoHandler;
struct DigitalWrite : public request<void, DigitalIoHandler> {
  Pin pin;
  Logic l;
};

class DigitalIoHandler
  : public handler<DigitalWrite> {
 public:
  void handle(const DigitalWrite& m) {
    // TODO: write out.
  }
};

} // namespace digital_io
} // namespace vds
} // namespace rcr

#endif // _RCR_DIGITALIO_HH_
