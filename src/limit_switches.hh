#ifndef _RCR_LIMITSWITCHES_HH_
#define _RCR_LIMITSWITCHES_HH_

#include <mediator.hpp>

template <typename T, typename U>
using request = holden::mediator::request<T, U>;
template <typename T>
using handler = holden::mediator::request_handler<T>;

namespace rcr {
namespace vds {
namespace blades {
namespace limit_switches {

enum class LimitSwitch {
  Inner,
  Outer,
};

class LimitSwitchHandler;

struct IsInnerSwitchEnabled : public request<bool, LimitSwitchHandler> {};
struct IsOuterSwitchEnabled : public request<bool, LimitSwitchHandler> {};

class LimitSwitchHandler
  : public handler<IsInnerSwitchEnabled>
  , public handler<IsOuterSwitchEnabled> {
 public:
  // TODO: implement these (was digitalRead(rcr::vds::digital_io::Pin::LIM_IN);)
  bool handle(const IsOuterSwitchEnabled& m) { return true; }
  bool handle(const IsInnerSwitchEnabled& m) { return true; }
};

} // namespace limit_switches
} // namespace blades
} // namespace vds
} // namespace rcr

#endif // _RCR_LIMITSWITCHES_HH_
