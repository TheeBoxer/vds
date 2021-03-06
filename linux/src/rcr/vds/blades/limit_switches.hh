#ifndef _RCR_VDS_BLADES_LIMITSWITCHES_HH_
#define _RCR_VDS_BLADES_LIMITSWITCHES_HH_

#include <cpp_mediator/mediator.hpp>

namespace rcr {
namespace vds {
namespace blades {
namespace limit_switches {

enum class LimitSwitch {
  Inner,
  Outer,
};

class LimitSwitchHandler;

struct IsInnerSwitchEnabled
  : public holden::request<bool, LimitSwitchHandler> {};
struct IsOuterSwitchEnabled
  : public holden::request<bool, LimitSwitchHandler> {};

class LimitSwitchHandler
  : public holden::request_handler<IsInnerSwitchEnabled>
  , public holden::request_handler<IsOuterSwitchEnabled> {
 public:
  // TODO: implement these (was digitalRead(rcr::vds::io::Pin::LimitIn);)
  bool handle(const IsOuterSwitchEnabled& m) { return true; }
  bool handle(const IsInnerSwitchEnabled& m) { return true; }
};

} // namespace limit_switches
} // namespace blades
} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_BLADES_LIMITSWITCHES_HH_
