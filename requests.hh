// The following data structures are provided as standardized requests for data 
// available throughout the system.

#ifndef _RCR_VDS_REQUESTS_HH_
#define _RCR_VDS_REQUESTS_HH_

#include <typeinfo>
#include <vector>

namespace rcr {
namespace vds {
namespace requests {

struct Output {};

// TODO:
struct GetAltitude {};
struct GetVoltage {};
struct SetPadAltitude {};
struct SetEncoderPosition {};

const std::vector<const std::type_info&> requests = std::vector<const std::type_info&>{
  typeid(GetAltitude),
  typeid(GetVoltage),
};
static_assert(, "");

class Mediator {
 public:
  template <typename TRequest, typename TResponse>
  TResponse Send(TRequest r) {
    switch (typeid(r)) {
      case typeid(Output): {

      }
      default: { break; }
    }
  }
};

} // namespace requests
} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_REQUESTS_HH_
