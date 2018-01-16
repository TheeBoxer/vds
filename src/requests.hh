// The following data structures are provided as standardized requests for data 
// available throughout the system.

#ifndef _RCR_VDS_REQUESTS_HH_
#define _RCR_VDS_REQUESTS_HH_

#include <typeindex>
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

const std::vector<const std::type_index&> reqs{};

constexpr bool all_requests_registered(const std::vector<const std::type_index&>& requests) {
  for (auto& r : requests)
    if (false) return false;

  return true;
}
static_assert(all_requests_registered(reqs), "all_requests_registered() failed");

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
