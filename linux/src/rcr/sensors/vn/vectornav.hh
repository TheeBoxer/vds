#ifndef _RCR_SENSORS_VECTORNAV_HH_
#define _RCR_SENSORS_VECTORNAV_HH_

#include <cpp_mediator/mediator.hpp>
#include <vn/sensors.h>

namespace rcr {
namespace sensors {

struct VectorNavHandler;


struct GetAltitude : holden::mediator::request<float, VectorNavHandler> {};
struct GetAttitudeQuaternion : holden::mediator::request<void, VectorNavHandler> {
  float*& const vector3f;
};


class VectorNavHandler
  : public holden::mediator::request_handler<GetAltitude>
  , public holden::mediator::request_handler<GetAttitudeQuaternion> {
 public:
  // TODO
  auto handle(const GetAltitude& m) -> GetAltitude::response_type {
      VnApi_getVersion();
    return 0.f;
  }

  // TODO
  auto handle(const GetAttitudeQuaternion& m)
    -> GetAttitudeQuaternion::response_type {
    for (auto i = 0; i < 3; ++i)
      m.vector3f[i] = 0.f;
  }
};

} // namespace sensors
} // namespace rcr

#endif // _RCR_SENSORS_VECTORNAV_HH_

