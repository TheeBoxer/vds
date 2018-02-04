#ifndef _RCR_SENSORS_VECTORNAV_HH_
#define _RCR_SENSORS_VECTORNAV_HH_

#include <BlackSPI/BlackSPI.h>
#include <cpp_mediator/mediator.hpp>
#include <vn/sensors.h>
#include <rcr/util/publish.h>

#define MAX_PUBLISH_DETAIL 10

namespace rcr {
namespace sensors {
namespace vn {

namespace structs {
  struct Vec3f { float    x, y, z; };
  struct Vec4f { float w, x, y, z; };
} // namespace structs

namespace detail {
  void apivec_to_vec(vec3f& a, structs::Vec3f& b) {
    b.x = a.c[0];
    b.y = a.c[1];
    b.z = a.c[2];
  }

  void apivec_to_vec(vec4f& a, structs::Vec4f& b) {
    b.w = a.c[0];
    b.x = a.c[1];
    b.y = a.c[2];
    b.z = a.c[3];
  }
} // namespace detail

struct VectorNavHandler;
struct GetAttitudeQuaternion : holden::request<void, VectorNavHandler> {
  structs::Vec4f& vec3f;
};
struct PrintAllStatus : holden::request<void, VectorNavHandler> {
  int(*print)(const char* format, ...);
};

class VectorNavHandler
  : public holden::request_handler<GetAttitudeQuaternion>
  , public holden::request_handler<PrintAllStatus> {
 public:
  VnError Init() {
    PUBLISH(1, "vn sensor struct initializing");
    auto error = VnSensor_initialize(&vn_);
    PUBLISH(0, "VnSensor_initialize with result %d", error);
    return error;
  }

  VnError Connect(const char* port_name, uint32_t baudrate) {
    PUBLISH(1, "vn sensor connecting");
    auto error = VnSensor_connect(&vn_, port_name, baudrate);
    PUBLISH(0, "VnSensor_connect called with result %d", error);
    return error;
  }

  void handle(const PrintAllStatus& m) {
    PUBLISH(1, "responding to PrintAllStatus request");


  }

  void handle(const GetAttitudeQuaternion& m) {
    PUBLISH(1, "reading attitude quaternion");
    detail::apivec_to_vec(vec4f_, m.vec3f);
    auto error = VnSensor_readAttitudeQuaternion(&vn_, &vec4f_);
    PUBLISH(0, "VnSensor_readAttitudeQuaternion called with result %d", error);
    PUBLISH(1, "attitude quaternion read as:\n  w: %f\n  x: %f\n  y: %f\n  z: %f\n", vec4f_.c[0], vec4f_.c[1], vec4f_.c[2], vec4f_.c[3]);

    // TODO: see if these are the same
    VnSensor_readYawPitchRoll(&vn_, &vec3f_);
    VnSensor_readImuMeasurements(&vn_, &imu_data_);
  }


 private:
  // VN100 API class from manufacturer code.
  VnSensor vn_{};

  // SPI API for interfacing with BeagleBone.
  BlackLib::BlackSPI spi_{ BlackLib::spiName::SPI0_0 };

  // reusable types for api interfacing.
  // use only transiently.
  // not thread safe.
  ImuMeasurementsRegister imu_data_{};
  vec3f vec3f_{};
  vec4f vec4f_{};
};

} // namespace vn
} // namespace sensors
} // namespace rcr

#endif // _RCR_SENSORS_VECTORNAV_HH_

