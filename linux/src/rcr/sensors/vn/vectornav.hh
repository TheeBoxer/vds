#ifndef _RCR_SENSORS_VECTORNAV_HH_
#define _RCR_SENSORS_VECTORNAV_HH_

#include <rcr/util/publish.h>
#include <rcr/beaglebone/io/spi/spi.hh>

#include <cpp_mediator/mediator.hpp>
#include <vn/error.h>
#include <vn/math/vector.h>
#include <vn/protocol/common.h>
#include <vn/protocol/spi.h>
#include <vn/xplat/thread.h>

#include <algorithm>
#include <climits>
#include <iostream>
#include <type_traits>

#include <stdio.h>
#include <string.h>
#include <time.h>

namespace rcr {
namespace sensors {
namespace vn {

namespace structs {
  struct Vec3f { float    x, y, z; };
  struct Vec4f { float w, x, y, z; };
} // namespace structs

static_assert(std::is_same<uint8_t, unsigned char>::value, "This software requires std::uint8_t to be implemented as unsigned char for uint8_t <--> char reinterpert casting.");
static_assert(CHAR_BIT == 8, "This software requires CHAR_BIT to equal exactly 8 for uint8_t <--> char reinterpert casting.");

struct VectorNavHandler;
struct PrintAllStatus : holden::request<VnError, VectorNavHandler> {
  PrintAllStatus(FILE** file, int(*print)(FILE*, const char * fmt, ...))
    : file(file), print(print) {}
  FILE** file = NULL;
  int(*print)(FILE*, const char * fmt, ...) = NULL;
};

class VectorNavHandler
  : public holden::request_handler<PrintAllStatus> {
 public:
  VectorNavHandler(holden::mediator& m) : mediator_(m) {}

  VnError handle(const PrintAllStatus& m);

 private:
  // Reference to the mediator to request dependancies.
  holden::mediator& mediator_;

  vec3f accel_{}, mag_{}, gyro_{};
  float temp_ = 0.f, pressure_ = 0.f;

  // reusable types for api interfacing.
  // use only transiently.
  // not thread safe.
  vec3f vec3f_{};
  vec4f vec4f_{};
};

} // namespace vn
} // namespace sensors
} // namespace rcr

#endif // _RCR_SENSORS_VECTORNAV_HH_
