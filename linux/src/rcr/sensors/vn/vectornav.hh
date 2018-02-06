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

static_assert(std::is_same<uint8_t, unsigned char>::value, "This software requires std::uint8_t to be implemented as unsigned char for uint8_t <--> char reinterpert casting.");
static_assert(CHAR_BIT == 8, "This software requires CHAR_BIT to equal exactly 8 for uint8_t <--> char reinterpert casting.");

struct VectorNavHandler;
struct GetAttitudeQuaternion : holden::request<void, VectorNavHandler> {
  structs::Vec4f& vec4f;
};
struct PrintAllStatus : holden::request<void, VectorNavHandler> {
  PrintAllStatus(FILE** file, int(*print)(FILE*, const char * fmt, ...))
    : file(file), print(print) {}
  FILE** file = NULL;
  int(*print)(FILE*, const char * fmt, ...) = NULL;
};

class VectorNavHandler
  //: public holden::request_handler<GetAttitudeQuaternion>
  : public holden::request_handler<PrintAllStatus> {
 public:
  VectorNavHandler(holden::mediator& m) : mediator_(m) {}

  void handle(const PrintAllStatus& m) {
    PUBLISH(1, "responding to PrintAllStatus request\n");
    if (m.file == NULL || m.print == NULL) {
      printf("ERRR");
    }

    std::uint8_t txbuf[0x100];
    std::uint8_t rxbuf[0x100];
    std::size_t txcommand_size = sizeof(txbuf);
    std::size_t response_size = 0;
    char strConversions[50];

    auto gen_error = VnSpi_genReadYawPitchRoll(
      reinterpret_cast<char*>(txbuf),
      &txcommand_size,
      0,
      &response_size);
    PUBLISH(1, "generated read yaw, pitch, roll command with\n");
    PUBLISH(1, "  error:                  %d\n", gen_error);
    PUBLISH(1, "  command size:           %lu\n", txcommand_size);
    PUBLISH(1, "  expected response size: %lu\n", response_size);
    PUBLISH(1, "  ");
    DO_AND_PUBLISH(for(auto i=0; i<txcommand_size; ++i) PUBLISH(1, "%x, ", txbuf[i]), 1, "\n");

    beaglebone::io::spi::WriteRead request{txbuf, rxbuf, std::max(txcommand_size, response_size)};

    mediator_.send(request);
    VnThread_sleepUs(100);
    mediator_.send(request);

    auto parse_error = VnSpi_parseYawPitchRoll(
      reinterpret_cast<const char*>(rxbuf),
      &vec3f_);
    PUBLISH(1, "yaw, pitch, roll parsed with result %d", parse_error);
    DO_AND_PUBLISH(str_vec3f(strConversions, vec3f_), 0, "Current YPR: %s\n", strConversions);
  }

 private:
  // Reference to the mediator to request dependancies.
  holden::mediator& mediator_;

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

