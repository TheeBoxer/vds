#ifndef _RCR_SENSORS_VECTORNAV_HH_
#define _RCR_SENSORS_VECTORNAV_HH_

#include <rcr/util/publish.h>

#include <BlackSPI/BlackSPI.h>
#include <cpp_mediator/mediator.hpp>
#include <vn/math/vector.h>
#include <vn/protocol/common.h>
#include <vn/protocol/spi.h>
#include <vn/xplat/thread.h>

#include <climits>
#include <iostream>
#include <type_traits>

#include <stdio.h>
#include <string.h>

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

void mockspi_initialize(void) {}
void mockspi_writeread(uint8_t* tx_bytes, uint8_t* recv_bytes, size_t recv_bytes_size) {
  BlackLib::BlackSPI spi{ BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000 };
  spi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
  spi.transfer(tx_bytes, recv_bytes, recv_bytes_size);

  printf("\nLoopback spi test result:\n");
  for (decltype(recv_bytes_size) i = 0; i < recv_bytes_size; ++i) {
    printf("%x ", static_cast<int>(recv_bytes[i]));
  }
}

struct VectorNavHandler;
struct GetAttitudeQuaternion : holden::request<void, VectorNavHandler> {
  structs::Vec4f& vec4f;
};
struct PrintAllStatus : holden::request<void, VectorNavHandler> {
  int(*print)(const char* format, ...);
};

class VectorNavHandler
  //: public holden::request_handler<GetAttitudeQuaternion>
  : public holden::request_handler<PrintAllStatus> {
 public:
  VectorNavHandler(holden::mediator& m) : mediator_(m) {}

  void handle(const PrintAllStatus& m) {
    PUBLISH(1, "responding to PrintAllStatus request\n");

    std::uint8_t txbuf[0x100];
    uint8_t rxbuf[0x100];
    const std::size_t txbuf_size = sizeof(txbuf);
    std::size_t txcommand_size = 0;
    size_t responseSize;
    char strConversions[50];

    mockspi_initialize();

    {
      auto error = VnSpi_genReadYawPitchRoll(
        reinterpret_cast<char*>(txbuf),
        &txcommand_size,
        0,
        &responseSize);
      PUBLISH(0, "generated read yaw, pitch, roll command with result %d\n", error);
    }

    // Send out the YPR command over SPI
    mockspi_writeread(
      txbuf,
      rxbuf,
      responseSize);

    /* Now the sensor will have responded with data on this transaction but
    * since the sensor only responds on following transaction, we will
    * disregard this data. These double transactions can be mitigated by only
    * requesting the same data each time or by staggering the the requested
    * data in an appropriate order. */

    // TODO: lower this? (Actual sensor requirement is only 50 us.)
    // permit VN time to format the previous response
    VnThread_sleepMs(1);

    // Retransmit so the sensor responds with the previous request.
    mockspi_writeread(
      txbuf,
      rxbuf,
      responseSize);

    // Parse the response.
    {
      auto error = VnSpi_parseYawPitchRoll(
        reinterpret_cast<const char*>(rxbuf),
        &vec3f_);
      PUBLISH(1, "parsed yaw, pitch, roll with result %d", error);
    }
    
    str_vec3f(strConversions, vec3f_);
    printf("\nCurrent YPR: %s", strConversions);

    /* We have now shown how to process one full command transaction which
    * requires two SPI transactions because the VectorNav sensor requires a
    * short amount of time to ready the response. Now we can optimize this
    * transaction squence to utilize this behavior when we are only requesting
    * the same data each time. This is illustrated in the for loop below. */

    for (auto i = 0; i < 5; i++)
    {
      /* For this loop, we want to display data at ~10 Hz. */
      VnThread_sleepMs(100);

      /* Perform a transaction for the same sensor register. */
      mockspi_writeread(
        txbuf,
        rxbuf,
        responseSize);

      /* Now since the previous command was for the same register, we will
      * have valid data and can print/use the results. */
      auto error = VnSpi_parseYawPitchRoll(
        reinterpret_cast<const char*>(rxbuf),
        &vec3f_);
      PUBLISH(1, "yaw, pitch, roll parsed with result %d", error);

      
      DO_AND_PUBLISH(str_vec3f(strConversions, vec3f_), 0, "Current YPR: %s\n", strConversions);
    }
  }

 private:
  // Reference to the mediator to request dependancies.
  holden::mediator& mediator_;

  // SPI API for interfacing with BeagleBone.
  BlackLib::BlackSPI spi_{ BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000 };

  // // VN100 API class from manufacturer code.
  // VnSensor vn_{};

  // // reusable types for api interfacing.
  // // use only transiently.
  // // not thread safe.
  // ImuMeasurementsRegister imu_data_{};
  vec3f vec3f_{};
  vec4f vec4f_{};
};

} // namespace vn
} // namespace sensors
} // namespace rcr

#endif // _RCR_SENSORS_VECTORNAV_HH_

