#ifndef _RCR_SENSORS_VECTORNAV_HH_
#define _RCR_SENSORS_VECTORNAV_HH_

#include <rcr/util/publish.h>

#include <BlackSPI/BlackSPI.h>
#include <cpp_mediator/mediator.hpp>
#include <vn/protocol/common.h>
#include <vn/protocol/spi.h>
#include <vn/xplat/thread.h>

#include <stdio.h>
#include <string.h>

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

void mockspi_initialize(void);
void mockspi_writeread(const char* dataOut, size_t dataOutSize, char* dataIn);

void mockspi_initialize(void)
{
	/* Do nothing since we are faking the SPI interface. */
}

void mockspi_writeread(const char* dataOut, size_t dataOutSize, char* dataIn)
{
	/* This function fakes a SPI subsystem for this example. */

	char yprResponse[] = { (char)0x00, (char)0x01, (char)0x08, (char)0x00, (char)0xd8, (char)0x9c, (char)0xd4, (char)0x42, (char)0x44, (char)0xba, (char)0x9e, (char)0x40, (char)0x4e, (char)0xe4, (char)0x8b, (char)0x40 };

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(dataOut);
	(dataOutSize);

	memcpy(dataIn, yprResponse, sizeof(yprResponse));
}


struct VectorNavHandler;
struct GetAttitudeQuaternion : holden::request<void, VectorNavHandler> {
  structs::Vec4f& vec3f;
};
struct PrintAllStatus : holden::request<void, VectorNavHandler> {
  int(*print)(const char* format, ...);
};

class VectorNavHandler
  //: public holden::request_handler<GetAttitudeQuaternion>
  : public holden::request_handler<PrintAllStatus> {
 public:
  void handle(const PrintAllStatus& m) {
    PUBLISH(1, "responding to PrintAllStatus request");
    /* This example walks through using the VectorNav C Library to connect to
    * and interact with a mock VectorNav sensor through the Serial Peripheral
    * Interaface (SPI). Once you work through and understand the example, you
    * may want to try replacing the mock functions with ones that interface
    * with your SPI subsystem. */

    char txbuf[(char)0x100];
    char rxbuf[(char)0x100];
    size_t bufcmdsize;
    size_t responseSize;
    vec3f vec3f_;
    char strConversions[50];
    size_t i;

    mockspi_initialize();

    /* With SPI 'initialize', let's work through reading the current yaw, pitch,
    * roll values from the sensor. */

    /* First we must generate the command to query the sensor. */
    bufcmdsize = sizeof(txbuf);		/* First set this variable to the size of the buffer. */
    if (VnSpi_genReadYawPitchRoll(
      txbuf,
      &bufcmdsize,				/* Pass in the pointer since the function will set this with the length of the command generate. */
      0,
      &responseSize) != E_NONE)
      PUBLISH(0, "Error generating read yaw, pitch, roll command.");

    /* Send out the command over SPI. */
    mockspi_writeread(
      txbuf,
      responseSize,
      rxbuf);

    /* Now the sensor will have responded with data on this transaction but
    * since the sensor only responds on following transaction, we will
    * disregard this data. These double transactions can be mitigated by only
    * requesting the same data each time or by staggering the the requested
    * data in an appropriate order. */

    /* Make sure enough time has passed for the sensor to format the previous response. */
    VnThread_sleepMs(1);	/* Actual sensor requirement is only 50 us. */

    /* Retransmit so the sensor responds with the previous request. */
    mockspi_writeread(
      txbuf,
      responseSize,
      rxbuf);

    /* Now parse the received response. */
    if (VnSpi_parseYawPitchRoll(
      rxbuf,
      &vec3f_) != E_NONE)
      PUBLISH(0, "Error parsing yaw, pitch, roll.");

    str_vec3f(strConversions, vec3f_);
    printf("Current YPR: %s\n", strConversions);

    /* We have now shown how to process one full command transaction which
    * requires two SPI transactions because the VectorNav sensor requires a
    * short amount of time to ready the response. Now we can optimize this
    * transaction squence to utilize this behavior when we are only requesting
    * the same data each time. This is illustrated in the for loop below. */

    for (i = 0; i < 25; i++)
    {
      /* For this loop, we want to display data at ~10 Hz. */
      VnThread_sleepMs(100);

      /* Perform a transaction for the same sensor register. */
      mockspi_writeread(
        txbuf,
        responseSize,
        rxbuf);

      /* Now since the previous command was for the same register, we will
      * have valid data and can print/use the results. */
      if (VnSpi_parseYawPitchRoll(
        rxbuf,
        &vec3f_) != E_NONE)
        PUBLISH(0, "Error parsing yaw, pitch, roll.");

      str_vec3f(strConversions, vec3f_);
      printf("Current YPR: %s\n", strConversions);
    }

    /* We illustrate how to write settings to the sensor by changing the
    * asynchronous data output type. Note that this setting only affects the
    * output on the UART ports and has no effect on the SPI ports. It is only
    * used for illustration purposes. */

    /* Remember to reset the bufcmdsize variable to let the function know how
    * large the provided buffer is. */
    bufcmdsize = sizeof(txbuf);

    if (VnSpi_genWriteAsyncDataOutputType(
      txbuf,
      &bufcmdsize,
      0,
      &responseSize,
      VNYPR) != E_NONE)
      PUBLISH(0, "Error generating write async data output type command.\n");

    mockspi_writeread(
      txbuf,
      responseSize,
      rxbuf);
  }

  // void handle(const GetAttitudeQuaternion& m) {
  //   PUBLISH(1, "reading attitude quaternion");
  //   detail::apivec_to_vec(vec4f_, m.vec3f);
  //   auto error = VnSensor_readAttitudeQuaternion(&vn_, &vec4f_);
  //   PUBLISH(0, "VnSensor_readAttitudeQuaternion called with result %d", error);
  //   PUBLISH(1, "attitude quaternion read as:\n  w: %f\n  x: %f\n  y: %f\n  z: %f\n", vec4f_.c[0], vec4f_.c[1], vec4f_.c[2], vec4f_.c[3]);

  //   // TODO: see if these are the same
  //   VnSensor_readYawPitchRoll(&vn_, &vec3f_);
  //   VnSensor_readImuMeasurements(&vn_, &imu_data_);
  // }

  // VnError Init() {
  //   PUBLISH(1, "vn sensor struct initializing");
  //   auto error = VnSensor_initialize(&vn_);
  //   PUBLISH(0, "VnSensor_initialize with result %d", error);
  //   return error;
  // }

  // VnError Connect(const char* port_name, uint32_t baudrate) {
  //   PUBLISH(1, "vn sensor connecting");
  //   auto error = VnSensor_connect(&vn_, port_name, baudrate);
  //   PUBLISH(0, "VnSensor_connect called with result %d", error);
  //   return error;
  // }

 private:
  // SPI API for interfacing with BeagleBone.
  BlackLib::BlackSPI spi_{ BlackLib::spiName::SPI0_0 };

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

