#include <rcr/sensors/vn/vectornav.hh>

#include <vn/sensors/ezasyncdata.h>

namespace rcr {
namespace sensors {
namespace vn {

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

VnError processErrorReceived(const char* errorMessage, VnError errorCode) {
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	PUBLISH(0, "%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return errorCode;
}

VnError
VectorNavHandler::handle(const PrintAllStatus& m) {
  PUBLISH(1, "responding to PrintAllStatus request");
  if (m.file == NULL) {
    PUBLISH(1, "error: PrintAllStatus file** is null");
    return E_ILL_CONDITIONED;
  }
  if (*m.file == NULL) {
    PUBLISH(1, "error: PrintAllStatus file* is null");
    return E_ILL_CONDITIONED;
  }
  
  if (m.print == NULL) {
    PUBLISH(1, "error: PrintAllStatus print fn pointer is null");
    return E_ILL_CONDITIONED;
  }
  m.print(*m.file, "handled");
  VnEzAsyncData ez;
  VnError error = E_NONE;
  size_t i = 0;
  char strConversions[50];
  const char SENSOR_PORT[] = "/dev/ttyS1";
  const uint32_t SENSOR_BAUDRATE = 115200;

  if ((error = VnEzAsyncData_initializeAndConnect(&ez, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
    return processErrorReceived("Error connecting to sensor.", error);

  PUBLISH(0, "Displaying yaw, pitch, roll at 5 Hz for 5 seconds.\n");
  for (i = 0; i < 25; i++) {
    VnCompositeData cd;
    VnThread_sleepMs(200);
    cd = VnEzAsyncData_currentData(&ez);
    str_vec3f(strConversions, cd.yawPitchRoll);
    PUBLISH(0, "Current YPR: %s\n", strConversions);
  }

  if ((error = VnSensor_writeAsyncDataOutputType(VnEzAsyncData_sensor(&ez), VNYPR, true)) != E_NONE)
    return processErrorReceived("Error setting async data output type.", error);

  PUBLISH(0, "Displaying yaw, pitch, roll from new ASCII async type.\n");
  for (i = 0; i < 25; i++) {
    VnCompositeData cd;
    VnThread_sleepMs(200);
    cd = VnEzAsyncData_currentData(&ez);
    str_vec3f(strConversions, cd.yawPitchRoll);
    PUBLISH(0, "Current YPR: %s\n", strConversions);
  }

  if ((error = VnEzAsyncData_disconnectAndUninitialize(&ez)) != E_NONE)
    return processErrorReceived("Error disconnecting from sensor.", error);

  DO_AND_PUBLISH(str_vec3f(strConversions, accel_), 3, "  accel:    %s", strConversions);
  DO_AND_PUBLISH(str_vec3f(strConversions, mag_), 3,   "  mag:      %s", strConversions);
  DO_AND_PUBLISH(str_vec3f(strConversions, gyro_), 3,  "  gyro:     %s", strConversions);
  PUBLISH(3, "  temp:     %f", temp_);
  PUBLISH(3, "  pressure: %f", pressure_);
  
  return E_NONE;
}

} // namespace vn
} // namespace sensors
} // namespace rcr
