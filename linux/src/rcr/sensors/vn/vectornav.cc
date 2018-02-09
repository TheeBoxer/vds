#include <rcr/sensors/vn/vectornav.hh>

#include <vn/sensors/ezasyncdata.h>

namespace rcr {
namespace sensors {
namespace vn {

VnError processErrorReceived(char* errorMessage, VnError errorCode) {
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
  const char SENSOR_PORT[] = "COM1"; 	/* Windows format for physical and virtual (USB) serial port. */
  /*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
  /*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
  /*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
  /*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
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
