#ifndef _RCR_VDS_DAQ_HH_
#define _RCR_VDS_DAQ_HH_

#include "Adafruit_BMP280.h"
#include "Adafruit_BNO055.h"
#include "globals.hh"

namespace rcr {
namespace vds {

class DaqController {
 public:
	void init(bool bnoToo);
	void setPadAlt();
  bool getRawState(VehicleState* rawState, bool testMode);

 protected:
  Adafruit_BNO055 bno{};
  Adafruit_BMP280 bmp280{};
	float padAlt;
	bool timeOverflow = false;
	float lastAlt;
	float altitude_plz();
  float calculateVelocity(VehicleState rawState);
  float get_vertical_accel();
	
  // deep copy
  void copyState(VehicleState* destination, VehicleState* original);

  // Contains past BUFF_N state structures.
  VehicleState pastRawStates[BUFF_N];
};

extern DaqController daq_controller;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_DAQ_HH_
