#ifndef _RCR_VDS_DAQ_HH_
#define _RCR_VDS_DAQ_HH_

#include "globals.hh"
#include "vehicle_state.hh"

namespace rcr {
namespace vds {

class DaqController {
 public:
	void init(bool bnoToo);
	void setPadAlt();
  bool getRawState(VehicleState* rawState, bool testMode);

 protected:
  void* bno{};
  void* bmp280{};
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

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_DAQ_HH_
