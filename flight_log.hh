#ifndef _RCR_VDS_FLIGHTLOG_HH_
#define _RCR_VDS_FLIGHTLOG_HH_

#include "state_log.hh"
#include "vehicle_state.hh"

namespace rcr {
namespace vds {

class FlightLog {
 public:
	void printTestFileNames();
	
  void init();

	void* sd; // SdSio

	void logData(bool testMode);

  template <typename T>
	void logError(T message);
	
  void newFlight(bool flightMode);
	
  bool readCSV(VehicleState* destination);

  StateLog supStat;

 protected:
  void* data; // File
  int pos = 0;
  uint32_t testFileSize;
};

extern FlightLog flight_log;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_FLIGHTLOG_HH_
