#ifndef _RCR_VDS_FLIGHTLOG_HH_
#define _RCR_VDS_FLIGHTLOG_HH_

#include "state_log.hh"
#include "vehicle_state.hh"

class FlightLog {
 public:
	void printTestFileNames();
	void init();
	void* sd;
	void logData(bool testMode);
	StateLog supStat;
	void logError(const char* error);
	void newFlight(bool flightMode);
	bool readCSV(VehicleState* destination);

 protected:
  File data;
  int pos = 0;
  uint32_t testFileSize;
};

extern FlightLog flight_log;

#endif // _RCR_VDS_FLIGHTLOG_HH_
