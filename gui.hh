#ifndef _RCR_VDS_GUI_HH_
#define _RCR_VDS_GUI_HH_

#include "vehicle_state.hh"

namespace rcr {
namespace vds {

class Gui {
 public:
	void init();

	void rocketMenu();

	void printPastStates(VehicleState* states);

  // prints one state and it's location in the pastRawStates array.
  void printState(VehicleState state, int label);

  // prints one state and it's location in the pastRawStates array.
  void printState(VehicleState, const char* label);

	void printTitle();

	void printMenu();

	void flush_input();

 protected:
	void editRocket();
	float readFloat(int address);
	void writeFloat(float value, int address);
	String readString(int address);
	uint8_t readUint8_t(int address);
	void writeUint8_t(uint8_t value, int address);
	void writeString(String value, int address);
	bool loadRocket(uint8_t whichOne);
	void saveRocket(uint8_t whichOne);
	void printRocket();
	void printRocketMenu();
	uint8_t currentRocket = 1;
};

extern Gui gui;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_GUI_HH_
