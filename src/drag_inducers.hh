#ifndef _RCR_VDS_DRAGINDUCERS_HH_
#define _RCR_VDS_DRAGINDUCERS_HH_

#include "pid.hh"

namespace rcr {
namespace vds {

class DragInducers {
 public:
  DragInducers() : motorPID(&encPos, &mtrSpdCmd, &encPosCmd, KP, KI, KD, KN, -255, 255) {}
  void init();
  void dragBladesCheck();
  void powerTest();
  int airBrakesGoToEncPos(float vehVel, float sppVel);
  void motorDo(bool direction, uint8_t speed);
  void motorDont();
  bool motorGoTo(int16_t encCmd);
  void motorTest();
  void motorExercise();

  int encMin = 0;
  int encMax = ENC_RANGE;
  volatile int encPos = 0;

  // encoder position command
  int encPosCmd = 0;

 private:
	Pid motorPID;

  // motor speed command
  int mtrSpdCmd = 0;

	void motorGoToPersistent(uint16_t goTo);
};

extern DragInducers drag_inducers;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_DRAGINDUCERS_HH_