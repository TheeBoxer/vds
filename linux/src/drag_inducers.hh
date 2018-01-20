#ifndef _RCR_VDS_DRAGINDUCERS_HH_
#define _RCR_VDS_DRAGINDUCERS_HH_

#include <mediator.hpp>
#include "pid.hh"

namespace rcr {
namespace vds {

class DragInducers {
 public:
  DragInducers(holden::mediator::mediator& m) : m_(m),
    motorPID(&encPos, &mtrSpdCmd, &encPosCmd, KP, KI, KD, KN, -255, 255) {}
  void init();
  void dragBladesCheck();
  void powerTest();
  int airBrakesGoToEncPos(float vehVel, float sppVel);
  void motorDo(BladeDirection direction, uint8_t speed);
  void motorDont();
  bool motorGoTo(int encCmd);
  void motorTest();
  void motorExercise();

  int encMin = 0;
  int encMax = ENC_RANGE;
  volatile int encPos = 0;

  // encoder position command
  int encPosCmd = 0;

 private:
   holden::mediator::mediator& m_;
	Pid motorPID;

  // motor speed command
  int mtrSpdCmd = 0;

	void motorGoToPersistent(int goTo);
};

extern DragInducers drag_inducers;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_DRAGINDUCERS_HH_
