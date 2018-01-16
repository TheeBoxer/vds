#ifndef _RCR_VDS_VEHICLESTATE_HH_
#define _RCR_VDS_VEHICLESTATE_HH_

struct VehicleState {
  // acceleration (m/s^2)
  // TODO: magnitude? or what
  float accel;

  // altitude (m)
  float alt;

  // time relative to the present moment. (used in calculateVelocity()) (s)
  // TODO: what?
  float buff_t;
  
  // time since the program began (s * 10^-5)
  // TODO: is that unit correct?
  unsigned long time;

  // velocity derived from calculateVelocity() (m/s)
  // TODO: what is this? verticle?
  float vel;
};

#endif // _RCR_VDS_VEHICLESTATE_HH_
