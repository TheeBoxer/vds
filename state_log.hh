#ifndef _RCR_VDS_STATELOG_HH_
#define _RCR_VDS_STATELOG_HH_

struct StateLog {
  unsigned long time;
  float alt;
  float vel;
  float leftVel;
  float rightVel;
  float accel;
  float rollAxisGrav;
  float yawAxisGrav;
  float pitchAxisGrav;
  float rollAxisLin;
  float yawAxisLin;
  float pitchAxisLin;
  float rollAxisGyro;
  float yawAxisGyro;
  float pitchAxisGyro;
  float roll;
  float yaw;
  float pitch;
  float alt_k;
  float vel_k;
  float accel_k;
  float vSPP;
  int16_t encPos;
  int16_t encPosCmd;
  bool limit_out;
  bool limit_in;
  int16_t encMax;
  int16_t encMin;
  int mtrSpdCmd;
};

#endif // _RCR_VDS_STATELOG_HH_
