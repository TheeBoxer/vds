#ifndef _RCR_VDS_VEHICLE_HH_
#define _RCR_VDS_VEHICLE_HH_

#include <string>

namespace rcr {
namespace vds {
  
struct Vehicle {
  std::string name;
  double dryMass;
  double propMass;
  double Cd_r;
  double Cd_b;
  double Ar;
  double Ab;
  int avgMotorThrust;
  int targetAlt;
  int interVel;
  int interAlt;
  double Cmin;
  double Cmax;
  double Cspp;
};

extern Vehicle vehicle;

} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_VEHICLE_HH_
