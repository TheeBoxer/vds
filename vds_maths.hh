#ifndef _RCR_VDS_MATHS_HH_
#define _RCR_VDS_MATHS_HH_

namespace rcr {
namespace vds {
namespace maths {

// Calculate velocity as a function of altitude.
// Author: Jacob & Ben
double
velocity_h(double c, double alt, double v0, double h0);

// Velocity of the Set Point Path (SPP). Returns a velocity at which the 
// vehicle should be moving for a given altitude argument. The SPP is also 
// piecewise.
// Author: Jacob & Ben
double vSPP(double alt, double vel);

} // namespace maths
} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_MATHS_HH_
