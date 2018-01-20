#ifndef _RCR_VDS_MATHS_HH_
#define _RCR_VDS_MATHS_HH_

namespace rcr {
namespace vds {
namespace maths {

// Calculate velocity as a function of altitude.
// Author: Jacob & Ben
template <typename Real>
constexpr typename std::enable_if<std::is_floating_point<Real>::value>::type
velocity_h(Real c, Real alt, Real v0, Real h0);

// Velocity of the Set Point Path (SPP). Returns a velocity at which the 
// vehicle should be moving for a given altitude argument. The SPP is also 
// piecewise.
// Author: Jacob & Ben
template <typename Real>
constexpr typename std::enable_if<std::is_floating_point<Real>::value>::type
spp_velocity(Real alt, Real vel, const Vehicle& vehicle);

} // namespace maths
} // namespace vds
} // namespace rcr

#endif // _RCR_VDS_MATHS_HH_
