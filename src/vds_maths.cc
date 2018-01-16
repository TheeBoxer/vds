#include "vds_maths.hh"

#include "vehicle.hh"

#include <cmath>
#include <type_traits>

namespace rcr {
namespace vds {
namespace maths {

template <typename Real>
constexpr typename std::enable_if<std::is_floating_point<Real>::value>::type
spp_velocity(Real alt, Real vel, const Vehicle& vehicle) {
  if (vel < vehicle.interVel)
    return velocity_h(vehicle.Cmin, alt, 0., vehicle.targetAlt);
  
  if (alt < vehicle.targetAlt)
    return velocity_h(vehicle.Cspp, alt, vehicle.interVel, vehicle.interAlt);
  
  return 0.;
}

template <typename Real>
constexpr typename std::enable_if<std::is_floating_point<Real>::value>::type
velocity_h(Real c, Real alt, Real v0, Real h0) {
  Real K1 = -1. / std::sqrt(c * G) * std::atan(v0 * std::sqrt(c / G));
  Real K2 = h0 - 1. / c * std::log(std::cos(std::sqrt(c * G) * K1));
  Real x = 1. - std::exp(-2. * c * (K2 - alt));
  Real sqrt_x = x > 0. ? std::sqrt(x) : 0.;
  Real res = std::exp(c * (K2 - alt)) * std::sqrt(G / c) * sqrt_x;
  return res;
}

} // namespace maths
} // namespace vds
} // namespace rcr
