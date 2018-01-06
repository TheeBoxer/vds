#include "vds_maths.hh"

namespace rcr {
namespace vds {
namespace maths {

//template <typename T>
//typename <std::enable_if<std::is_floating_point<T>::value>>
//constexpr T
//velocity_h(T c, T alt, T v0, T h0) {
//  auto K1 = -1. / sqrt(c * G) * atan(v0 * sqrt(c / G));
//  auto K2 = h0 - 1. / c * log(cos(sqrt(c * G) * K1));
//  auto x = 1. - exp(-2 * c * (K2 - alt));
//  auto sqrt_x = x > 0. ? sqrt(x) : 0.;
//  auto res = exp(c * (K2 - alt)) * sqrt(G / c) * sqrt_x;
//  return res;
//}

double vSPP(double alt, double vel, const Rocket& vehicle) {
  double returnVal = 0.;
  double x = 1. - exp(-2. * vehicle.Cmin * (vehicle.targetAlt - alt));
  if (x < 0.) x = 0;

  if (vel < vehicle.interVel) {
    returnVal = velocity_h(vehicle.Cmin, alt, 0, vehicle.targetAlt);
  }
  else if (vel >= vehicle.interVel) {
    if (alt < vehicle.targetAlt) {
      returnVal = velocity_h(vehicle.Cspp, alt, vehicle.interVel, vehicle.interAlt);
    }
    else {
      returnVal = 0;
    }
  }
  else {
    returnVal = 0;
  }

  return returnVal;
}

} // namespace maths
} // namespace vds
} // namespace rcr
