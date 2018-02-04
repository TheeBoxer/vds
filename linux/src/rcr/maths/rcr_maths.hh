#ifndef _RCR_MATHS_HH_
#define _RCR_MATHS_HH_

#include <type_traits>

namespace rcr {
namespace maths {

template <typename TInt, typename = std::enable_if_t<
  std::is_integral<TInt>::value>>
constexpr inline
TInt map(TInt x, TInt in_min, TInt in_max, TInt out_min, TInt out_max) {
  auto res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return res;
}

} // namespace maths
} // namespace rcr

#endif // _RCR_MATHS_HH_
