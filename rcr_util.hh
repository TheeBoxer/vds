#ifndef _RCR_UTIL_HH_
#define _RCR_UTIL_HH_

namespace rcr {
namespace util {

#undef abs // undefine forever

template <typename T> // todo: enableif signed realnumber
constexpr auto abs(const T&& x) -> T {
  return x < 0 ? -x : x;
}

} // namespace util
} // namespace rcr

#endif // _RCR_UTIL_HH_