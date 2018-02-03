#ifndef _RCR_UTIL_HH_
#define _RCR_UTIL_HH_

#include <chrono>
#include <thread>

namespace rcr {
namespace util {

template<typename T, class = typename std::enable_if<
  std::is_integral<T>::value>::type>
constexpr bool
is_yes(T in) { return in == 'y' || in == 'Y'; }

template<typename T, class = typename std::enable_if<
  std::is_integral<T>::value>::type>
constexpr bool
is_no(T in) { return in == 'n' || in == 'N'; }

template <typename Tick, typename Period>
inline void
sleep_for(const std::chrono::duration<Tick, Period>& d) {
  std::this_thread::sleep_for(d);
}

} // namespace util
} // namespace rcr

#endif // _RCR_UTIL_HH_
