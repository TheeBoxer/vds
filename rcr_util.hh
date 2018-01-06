#ifndef _RCR_UTIL_HH_
#define _RCR_UTIL_HH_

namespace rcr {
namespace util {

void
clear_input(HardwareSerial& s) { while (s.available()) s.read(); }

bool
get_authorization(HardwareSerial& out);

template <typename T>
constexpr inline bool
is_yes(T in) { return in == 'y' || in == 'Y'; }

template <typename T>
constexpr inline bool
is_no(T in) { return in == 'n' || in == 'N'; }

} // namespace util
} // namespace rcr

#endif // _RCR_UTIL_HH_