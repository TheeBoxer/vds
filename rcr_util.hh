#ifndef _RCR_UTIL_HH_
#define _RCR_UTIL_HH_

#include <istream>
#include <ostream>

namespace rcr {
namespace util {

struct IoStreams {
  // stream dedicated to the expression of errors
  std::ostream& err;

  // input stream
  std::istream& in;

  // output stream
  std::ostream& out;
};

// Flush the input stream.
void
clear_input(IoStreams& s) { s.in.clear(); }

// Get 'yes' or 'no' response from user.
bool
get_authorization(IoStreams& s);

template<typename T, class = typename std::enable_if<std::is_integral<T>::value>::type>
constexpr bool
is_yes(T in) { return in == 'y' || in == 'Y'; }

template<typename T, class = typename std::enable_if<std::is_integral<T>::value>::type>
constexpr bool
is_no(T in) { return in == 'n' || in == 'N'; }

} // namespace util
} // namespace rcr

#endif // _RCR_UTIL_HH_
