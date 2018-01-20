#include "rcr_util.hh"

namespace rcr {
namespace util {

bool
get_authorization(IoStreams& s) {
  clear_input(s);
  s.out << "yes or no? (y/n)\n";

  for (;;) {
    while (s.in.gcount() == 0) {}
    auto response = s.in.get();
    clear_input(s);

    if (is_yes(response)) return true;
    if (is_no(response))  return false;
    s.out << "Not a valid response; enter 'y' or 'n'.\n";
  }
}

} // namespace util
} // namespace rcr
