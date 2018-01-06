#include "rcr_util.hh"

namespace rcr {
namespace util {

bool get_authorization(HardwareSerial & out) {
  clear_input(out);
  out.println("yes or no? (y/n)");

  for (;;) {
    while (!out.available()) {}
    auto response = out.read();
    clear_input(out);

    if (is_yes(response)) return true;
    if (is_no(response))  return false;
    out.println("Not a valid response; enter 'y' or 'n'.");
  }
}

} // namespace util
} // namespace rcr
