#include "lib/cpp-mediator/include/cpp_mediator/mediator.hpp"
#include "src/rcr/sensors/vn/vectornav.hh"

#include <memory>

holden::mediator m{};
auto vn_handler = std::make_shared<rcr::sensors::vn::VectorNavHandler>();

int main() {
  m.register_handler(vn_handler);
  rcr::sensors::vn::PrintAllStatus request{};

for (;;) {
  m.send(request);
}

  return 0;
}