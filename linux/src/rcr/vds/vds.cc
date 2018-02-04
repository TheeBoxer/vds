#include <cpp_mediator/mediator.hpp>
#include <rcr/sensors/vn/vectornav.hh>
#include <memory>

holden::mediator m{};
auto vn_handler = std::make_shared<rcr::sensors::vn::VectorNavHandler>(m);

int main() {
  m.register_handler(vn_handler);
  rcr::sensors::vn::PrintAllStatus request{};

//for (;;) {
  m.send(request);
//}

  return 0;
}