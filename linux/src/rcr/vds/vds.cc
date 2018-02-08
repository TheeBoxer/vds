#include <cpp_mediator/mediator.hpp>
#include <rcr/sensors/vn/vectornav.hh>
#include <rcr/beaglebone/io/spi/spi.hh>

#include <memory>

#include <stdio.h>

namespace rcr {
namespace vds {

namespace sdata {
  constexpr const char* const kFlightLogFilename = "/tmp/vds_flight_log";
  constexpr const char* const kFileIoMode = "a";
  holden::mediator m{};
  auto vn_handler = std::make_shared<sensors::vn::VectorNavHandler>(m);
  auto spi_handler = std::make_shared<rcr::beaglebone::io::spi::SpiHandler>(m);

  FILE* flight_log = NULL;
  sensors::vn::PrintAllStatus vn_request_all{ &flight_log, fprintf };
} // namespace sdata
} // namespace vds
} // namespace rcr

int main() {
  using namespace rcr::vds::sdata;
   
  m.register_handler(vn_handler);
  m.register_handler(spi_handler);
  m.send(rcr::beaglebone::io::spi::Open{});
while(1) {
  flight_log = NULL;
  flight_log = fopen(kFlightLogFilename, kFileIoMode);
  if (flight_log) {
    fprintf(flight_log, "yo dawg\n");
    m.send(vn_request_all);
    fclose(flight_log);
  } else {
    printf("fatal error: \"/tmp/vds_flight_log\" could not open");
  }
}

  return 0;
}
