#include <rcr/beaglebone/io/spi/spi.hh>

#include <fcntl.h>
#include <rcr/linux/spi/spi.hh>
#include <rcr/util/publish.h>

namespace rcr {
namespace beaglebone {
namespace io {
namespace spi {

namespace sdata {
  bool is_open = false;
  linux::spi::spi_properties spi_props{0, linux::spi::spi0, 8, 1, 5000, O_RDWR};
} // namespace sdata

bool
SpiHandler::handle(const IsOpen& m) { return sdata::is_open; }

inline bool
SpiHandler::handle(const Open& m) {
  return sdata::is_open
     || (sdata::is_open = rcr::linux::spi::spi_open(&sdata::spi_props));
}

inline bool
SpiHandler::handle(const WriteRead& m) {
  PUBLISH(15, "sending: ");
  DO_AND_PUBLISH(for(auto i=0; i<m.length; ++i) PUBLISH(1, "%x ", m.tx_bytes[i]), 15, "");
  
  bool ok = rcr::linux::spi::spi_transfer(
    &sdata::spi_props, m.tx_bytes, m.rx_bytes, m.length) == 0;
  
  PUBLISH(15, "received: ");
  DO_AND_PUBLISH(for(auto i=0; i<m.length; ++i) PUBLISH(1, "%x ", m.rx_bytes[i]), 15, "");
  
  return ok;
}

} // namespace spi
} // namespace io
} // namespace beaglebone
} // namespace rcr
