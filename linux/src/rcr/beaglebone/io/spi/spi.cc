#include <rcr/beaglebone/io/spi/spi.hh>

#include <bbc/spi.h>

namespace rcr {
namespace beaglebone {
namespace io {
namespace spi {

namespace sdata {
  bool is_open = false;
  spi_properties spi_props{0, spi0, 8, 0, 8*1000*1000, O_RDWR};
} // namespace sdata

inline bool
SpiHandler::handle(const IsOpen& m) { return sdata::is_open; }

inline bool
SpiHandler::handle(const Open& m) {
  return sdata::is_open
     || (sdata::is_open = spi_open(&sdata::spi_props));
}

inline bool
SpiHandler::handle(const WriteRead& m) {
  spi_transfer(&sdata::spi_props, m.tx_bytes, m.rx_bytes, m.length);
}

} // namespace spihandle
} // namespace io
} // namespace beaglebone
} // namespace rcr
