#include <rcr/beaglebone/io/spi/spi.hh>

#include <BlackSPI/BlackSPI.h>

namespace rcr {
namespace beaglebone {
namespace io {
namespace spi {

namespace sdata {
  BlackLib::BlackSPI spi{ BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 8*1000*1000 };
} // namespace sdata

inline bool
SpiHandler::handle(const IsOpen& m) { return sdata::spi.isOpen(); }

inline bool
SpiHandler::handle(const Open& m) {
  return sdata::spi.isOpen()
    || sdata::spi.open(BlackLib::ReadWrite | BlackLib::NonBlock);
}

inline bool
SpiHandler::handle(const WriteRead& m) {
    sdata::spi.transfer(m.tx_bytes, m.rx_bytes, m.rx_bytes_size);
}

} // namespace spihandle
} // namespace io
} // namespace beaglebone
} // namespace rcr
