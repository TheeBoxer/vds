#ifndef _RCR_BEAGLEBONE_IO_SPI_SPI_HH_
#define _RCR_BEAGLEBONE_IO_SPI_SPI_HH_

#include <cpp_mediator/mediator.hpp>
#include <BlackSPI/BlackSPI.h>

#include <cstdint>

namespace rcr {
namespace beaglebone {
namespace io {
namespace spi {

class SpiHandler;
struct IsOpen    : holden::request<bool, SpiHandler> {};
struct Open      : holden::request<bool, SpiHandler> {};
struct WriteRead : holden::request<bool, SpiHandler> {
  WriteRead(uint8_t* tx_bytes, uint8_t* rx_bytes, size_t rx_bytes_size)
    : tx_bytes(tx_bytes), rx_bytes(rx_bytes)
    , rx_bytes_size(rx_bytes_size) {}
  uint8_t* tx_bytes;
  uint8_t* rx_bytes;
  size_t rx_bytes_size;
};

class SpiHandler
  : public holden::request_handler<IsOpen>
  , public holden::request_handler<Open>
  , public holden::request_handler<WriteRead> {
 public:
  SpiHandler(holden::mediator& m) : mediator_(m) {}
  bool handle(const IsOpen& m);
  bool handle(const Open& m);
  bool handle(const WriteRead& m);

 protected:
  holden::mediator& mediator_;
};

} // namespace spi
} // namespace io
} // namespace beaglebone
} // namespace rcr

#endif // _RCR_BEAGLEBONE_IO_SPI_SPI_HH_
