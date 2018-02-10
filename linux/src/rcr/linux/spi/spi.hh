#ifndef _RCR_LINUX_SPI_SPI_HH_
#define _RCR_LINUX_SPI_SPI_HH_

#include <stdint.h>

#undef linux

namespace rcr {
namespace linux {
namespace spi {

typedef enum {
	spi0 = 0, spi1 = 1
} spi;

struct spi_properties {
	spi_properties(
		int fd,
		spi spi_id,
		uint8_t bits_per_word,
		uint8_t mode,
		uint32_t speed,
		uint8_t flags)
	  : fd(fd), spi_id(spi_id), bits_per_word(bits_per_word), mode(mode)
	  , speed(speed), flags(flags) {}
	int fd;
	spi spi_id;
	uint8_t bits_per_word; /*!< @brief is used to hold the bits per word size of SPI */
	uint8_t mode; /*!< @brief is used to hold the mode of SPI */
	uint32_t speed; /*!< @brief is used to hold the speed of SPI */
	uint8_t flags;
};

extern uint8_t spi_open(spi_properties* spi);
extern uint8_t spi_send(spi_properties *spi, unsigned char tx[], int length);
extern uint8_t spi_transfer(spi_properties *spi, unsigned char tx[], unsigned char rx[], int length);
extern uint8_t spi_close(spi_properties *spi);

} // namespace spi
} // namespace linux
} // namespace rcr

#endif // _RCR_LINUX_SPI_SPI_HH_
