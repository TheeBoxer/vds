#include <rcr/linux/spi/spi.hh>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spidev.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

namespace rcr {
namespace linux {
namespace spi {

uint8_t spi_open(spi_properties *spi) {
    char filename[20];
    sprintf(filename, "/dev/spidev1.%d", spi->spi_id);
    spi->fd = open(filename, spi->flags);
    if (spi->fd < 0) {
		perror("could not open spi.");
		return -1;
    }
    syslog(LOG_INFO,"FD: %i", spi->fd);
    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &spi->mode)==-1){
       perror("SPI: Can't set SPI mode.");
       return -1;
    }
    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &spi->bits_per_word)==-1){
       perror("SPI: Can't set bits per word.");
       return -1;
    }
    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi->speed)==-1){
       perror("SPI: Can't set max speed HZ");
       return -1;
    }

    // Check that the properties have been set
    syslog (LOG_INFO,"SPI fd is: %d\n", spi->fd);
    syslog (LOG_INFO,"SPI Mode is: %d\n", spi->mode);
    syslog (LOG_INFO,"SPI Bits is: %d\n", spi->bits_per_word);
    syslog (LOG_INFO,"SPI Speed is: %d\n", spi->speed);
    return 0;
}

uint8_t spi_close(spi_properties *spi) {
	syslog (LOG_INFO, "spi close - spi:%d", spi->fd);
    close(spi->fd);
    return 0;
}

uint8_t spi_transfer(spi_properties *spi, unsigned char tx[], unsigned char rx[], int length) {
  struct spi_ioc_transfer transfer{};
  transfer.tx_buf = (unsigned long)tx;
  transfer.rx_buf = (unsigned long)rx;
  transfer.len = length;
  transfer.delay_usecs = 0;
  transfer.speed_hz = spi->speed;
  transfer.bits_per_word = spi->bits_per_word;
  // send the SPI message (all of the above fields, inc. buffers)
  int status = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &transfer);
  if (status < 0) {
    perror("SPI: SPI_IOC_MESSAGE Failed");
    return -1;
  }
  return 0; //status;
}

} // namespace spi
} // namespace linux
} // namespace rcr
