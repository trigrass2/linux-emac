#ifndef SPI_CHAR_H_
#define SPI_CHAR_H_

#ifdef __KERNEL__
int spi_char_init(void);
int spi_char_create(struct spi_s *spi);
void spi_char_destroy(struct spi_s *spi);
void spi_char_remove(void);

#ifndef SPI_MAJOR
#define SPI_MAJOR 0
#endif

/* we use the max_devs define to register a region on init */
#define SPI_MAX_DEVS 15
#endif /*__KERNEL__*/

/* This header defines the ioctl commands */
#include <linux/class/rtdm/spi_rtdm.h>

#define CHAR_CLASS_SPI RTDM_CLASS_SPI

#endif /*SPI_CHAR_H_*/
