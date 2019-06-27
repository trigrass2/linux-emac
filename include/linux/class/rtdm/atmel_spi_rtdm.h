#ifndef ATMEL_SPI_RTDM_H_
#define ATMEL_SPI_RTDM_H_

#include <linux/class/spi.h>
#include <linux/class/rtdm/spi_rtdm.h>

/**
 * platform data for defining an RT SPI device
 */
struct atmel_rt_spi_plat
{
	u8 num_devices; /**< number of devices on this bus */
	struct rt_spi_device *devices; /**< array of rt_spi_devices */
};

#endif /* ATMEL_SPI_RTDM_H_ */
