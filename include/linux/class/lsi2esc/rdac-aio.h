#ifndef RDAC_AIO_H_
#define RDAC_AIO_H_

#include <linux/class/spi_interface.h>
#include <linux/class/gpio.h>

#if defined(CONFIG_LSI2ESC_RDAC_AIO) || defined(CONFIG_LSI2ESC_RDAC_AIO_MODULE)
struct device *rdac_aio_create(struct spi_s *spi, const char *name);
void rdac_aio_delete(struct spi_s *spi);

gpio_data aio_analog_handler(struct gpio_s *gpio);
void aio_analog_irq_config(struct gpio_s *gpio);
void aio_analog_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify);
// can't get an || to work in the ifdef, so repeating
#elif defined(CONFIG_LSI2ESC_RDAC_AIO_EA) || defined(CONFIG_LSI2ESC_RDAC_AIO_EA_MODULE)
struct device *rdac_aio_create(struct spi_s *spi, const char *name);
void rdac_aio_delete(struct spi_s *spi);

gpio_data aio_analog_handler(struct gpio_s *gpio);
void aio_analog_irq_config(struct gpio_s *gpio);
void aio_analog_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify);

#else
#define rdac_aio_create(s,n) {}
#endif


#endif /* MCP3208_GPIO_H_ */

