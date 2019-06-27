#ifndef AD7766_GPIO_H_
#define AD7766_GPIO_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_AD7766
struct device *ad7766_gpio_class_create(struct spi_s *spi, const char *name);
#else
#define ad7766_gpio_class_create(s,n) {}
#endif

#endif /* AD7766_GPIO_H_ */

