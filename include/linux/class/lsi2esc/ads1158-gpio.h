#ifndef ADS1158_GPIO_H_
#define ADS1158_GPIO_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_ADS1158
struct device *ads1158_gpio_class_create(struct spi_s *spi, const char *name);
#else
#define ads1158_gpio_class_create(s,n) {}
#endif

#endif /* AD7766_GPIO_H_ */

