#ifndef LSI2ESC_GPIO_H_
#define LSI2ESC_GPIO_H_

#include <linux/class/spi_interface.h>
#include <linux/class/gpio.h>
#include <linux/class/lsi2esc/ad7766-gpio.h>
#include <linux/class/lsi2esc/ads1158-gpio.h>
#include <linux/class/lsi2esc/mcp3208-gpio.h>
#include <linux/class/lsi2esc/mcp4922-gpio.h>
#include <linux/class/lsi2esc/rdac-aio.h>
#include <linux/class/lsi2esc/rims-8051.h>

typedef struct device *(*gpio_func_ptr) (struct spi_s *, const char *);

struct gpio_class_create_entry {
	char * name;
	gpio_func_ptr func;
};

struct gpio_class_create_entry gpio_class_create_list[] = {
#ifdef CONFIG_LSI2ESC_AD7766
	{ "ad7766-gpio", ad7766_gpio_class_create },
#endif
#ifdef CONFIG_LSI2ESC_ADS1158
	{ "ads1158-gpio", ads1158_gpio_class_create },
#endif
#ifdef CONFIG_LSI2ESC_MCP3208
	{ "mcp3208-gpio", mcp3208_gpio_class_create },
#endif
#ifdef CONFIG_LSI2ESC_MCP4922
	{ "mcp4922-gpio", mcp4922_gpio_class_create },
#endif
#if defined(CONFIG_LSI2ESC_RDAC_AIO) || defined(CONFIG_LSI2ESC_RDAC_AIO_MODULE)
	{ "rdac-aio-gpio", rdac_aio_create },
#endif
#if defined(CONFIG_LSI2ESC_RDAC_AIO_EA) || defined(CONFIG_LSI2ESC_RDAC_AIO_EA_MODULE)
	{ "rdac-aio-gpio", rdac_aio_create },
#endif
#ifdef CONFIG_LSI2ESC_RIMS_8051
	{ "rims8051-gpio", rims8051_gpio_class_create },
#endif
	{ NULL },
};

gpio_func_ptr lsi2esc_find_gpio_create(char * name) {
	int i = 0;
	
	if(name == NULL) return NULL;

	while(gpio_class_create_list[i].name)
	{
		if(strcmp(name, gpio_class_create_list[i].name)==0)
			return gpio_class_create_list[i].func;
		i++;
	}
		
	return NULL;
}
#endif /* LSI2ESC_GPIO_H_ */

