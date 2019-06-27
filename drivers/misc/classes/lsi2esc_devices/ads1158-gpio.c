/*
 * drivers/misc/classes/lsi2esc_devices/ads1158-gpio.c
 *
 * Copyright (C) 2011 EMAC.Inc <support@emacinc.com>
 */

#include <linux/class/lsi2esc/ads1158-gpio.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/delay.h>

#define CONFIG0	0x00
#define CONFIG1	0x01
#define MUXSCH	0x02
#define MUXDIF	0x03
#define MUXSG0	0x04
#define MUXSG1	0x05
#define SYSRED	0x06
#define GPIOC	0x07
#define GPIOD	0x08
#define ID	0x09

#define CHAN_READ		(0x0 << 5)
#define CHAN_READ_REGISTER	(0x1 << 5)
#define REGISTER_READ		(0x2 << 5)
#define REGISTER_WRITE		(0x3 << 5)
#define PULSE_CONVERT		(0x4 << 5)
#define RESET_COMMAND		(0x6 << 5)
#define MUL_COMMAND		(0x1 << 4)

/**
 * function to initialize the settings for the interface.
 */
static void ads1158_init(struct spi_s * spi_dev)
{
	u32 size;
	u8 mosi[4];
	u8 miso[4];

	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);

	size = 4;
	mosi[0] = REGISTER_WRITE | MUXDIF | MUL_COMMAND;
	mosi[1] = 0x01;
	mosi[2] = 0x00;
	mosi[3] = 0x00;

	spi_dev->xmit(spi_dev, mosi, miso, size);

	size = 3;
	mosi[0] = REGISTER_WRITE | CONFIG0 | MUL_COMMAND;
	mosi[1] = 0x1A; /* enable bypass CONFIG0 */
	mosi[2] = 0x03; /* disable idle mode CONFIG1 */

	spi_dev->xmit(spi_dev, mosi, miso, size);
}

/**
 * function to read the value of the latest sample from the ADC. 
 */
static gpio_data ads1158_data_read(struct gpio_s *gpio)
{
	u32 size;
	u8 mosi[4];
	u8 miso[4];
	gpio_data sample;
	struct spi_s *spi_dev = (struct spi_s *) gpio->driver_data;

	size = 1;
	mosi[0] = PULSE_CONVERT;

	spi_dev->xmit(spi_dev, mosi, miso, size);

	udelay(100);

	size = 3;
	mosi[0] = CHAN_READ;

	spi_dev->xmit(spi_dev, mosi, miso, size);       

	sample = miso[1];
	sample = miso[2] | (sample << 8);

	return sample;
}

static int ads1158_data_write(struct gpio_s *gpio, gpio_data data)
{
	return 0;
}

/**
 * function to determine the current analog channel being read.
 */
static gpio_data ads1158_index_read(struct gpio_s *gpio)
{
	return gpio->index;
}

/**
 * function to set the channel for the next analog reading
 */
static int ads1158_index_write(struct gpio_s *gpio, gpio_data index)
{
	u32 size;
	u8 mosi[5];
	u8 miso[5];
	struct spi_s *spi_dev = (struct spi_s *) gpio->driver_data;

	size = 5;
	mosi[0] = REGISTER_WRITE | MUXDIF | MUL_COMMAND;
	mosi[1] = (1 << index) & 0xFF;
	mosi[2] = ((1 << index) >> 8) & 0xFF;
	mosi[3] = ((1 << index) >> 16) & 0xFF;
	mosi[4] = ((1 << index) >> 24) & 0xFF;

	spi_dev->xmit(spi_dev, mosi, miso, size);

	gpio->index = index;

	return 0;	
}


/**
 * function to register the class
 */
struct device *ads1158_gpio_class_create(struct spi_s *spi, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);

	ads1158_init(spi);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_write = ads1158_data_write;
	gpio->index_write = ads1158_index_write;
	gpio->index_read = ads1158_index_read;
	gpio->data_read = ads1158_data_read;
	gpio->driver_data = (void *) spi;
	gpio->range = 24;

	printk("registering ads1158 GPIO interface: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

