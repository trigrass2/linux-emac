/**
 * drivers/misc/classes/rtdm/ad7766_rtdm.c
 *
 * Copyright (C) 2011, EMAC, Inc. <support@emacinc.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <rtdm/rtdm_driver.h>
#include <rtdm/ad7766.h>
#include <linux/class/spi.h>
#include <linux/class/rtdm/spi_rtdm.h>
#include <native/heap.h>
#include <mach/gpio.h>

#define DEVICE_NAME		"ad7766-rtdm"

typedef struct sample_s {
	nanosecs_abs_t 	sample_time;
	u32 		sample_data;
} sample_t;

typedef struct buffer_s {
	sample_t * data;
	int size;
	int readptr;
	int writeptr;
} buffer_t;

buffer_t buffer;
bool buffer_full = 0;

RT_HEAP heap_desc;

#define HEAP_SIZE 	(sizeof(sample_t) * 5 * 32 * 1024)
#define MAX_SAMPLES 	(5 * 32 * 1000)
#define HEAP_MODE 0

#define AD7766_DRDY	AT91_PIN_PB31

rtdm_irq_t irq_handle;

struct rt_spi_device *rtspi;

static int ad7766_rtdm_open(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo, int oflags)
{
	return 0;
}

static int ad7766_rtdm_close(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo)
{
	return 0;
}

/**
 * Reads buffered samples from the ADC when sampling is complete.
 */
static ssize_t ad7766_rtdm_read_rt(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo, void *buf, size_t nbyte)
{
	int ret;

	if (!buffer_full)
		return -EAGAIN;

	if (((buffer.size - buffer.readptr) * sizeof(sample_t)) < nbyte) 
	{
		nbyte = (buffer.size - buffer.readptr) * sizeof(sample_t);
	}

	ret = rtdm_safe_copy_to_user(uinfo, buf, (void *) &buffer.data[buffer.readptr++], nbyte);

	if (ret)
		return ret;

	return nbyte;
}

/**
 * A write will set the sample buffer size and start a conversion period.
 */
static ssize_t ad7766_rtdm_write_rt(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo, const void *buf, size_t nbyte)
{
	int ret;

	ret = rtdm_safe_copy_from_user(uinfo, (void *) &buffer.size, buf, sizeof(int));
	
	if (ret)
		return ret;

	if(buffer.size > MAX_SAMPLES) buffer.size = MAX_SAMPLES;

	buffer.readptr = 0;
	buffer.writeptr = 0;
	buffer_full = 0;
	
	rtdm_irq_enable(&irq_handle);

	return buffer.size;
}

/**
 * Performs an SPI transaction when triggered by the ADCs DRDY line.
 */
static int ad7766_handler(rtdm_irq_t *irq_context)
{
	u8 mosi[3];
	u8 miso[3];
	int ret = 0;

	struct rt_spi_message msg;
	struct rt_spi_transfer xfer = {
		.tx_buf = mosi,
		.rx_buf = miso,
		.len = 3,
		.delay_usecs = 0,
	};

	if(buffer.writeptr == buffer.size) return RTDM_IRQ_HANDLED;

	if(at91_get_gpio_value(AD7766_DRDY) == 0)
	{
		rt_spi_message_init(&msg);
		rt_spi_message_add_tail(&xfer, &msg);
		msg.dma_disabled = 1;

		ret = rtspi->sync(&rtspi->spi, &msg);

		if(ret) 
			printk(KERN_ERR "AD7766 SPI transfer failed. (ret = %d)\n",ret);

		buffer.data[buffer.writeptr].sample_data = (miso[0] << 16) | (miso[1] << 8) | (miso[2]);
		buffer.data[buffer.writeptr].sample_time = rtdm_clock_read();

		buffer.writeptr++;

		if(buffer.writeptr == buffer.size)
		{
			rtdm_irq_disable(&irq_handle);
			buffer_full = 1;
		}
	}

	return RTDM_IRQ_HANDLED;
}

static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,
	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = 0,
	.device_name = DEVICE_NAME,

	.open_nrt = ad7766_rtdm_open,

	.ops = {
		.close_nrt = ad7766_rtdm_close,
		.read_rt   = ad7766_rtdm_read_rt,
		.write_rt  = ad7766_rtdm_write_rt,
	},

	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = RTDM_SUBCLASS_GENERIC,
	.profile_version = 1,
	.driver_name = "AD7766",
	.driver_version = RTDM_DRIVER_VER(1, 0, 0),
	.peripheral_name = "AD7766 RTDM",
	.provider_name = "EMAC, Inc.",
	.proc_name = device.device_name,
};

/**
 * Performs initialization of the heap, interrupt, and SPI interface.
 */
void init_ad7766_spi(struct spi_s *spi)
{
	int err;
	struct ad7766_config *config;

        err = rt_heap_create(&heap_desc, "ad7766-heap", HEAP_SIZE, HEAP_MODE);
	if(err) 
	{
		printk(KERN_ERR "rt_heap_create failed\n");
		return;
	}

        err = rt_heap_alloc(&heap_desc, MAX_SAMPLES*sizeof(sample_t), TM_NONBLOCK, (void**)&buffer.data);
	if(err) 
	{
		printk(KERN_ERR "rt_heap_alloc failed\n");
		return;
	}

	rtspi = spi_to_rt(spi);
	config = rtspi->driver_data;

	config->irq_init(config->irq);

	err = rtdm_irq_request(&irq_handle, config->irq, ad7766_handler, 0, "ad7766-rtdm", NULL);
	if(err) 
		printk(KERN_ERR "ad7766-rtdm irq request failed\n");

	rtdm_irq_disable(&irq_handle);
}

/** 
 * Registers the rtdm device.
 */
static int __init ad7766_init(void)
{
	int err;

	err = rtdm_dev_register(&device);

	if(err) 
		printk(KERN_ERR "Error registering RTDM device.\n");

	return err;
}

/**
 * Unregisters the rtdm device.
 */
void ad7766_exit(void)
{
	int err;

	err = rtdm_dev_unregister(&device, 0);

	if(err)
		printk(KERN_ERR "Error unregistering RTDM device.\n");
}

module_init(ad7766_init);
module_exit(ad7766_exit);

MODULE_AUTHOR("EMAC, Inc. <support@emacinc.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AD7766 RTDM Driver");

