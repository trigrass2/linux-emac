/*
 * drivers/misc/classes/lsi2esc_devices/ad7766-gpio.c
 *
 * Copyright (C) 2011 EMAC.Inc <support@emacinc.com>
 */

#include <linux/class/lsi2esc/ad7766-gpio.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>

static struct spi_s *spi_dev;

struct work_struct ad7766_work;
struct workqueue_struct * ad7766_wq;
gpio_data sample; 
DEFINE_MUTEX(sample_ready);

spinlock_t ad7766_lock = SPIN_LOCK_UNLOCKED;

#define AD7766_DRDY AT91_PIN_PB31

/**
 * function to initialize the settings for the interface.
 */
static void ad7766_init(void)
{
	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);
}

/**
 * function to read the value of the latest sample from the ADC. 
 */
static gpio_data ad7766_data_read(struct gpio_s *gpio)
{
	gpio_data data;

	/* lock the sample mutex for synchronization. if the current
	 * sample has been read, this will cause the read to block
	 * until the next sample arrives and the mutex is unlocked.
	 */	
	mutex_lock(&sample_ready);

	spin_lock(&ad7766_lock);
	data = sample;
	spin_unlock(&ad7766_lock);

	return data;
}

/**
 * function to enable/disable read on interrupt
 */
static int ad7766_data_write(struct gpio_s *gpio, gpio_data data)
{
	if(data == 0) 
		disable_irq(AD7766_DRDY);
	else
		 enable_irq(AD7766_DRDY);

        return 0;
}


/*
 * ad7766_handler
 *
 * This function is called after an interrupt occurs.
 */
irqreturn_t ad7766_handler(int irq, void *dev_id)
{
	if(at91_get_gpio_value(AD7766_DRDY) == 0)
	{
		/* queue work for SPI transaction if ~DRDY = 0 */
		queue_work(ad7766_wq, &ad7766_work);
	}

        return IRQ_HANDLED;
}

static void do_ad7766_work(struct work_struct *dummy)
{
        u32 size;
        u8 mosi[3];
        u8 miso[3];

        size = 3;

        memset(miso, 0x0, sizeof(miso));
        spi_dev->xmit(spi_dev, mosi, miso, size);

	spin_lock(&ad7766_lock);
        sample = (miso[0] << 16) | (miso[1] << 8) | (miso[2]);
	spin_unlock(&ad7766_lock);

	/* mark the sample ready for reading. */
	if (mutex_is_locked(&sample_ready)) mutex_unlock(&sample_ready);
}

/**
 * function to register the class
 */
struct device *ad7766_gpio_class_create(struct spi_s *spi, const char *name)
{
	int result;
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	spi_dev = spi;

	ad7766_init();

        at91_set_gpio_input(AD7766_DRDY, 1);
        at91_set_deglitch(AD7766_DRDY, 1);

        INIT_WORK(&ad7766_work, do_ad7766_work);
        ad7766_wq = create_singlethread_workqueue(name);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_write = ad7766_data_write;
	gpio->index_write = gpio_index_write;
	gpio->index_read = gpio_index_read;
	gpio->data_read = ad7766_data_read;

	result = request_irq(AD7766_DRDY, ad7766_handler, IRQF_SHARED | IRQ_TYPE_EDGE_BOTH , name, (void *) &gpio);

        if(result) printk("irq request failed: %s\n", gpio->name);

	/* irq is disabled until gpio write command */
	disable_irq(AD7766_DRDY);

	printk("registering ad7766 GPIO interface: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

