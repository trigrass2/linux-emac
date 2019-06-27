#ifndef SPI_CLASS_H_
#define SPI_CLASS_H_
#include <linux/device.h>
#include <linux/spi/spi.h>

#if defined(CONFIG_SPICLASS) || defined(CONFIG_SPICLASS_MODULE)

#define SPICL_CPHA	(1<<0)
#define SPICL_CPOL	(1<<1)

/* common bit settings -- should be mutually exclusive */
/* it would be possible to let 8 bit mode be 0x00 and let
 * this be the default, but this does not work well for the
 * LSI2ESC interface as the default controller setting is to
 * leave bits_per_word set to 0. Leave 8 bit as an explicit
 * setting. */
#define SPICL_EIGHTBIT		(1<<2)	/* 8 bit is default      */
#define SPICL_TENBIT		(1<<3)	/* 10 bits per transfer  */
#define SPICL_TWELVEBIT		(1<<4)	/* 12 bits per transfer  */
#define SPICL_SIXTEENBIT	(1<<5)	/* 16 bits per transfer  */
#define SPICL_NODMA			(1<<7)  /* disable DMA transfers */

typedef u8 spi_data;
typedef u32 spi_control;

/* control definitions for the tip function */
#define TIPOFF		0
#define TIPON		1
#define TIPSTATUS	2

/* defined in spi_rtdm.h */
struct spi_message_s;

/**********************
 * spi class structure
 */
typedef struct spi_s {
	const char *name;
	int subclass;
	spi_data *buf;
	int bsize;
	struct device *dev;
	/* if the SPI interface is used there needs to be a pointer to
	 * the associated spi_device struct used by the Linux SPI layer
	 */
#if defined(CONFIG_LSI2ESC) || defined(CONFIG_LSI2ESC_MODULE)
	struct spi_device *lsi;
#endif
	int (*tip) (struct spi_s *s, int ofs);	//declare transfer in process, for locking purposes
	int (*xmit) (struct spi_s *s, u8 *mosi, u8 *miso, int size);
	int (*confwrite) (struct spi_s *s, spi_control config);
	spi_control (*confread) (struct spi_s *s);
	int (*speedwrite) (struct spi_s *s, spi_control speed);
	spi_control (*speedread) (struct spi_s *s);
	/* method for creating a GPIO interface to this device */
	void *gpio_data; /* generic data for passing to the GPIO class */
	char *gpio_name; /* name of the gpio device to create */
	struct device *(*gpio_create) (struct spi_s *spi, const char *name);
	void (*gpio_delete)(struct spi_s *spi);
	struct device *gpio;
#ifdef CONFIG_SPICLASS_CHAR
	struct cdev *cdev;
#endif
	void *driver_data; /* generic data for passing to chip driver init */
	void (*driver_init) (struct spi_s *spi); /* if defined, initialize a chip driver on this device */
	int enable_rtdm;
	int disable_char;
} spi_t;

#define SPI_BASECLASNUM	0xC0
#define SPI_SUBCLASS 	(SPI_BASECLASNUM+0)

/***************************************************************************
 * initial class declaration, doesn't hurt to call it multiple times,
 * automatically checked during device instantiation 
 */
struct class *spi_class_declare(void);

/***************************************************************************
 * class instantiation
 */
struct device *spi_class_register_device(spi_t *s);
void spi_class_unregister_device(struct spi_s *s);
void spi_class_gpio_delete(struct spi_s *spi);

/***************************************************************************
 * atomic method wrappers
 * these should be used for all method calls to maintain synchronization across the
 * various interfaces
 */
int atomic_spi_xmit(struct spi_s *s, u8 * mosi, u8 * miso, int size);
int atomic_spi_conf_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_conf_read(struct spi_s *s);
int atomic_spi_speed_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_speed_read(struct spi_s *s);
int atomic_spi_tip_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_tip_read(struct spi_s *s);

#endif

#endif /*SPI_CLASS_H_ */
