/**
 * include/linux/class/rtdm/spi_rtdm.h
 *
 * This file provides the kernel-level interface for the implementation
 * of RTDM SPI devices using the EMAC SPI Class.
 *
 * Some definitions adapted from the standard Linux SPI API in
 * include/linux/spi/spi.h Copyright 2005 David Brownell
 *
 * Copyright (C) 2010 EMAC, Inc. <support@emacinc.com>
 * Code copied from spi.h Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#ifndef SPI_RTDM_H_
#define SPI_RTDM_H_

#include <linux/ioctl.h>

/* arbitrary assignment */
#define RTDM_CLASS_SPI 0x90

#ifdef __KERNEL__

int rt_spi_device_create(struct spi_s *spi);

struct rt_spi_message;
struct spi_s;


/**
 * struct defining an SPI slave device for use with this driver
 * @bus_num: the bus number associated with this device
 * @cs_pin: the processor pin number to use for the chip select
 * @cs_num: the device id (i.e. CS 0, 1, 2) on the bus
 * @spi: SPI Class device associated with this slave
 * @mode: SPI mode information bitmask
 * @max_speed_hz: maximum clock speed allowed in Hz
 *   (may be changed through speedwrite function)
 * @driver_data: data used for implementing a chip-level driver
 * @driver_init: driver initialization function, if implemented
 * @sync: RT SPI transmit function callback
 * @controller_data: controller-specific data
 */
struct rt_spi_device {
	u8 bus_num;
	int cs_pin;
	int cs_num;
	struct spi_s spi;
	u8 mode;
	u32 max_speed_hz;
	int (*sync) (struct spi_s *spi, struct rt_spi_message *msg);
	void *driver_data;
	void (*driver_init)(struct spi_s *spi);
	void *controller_data;
};

/** Derive the rt_spi_device parent of a struct spi_s */
#define spi_to_rt(s) container_of(s, struct rt_spi_device, spi)

/** return 1 if DMA is disabled for the device, 0 otherwise */
static inline unsigned spi_dma_disabled(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);
	return rt->mode & SPICL_NODMA;
}

/**
 * structure defining one SPI transfer. Based heavily on the spi_transfer
 * struct in the standard Linux SPI API.
 * @tx_buf: data to be written (dma-safe memory), or NULL
 * @rx_buf: data to be read (dma-safe memory), or NULL
 * @tx_dma: DMA address of tx_buf, if @spi_message.is_dma_mapped
 * @rx_dma: DMA address of rx_buf, if @spi_message.is_dma_mapped
 * @len: size of rx and tx buffers (in bytes)
 * @cs_change: affects chipselect after this transfer completes
 * @delay_usecs: microseconds to delay after this transfer before
 *   (optionally) changing the chip select status followed by starting
 *   the next transfer or completing the current message. Note: this
 *   feature is not supported by all drivers.
 * @transfer_list: transfers are sequenced through @rt_spi_message.transfers
 *
 * If the transmit buffer is null, zeroes will be shifted out
 * while filling @rx_buf.  If the receive buffer is null, the data
 * shifted in will be discarded.  Only "len" bytes shift out (or in).
 * It's an error to try to shift out a partial word.  (For example, by
 * shifting out three bytes with word size of sixteen or twenty bits;
 * the former uses two bytes per word, the latter uses four bytes.)
 *
 * All SPI transfers start with the relevant chipselect active.  Normally
 * it stays selected until after the last transfer in a message.  Drivers
 * can affect the chipselect signal using cs_change.
 *
 * (i) If the transfer isn't the last one in the message, this flag is
 * used to make the chipselect briefly go inactive in the middle of the
 * message.  Toggling chipselect in this way may be needed to terminate
 * a chip command, letting a single spi_message perform all of group of
 * chip transactions together.
 *
 * (ii) When the transfer is the last one in the message, the chip may
 * stay selected until the next transfer.  On multi-device SPI busses
 * with nothing blocking messages going to other devices, this is just
 * a performance hint; starting a message to another device deselects
 * this one.  But in other cases, this can be used to ensure correctness.
 * Some devices need protocol transactions to be built from a series of
 * spi_message submissions, where the content of one message is determined
 * by the results of previous messages and where the whole transaction
 * ends when the chipselect goes inactive.
 *
 */
struct rt_spi_transfer {
	void *tx_buf;
	void *rx_buf;
	unsigned len;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	u16 delay_usecs;
	unsigned cs_change:1;
	struct list_head transfer_list;
};

/**
 * struct defining an SPI message. based heavily on the spi_message
 * struct in the standard Linux SPI API.
 * @transfers: list of transfer segments in the transaction
 * @spi: SPI Class device to which the transaction is queued
 * @is_dma_mapped: if true, the caller provided both DMA and virtual
 *   addresses for each transfer buffer
 * @dma_disabled: if true, DMA should be disabled for this message if
 *   supported by the controller driver
 * @complete: function pointer called to signal completion of the message
 * @context: used as the argument for the complete function
 * @actual_length: the total number of bytes that were transferred in all
 *   successful segments
 * @status: 0 for success, else negative errno
 * @queue: driver-specific linked list
 * @state: driver-specific data
 *
 * Note: The complete() function can be called from interrupt handler
 *   context with spinlocks held. Must be atomic and RTDM interrupt
 *   safe.
 */
struct rt_spi_message {
	struct list_head transfers;
	struct spi_s *spi;
	unsigned is_dma_mapped:1;
	u8 dma_disabled;
	void (*complete)(void *context);
	void *context;
	u32 actual_length;
	int status;
	struct list_head queue;
	void *state;
};

/**
 * function to initialize an rt_spi_message. Adapted from spi.h
 * in the standard Linux SPI API.
 */
static inline void rt_spi_message_init(struct rt_spi_message *m)
{
	memset(m, 0, sizeof(struct rt_spi_message));
	INIT_LIST_HEAD(&m->transfers);
}


/**
 * function to add an rt_spi_transfer to a message. Adapted from spi.h
 * in the standard Linux SPI API.
 */
static inline void
rt_spi_message_add_tail(struct rt_spi_transfer *t, struct rt_spi_message *m)
{
	list_add_tail(&t->transfer_list, &m->transfers);
}

#endif /*__KERNEL__ */

/**
 * structure for defining an SPI transfer as passed to
 * ioctl from userspace
 * @mosi: master-out-slave-in buffer
 * @miso: master-in-slave-out buffer
 * @size: size of the data transfer
 *
 * Note: Both mosi and miso can be defined or NULL. If they are defined
 * they each must be at least size bytes in length. If mosi is NULL
 * 0xFF bytes are transmitted for the duration of the transfer.
 * If miso is NULL received bytes are discarded.
 */
typedef struct spi_transfer_s {
	spi_data *mosi;
	spi_data *miso;
	ssize_t size;
} spi_transfer_t;

/**
 * structure for defining an SPI message consisting of two
 * spi_transfer_t objects as passed via ioctl from userspace
 * @xfer_1: first transfer
 * @xfer_2: second transfer
 * @usec_between_xfer: microseconds delay between each transfer
 *   note that this is only valid for some configurations
 *   depending on the controller driver implementation
 * @cs_change: if true, the chip select will be toggled between each transfer
 */
typedef struct spi_message_s {
	struct spi_transfer_s xfer_1;
	struct spi_transfer_s xfer_2;
	u8 usec_between_xfer;
	u8 cs_change;
} spi_message_t;

/* ioctl commands */
#define CONFREAD		_IOR(RTDM_CLASS_SPI,0,spi_control)
#define CONFWRITE		_IOW(RTDM_CLASS_SPI,0,spi_control)
#define SPEEDREAD		_IOR(RTDM_CLASS_SPI,1,spi_control)
#define SPEEDWRITE		_IOW(RTDM_CLASS_SPI,1,spi_control)
#define TIPREAD			_IOR(RTDM_CLASS_SPI,2,spi_control)
#define TIPWRITE		_IOW(RTDM_CLASS_SPI,2,spi_control)
#define XMIT			_IOW(RTDM_CLASS_SPI,3,spi_transfer_t)
#define XMITMSG			_IOW(RTDM_CLASS_SPI,4,spi_message_t)

#ifdef __KERNEL__

/**
 * function to convert from a spi_tranfser_t to a rt_spi_transfer
 */
static inline void
spi_transfer_to_rt(struct spi_transfer_s *xfer, struct rt_spi_transfer *rt_xfer)
{
	rt_xfer->tx_buf = xfer->mosi;
	rt_xfer->rx_buf = xfer->miso;
	rt_xfer->len = xfer->size;
}

#endif /*__KERNEL__ */

#endif /* SPI_RTDM_H_ */
