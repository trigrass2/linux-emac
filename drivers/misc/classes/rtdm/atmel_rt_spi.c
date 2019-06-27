/**
 * drivers/misc/classes/rtdm/atmel_rt_spi.c
 *
 * This file provides access methods for implementing an RTDM SPI Class
 * interface to the Atmel AT91 SPI controllers.
 *
 * As much as possible of this driver was taken from the Linux Atmel driver
 * in atmel_spi.c. Most functions were adapted directly from this driver.
 *
 * Copyright (C) 2010 EMAC, Inc. <support@emacinc.com>
 * Code copied from atmel_spi.c Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

//#define DEBUG

#include <rtdm/rtdm_driver.h>
#include <xenomai/native/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/cpu.h>
#include <linux/class/spi.h>
#include <linux/class/rtdm/spi_rtdm.h>
#include <linux/class/rtdm/atmel_spi_rtdm.h>

#include "atmel_rt_spi.h"


/**
 * struct defining an atmel rt_spi device. Each bus is associated with
 * one of these structures.
 * @lock: lock to handle synchronization
 * @irq_handle: RTDM IRQ handle for this device
 * @regs: controller address space
 * @irq: interrupt number
 * @clk: associated SPI clock
 * @bus_hz: bus clock speed in Hz
 * @pdev: platform device used for registration
 * @new_1: determines the controller type
 * @stay: current slave device
 * @stopping: true if the controller is being stopped
 * @queue: queue of transfers
 * @current_transfer: the current transfer
 * @current_remaining_bytes: bytes left in current transfer
 * @next_transfer: next transfer in the queue
 * @next_remaining_bytes: bytes left in next transfer
 * @buffer: scratch buffer if MOSI/MISO NULL or disregarded
 * @buffer_dma: DMA address of scratch buffer
 * @num_devices: number of devices on this bus
 */
struct atmel_rt_spi {
	rtdm_lock_t	lock;
	rtdm_irq_t irq_handle;
	void __iomem *regs;
	int irq;
	struct clk *clk;
	unsigned long bus_hz;
	struct platform_device *pdev;
	unsigned new_1:1;
	struct spi_s *stay;
	u8 stopping;
	struct list_head queue;
	struct rt_spi_transfer *current_transfer;
	unsigned long current_remaining_bytes;
	struct rt_spi_transfer *next_transfer;
	unsigned long next_remaining_bytes;
	void *buffer;
	dma_addr_t buffer_dma;
	u8 num_devices;
};

#define BUFFER_SIZE		PAGE_SIZE
#define INVALID_DMA_ADDRESS	0xffffffff


/**
 * Function to activate chip select associated with the given device.
 *
 * Earlier SPI controllers (e.g. on at91rm9200) have a design bug whereby
 * they assume that spi slave device state will not change on deselect, so
 * that automagic deselection is OK.  ("NPCSx rises if no data is to be
 * transmitted")  Not so!  Workaround uses nCSx pins as GPIOs; or newer
 * controllers have CSAAT and friends.
 *
 * Since the CSAAT functionality is a bit weird on newer controllers as
 * well, we use GPIO to control nCSx pins on all controllers, updating
 * MR.PCS to avoid confusing the controller.  Using GPIOs also lets us
 * support active-high chipselects despite the controller's belief that
 * only active-low devices/systems exists.
 *
 * However, at91rm9200 has a second erratum whereby nCS0 doesn't work
 * right when driven with GPIO.  ("Mode Fault does not allow more than one
 * Master on Chip Select 0.")  No workaround exists for that ... so for
 * nCS0 on that chip, we (a) don't use the GPIO, (b) can't support CS_HIGH,
 * and (c) will trigger that first erratum in some cases.
 */
static void cs_activate(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);
	struct atmel_rt_spi *art = spi_to_atmel(spi);
	unsigned gpio = (unsigned) rt->cs_pin;
	unsigned active = 0; /* EMAC SPI Class does not have an option for this */
	u32 mr;
	int i;
	u32 csr;
	u32 cpol = (rt->mode & SPICL_CPOL) ? SPI_BIT(CPOL) : 0;

	/* Make sure clock polarity is correct */
	for (i = 0; i < art->num_devices; i++) {
		csr = spi_readl(art, CSR0 + 4 * x[i]);
		if ((csr ^ cpol) & SPI_BIT(CPOL))
			spi_writel(art, CSR0 + 4 * x[i], csr ^ SPI_BIT(CPOL));
	}

	mr = spi_readl(art, MR);
	mr = SPI_BFINS(PCS, ~(1 << x[rt->cs_num]), mr);

	dev_dbg(&art->pdev->dev, "activate %u%s, mr %08x\n",
			gpio, active ? " (high)" : "",
			mr);

	if (likely(!(cpu_is_at91rm9200() && rt->cs_num == 0)))
		gpio_set_value(gpio, active);
	spi_writel(art, MR, mr);
}


/**
 * function to deactivate chip select associated with the
 * given device.
 */
static void cs_deactivate(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);
	struct atmel_rt_spi *art = spi_to_atmel(spi);
	unsigned gpio = (unsigned) rt->cs_pin;
	/* note that currently CS always active low with SPI class */
	unsigned active = 0;
	u32 mr;

	/* only deactivate *this* device; sometimes transfers to
	 * another device may be active when this routine is called.
	 */
	mr = spi_readl(art, MR);
	if (~SPI_BFEXT(PCS, mr) & (1 << x[rt->cs_num])) {
		mr = SPI_BFINS(PCS, 0xf, mr);
		spi_writel(art, MR, mr);
	}

	dev_dbg(&art->pdev->dev, "DEactivate %u%s, mr %08x\n",
			gpio, active ? " (low)" : "",
			mr);

	if (likely(!(cpu_is_at91rm9200() && rt->cs_num == 0)))
		gpio_set_value(gpio, !active);
}

#define MODEBITS (SPICL_CPHA | SPICL_CPOL)
#define BPWBITS  (SPICL_EIGHTBIT | \
	SPICL_TENBIT | \
	SPICL_TWELVEBIT | \
	SPICL_SIXTEENBIT)

/**
 * helper function to get the bits-per-word setting from
 * a given SPI class mode value
 */
static int mode_get_bpw(u8 mode)
{
	switch (mode & BPWBITS)
	{
	case SPICL_EIGHTBIT:
		return 8;
		break;
	case SPICL_TENBIT:
		return 10;
		break;
	case SPICL_TWELVEBIT:
		return 12;
		break;
	case SPICL_SIXTEENBIT:
		return 16;
		break;
	default:
		return 8;
		break;
	}
}

/**
 * function to determine if given transfer is the last in the list
 * for the given message
 */
static inline int atmel_rt_spi_xfer_is_last(struct rt_spi_message *msg,
					struct rt_spi_transfer *xfer)
{
	return (msg->transfers.prev == &xfer->transfer_list);
}

/**
 * function to determine if the transfer is valid to be chained with
 * other transfers
 */
static inline int atmel_rt_spi_xfer_can_be_chained(struct rt_spi_transfer *xfer)
{
	return (xfer->delay_usecs == 0) && (!xfer->cs_change);
}

/**
 * function to submit the next data from the current transfer
 */
static void atmel_rt_spi_next_xfer_data(struct atmel_rt_spi *art,
				struct rt_spi_transfer *xfer,
				dma_addr_t *tx_dma,
				dma_addr_t *rx_dma,
				u32 *plen)
{
	u32	len = *plen;

	/* use scratch buffer only when rx or tx data is unspecified */
	if (xfer->rx_buf) {
		*rx_dma = xfer->rx_dma + xfer->len - len;
	} else {
		*rx_dma = art->buffer_dma;
		if (len > BUFFER_SIZE)
			len = BUFFER_SIZE;
	}
	if (xfer->tx_buf) {
		*tx_dma = xfer->tx_dma + xfer->len - len;
	} else {
		*tx_dma = art->buffer_dma;
		if (len > BUFFER_SIZE)
			len = BUFFER_SIZE;
		memset(art->buffer, 0, len);
		dma_sync_single_for_device(&art->pdev->dev,
				art->buffer_dma, len, DMA_TO_DEVICE);
	}
	dev_dbg(&art->pdev->dev, "%s\n", __func__);

	*plen = len;
}

/**
 * function to submit the next transfer in the given message
 * for DMA. The device lock is held, and spi irq is blocked
 * before calling this function.
 */
static void atmel_rt_spi_next_xfer(struct atmel_rt_spi *art,
				struct rt_spi_message *msg)
{
	struct rt_spi_transfer *xfer;
	struct rt_spi_device *rt;
	u32	len;
	u32 remaining;
	u32	ieval;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	u8 bits;

	rt = spi_to_rt(msg->spi);
	bits = mode_get_bpw(rt->mode);

	if (!art->current_transfer)
		xfer = list_entry(msg->transfers.next,
				struct rt_spi_transfer, transfer_list);
	else if (!art->next_transfer)
		xfer = list_entry(art->current_transfer->transfer_list.next,
				struct rt_spi_transfer, transfer_list);
	else
		xfer = NULL;

	if (xfer) {
		spi_writel(art, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));

		len = xfer->len;
		atmel_rt_spi_next_xfer_data(art, xfer, &tx_dma, &rx_dma, &len);
		remaining = xfer->len - len;

		spi_writel(art, RPR, rx_dma);
		spi_writel(art, TPR, tx_dma);

		if (bits > 8)
			len >>= 1;
		spi_writel(art, RCR, len);
		spi_writel(art, TCR, len);

		dev_dbg(&art->pdev->dev,
			"  start xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
			xfer->rx_buf, xfer->rx_dma);
	} else {
		xfer = art->next_transfer;
		remaining = art->next_remaining_bytes;
	}

	art->current_transfer = xfer;
	art->current_remaining_bytes = remaining;

	if (remaining > 0)
		len = remaining;
	else if (!atmel_rt_spi_xfer_is_last(msg, xfer)
			&& atmel_rt_spi_xfer_can_be_chained(xfer)) {
		xfer = list_entry(xfer->transfer_list.next,
				struct rt_spi_transfer, transfer_list);
		len = xfer->len;
	} else
		xfer = NULL;

	art->next_transfer = xfer;

	if (xfer) {
		u32	total;

		total = len;
		atmel_rt_spi_next_xfer_data(art, xfer, &tx_dma, &rx_dma, &len);
		art->next_remaining_bytes = total - len;

		spi_writel(art, RNPR, rx_dma);
		spi_writel(art, TNPR, tx_dma);

		if (bits > 8)
			len >>= 1;
		spi_writel(art, RNCR, len);
		spi_writel(art, TNCR, len);

		dev_dbg(&art->pdev->dev,
			"  next xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
			xfer->rx_buf, xfer->rx_dma);
		ieval = SPI_BIT(ENDRX) | SPI_BIT(OVRES);
	} else {
		spi_writel(art, RNCR, 0);
		spi_writel(art, TNCR, 0);
		ieval = SPI_BIT(RXBUFF) | SPI_BIT(ENDRX) | SPI_BIT(OVRES);
	}

	/*
	 * REVISIT: We're waiting for ENDRX before we start the next
	 * transfer because we need to handle some difficult timing
	 * issues otherwise. If we wait for ENDTX in one transfer and
	 * then start waiting for ENDRX in the next, it's difficult
	 * to tell the difference between the ENDRX interrupt we're
	 * actually waiting for and the ENDRX interrupt of the
	 * previous transfer. (per atmel_spi.c driver comments)
	 */
	spi_writel(art, IER, ieval);
	spi_writel(art, PTCR, SPI_BIT(TXTEN) | SPI_BIT(RXTEN));
}


/**
 * function to initiate the next message for the given device
 */
static void atmel_rt_spi_next_message(struct atmel_rt_spi *art)
{
	struct rt_spi_message *msg;
	struct spi_s *spi;

	BUG_ON(art->current_transfer);

	msg = list_entry(art->queue.next, struct rt_spi_message, queue);
	spi = msg->spi;

	dev_dbg(&art->pdev->dev, "start message %p for %s\n",
			msg, spi->name);

	/* select chip if it's not still active */
	if (art->stay) {
		if (art->stay != spi) {
			cs_deactivate(art->stay);
			cs_activate(spi);
		}
		art->stay = NULL;
	} else
		cs_activate(spi);

	atmel_rt_spi_next_xfer(art, msg);
}


/**
 * function called to complete the current message
 */
static void
atmel_rt_spi_msg_done(struct atmel_rt_spi *art,	struct rt_spi_message *msg,
		int status, int stay)
{
	dev_dbg(&art->pdev->dev, "%s: status = %d, stay = %d\n",
			__func__, status, stay);
	if (!stay || status < 0)
		cs_deactivate(msg->spi);
	else
		art->stay = msg->spi;

	msg->status = status;

	dev_dbg(&art->pdev->dev,
		"xfer complete: %u bytes transferred\n",
		msg->actual_length);

	/*
	 * complete the current message -- this is called with the
	 * spinlock held from interrupt handler. msg->complete()
	 * must be interrupt safe.
	 */
	if (!msg->dma_disabled) {
		list_del(&msg->queue);
		msg->complete(msg->context);

		art->current_transfer = NULL;
		art->next_transfer = NULL;

		/* continue if needed */
		if (list_empty(&art->queue) || art->stopping)
			spi_writel(art, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));
		else
			atmel_rt_spi_next_message(art);
	}
}


/**
 * function to configure an SPI device based on the mode and speed
 * settings associated with it. Called from both the speedwrite and
 * confwrite functions. After configuration the members of the spi
 * device will be adjusted to match the actual configuration if
 * different from the desired settings.
 */
static int atmel_rt_spi_setup(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);
	struct atmel_rt_spi *art = spi_to_atmel(spi);
	u32	scbr;
	u32 csr;
	u8 bits;
	rtdm_lockctx_t context;

	if (art->stopping)
		return -ESHUTDOWN;

	bits = mode_get_bpw(rt->mode);

	/*
	 * this is a fact that should be reflected to the
	 * user if speedread is called
	 */
	if (rt->max_speed_hz > art->bus_hz)
		rt->max_speed_hz = art->bus_hz;

	if (rt->max_speed_hz) {
		/*
		 * Calculate the lowest divider that satisfies the
		 * constraint, assuming div32/fdiv/mbz == 0.
		 */
		scbr = DIV_ROUND_UP(art->bus_hz, rt->max_speed_hz);

		/*
		 * If the resulting divider doesn't fit into the
		 * register bitfield, we can't satisfy the constraint.
		 */
		if (scbr >= (1 << SPI_SCBR_SIZE)) {
			dev_dbg(&art->pdev->dev,
				"setup: %d Hz too slow, scbr %u; min %ld Hz\n",
				rt->max_speed_hz, scbr, art->bus_hz/255);
			return -EINVAL;
		}
	} else
		/* speed zero means "as slow as possible" */
		scbr = 0xff;

	csr = SPI_BF(SCBR, scbr) | SPI_BF(BITS, bits - 8);
	if (rt->mode & SPICL_CPOL)
		csr |= SPI_BIT(CPOL);
	if (!(rt->mode & SPICL_CPHA))
		csr |= SPI_BIT(NCPHA);

	rtdm_lock_get_irqsave(&art->lock, context);
	if (art->stay == spi)
		art->stay = NULL;
	cs_deactivate(spi);
	rtdm_lock_put_irqrestore(&art->lock, context);

	dev_dbg(&art->pdev->dev,
		"setup: %lu Hz bpw %u mode 0x%x -> csr%d %08x\n",
		art->bus_hz / scbr, bits, rt->mode, rt->cs_num, csr);

	spi_writel(art, CSR0 + 4 * x[rt->cs_num], csr);

	return 0;
}

/**
 * function to map dma addresses for each tx/rx buffer
 * in a transfer. Buffers are guaranteed to be DMA safe
 * before being passed to the driver.
 */
static int
atmel_rt_spi_dma_map_xfer(struct atmel_rt_spi *art, struct rt_spi_transfer *xfer)
{
	struct device *dev = &art->pdev->dev;

	xfer->tx_dma = xfer->rx_dma = INVALID_DMA_ADDRESS;
	if (xfer->tx_buf) {
		xfer->tx_dma = dma_map_single(dev,
				xfer->tx_buf, xfer->len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma))
			return -ENOMEM;
		dev_dbg(dev, "%s mapped tx_dma 0x%p -> 0x%X\n",
				__func__, xfer->tx_buf, xfer->tx_dma);
	}
	if (xfer->rx_buf) {
		xfer->rx_dma = dma_map_single(dev,
				xfer->rx_buf, xfer->len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			if (xfer->tx_buf)
				dma_unmap_single(dev,
						xfer->tx_dma, xfer->len,
						DMA_TO_DEVICE);
			return -ENOMEM;
		}
		dev_dbg(dev, "%s mapped rx_dma 0x%p -> 0x%X\n",
				__func__, xfer->rx_buf, xfer->rx_dma);
	}
	return 0;
}

/**
 * function to unmap dma transfer addresses
 */
static void
atmel_rt_spi_dma_unmap_xfer(struct atmel_rt_spi *art, struct rt_spi_transfer *xfer)
{
	struct device *dev = &art->pdev->dev;

	if (xfer->tx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->tx_dma,
				 xfer->len, DMA_TO_DEVICE);
	if (xfer->rx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->rx_dma,
				 xfer->len, DMA_FROM_DEVICE);
	dev_dbg(dev, "%s\n", __func__);
}


/**
 * interrupt handler
 */
static int atmel_rt_spi_interrupt(rtdm_irq_t *handle)
{
	struct atmel_rt_spi *art;
	struct rt_spi_message *msg;
	struct rt_spi_transfer *xfer;
	u32 status;
	u32 pending;
	u32 imr;
	int ret = RTDM_IRQ_NONE;
	rtdm_lockctx_t context;

	art = rtdm_irq_get_arg(handle, struct atmel_rt_spi);
	dev_dbg(&art->pdev->dev, "atmel_rt_spi_interrupt\n");

	rtdm_lock_get_irqsave(&art->lock, context);

	xfer = art->current_transfer;
	msg = list_entry(art->queue.next, struct rt_spi_message, queue);

	imr = spi_readl(art, IMR);
	status = spi_readl(art, SR);
	pending = status & imr;

	if (pending & SPI_BIT(OVRES)) {
		int timeout;

		ret = RTDM_IRQ_HANDLED;

		spi_writel(art, IDR, (SPI_BIT(RXBUFF) | SPI_BIT(ENDRX)
				     | SPI_BIT(OVRES)));

		/*
		 * When we get an overrun, we disregard the current
		 * transfer. Data will not be copied back from any
		 * bounce buffer and msg->actual_len will not be
		 * updated with the last xfer.
		 *
		 * We will also not process any remaning transfers in
		 * the message.
		 *
		 * First, stop the transfer and unmap the DMA buffers.
		 */
		spi_writel(art, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));
		if (!msg->is_dma_mapped)
			atmel_rt_spi_dma_unmap_xfer(art, xfer);

		/*
		 * Atmel driver was doing a udelay here. This is unacceptable
		 * especially in real-time interrupt handler. We skip the
		 * cs delay in case of an overrun instead.
		 */

		dev_warn(&art->pdev->dev, "overrun (%u/%u remaining)\n",
			 spi_readl(art, TCR), spi_readl(art, RCR));

		/*
		 * Clean up DMA registers and make sure the data
		 * registers are empty.
		 */
		spi_writel(art, RNCR, 0);
		spi_writel(art, TNCR, 0);
		spi_writel(art, RCR, 0);
		spi_writel(art, TCR, 0);
		for (timeout = 1000; timeout; timeout--)
			if (spi_readl(art, SR) & SPI_BIT(TXEMPTY))
				break;
		if (!timeout)
			dev_warn(&art->pdev->dev,
				 "timeout waiting for TXEMPTY");
		while (spi_readl(art, SR) & SPI_BIT(RDRF))
			spi_readl(art, RDR);

		/* Clear any overrun happening while cleaning up */
		spi_readl(art, SR);

		atmel_rt_spi_msg_done(art, msg, -EIO, 0);
	} else if (pending & (SPI_BIT(RXBUFF) | SPI_BIT(ENDRX))) {
		ret = RTDM_IRQ_HANDLED;

		spi_writel(art, IDR, pending);

		if (art->current_remaining_bytes == 0) {
			msg->actual_length += xfer->len;

			if (!msg->dma_disabled && !msg->is_dma_mapped)
				atmel_rt_spi_dma_unmap_xfer(art, xfer);

			/*
			 * Delay between transfer is not supported in DMA mode. Having a busy sleep
			 * of several microseconds inside the interrupt handler is not acceptable.
			 * Enable for testing only!
			 */
#if 0
			if (xfer->delay_usecs) {
				dev_dbg(&art->pdev->dev, "  delaying %d usecs\n", xfer->delay_usecs);
				rtdm_task_busy_sleep(xfer->delay_usecs*1000);
			}
#endif
			if (atmel_rt_spi_xfer_is_last(msg, xfer)) {
				/* report completed message */
				atmel_rt_spi_msg_done(art, msg, 0, xfer->cs_change);
			} else {
				if (xfer->cs_change) {
					cs_deactivate(msg->spi);
					/*
					 * a delay of some sort is probably necessary here. Original atmel driver
					 * had 1 usec. reduced to 500 ns to keep the delay as short as possible.
					 * Note that this condition should be encountered rarely.
					 */
					rtdm_task_busy_sleep(500);
					cs_activate(msg->spi);
				}

				/*
				 * Not done yet. Submit the next transfer.
				 */
				atmel_rt_spi_next_xfer(art, msg);
			}
		} else {
			/*
			 * Keep going, we still have data to send in
			 * the current transfer.
			 */
			atmel_rt_spi_next_xfer(art, msg);
		}
	}

	rtdm_lock_put_irqrestore(&art->lock, context);

	return ret;
}

/**
 * function to transfer an entire message in non-DMA mode
 */
static void atmel_rt_spi_xfer_msg_nodma(struct atmel_rt_spi *art, struct rt_spi_message *msg)
{
	struct rt_spi_transfer *xfer = NULL;
	struct rt_spi_device *rt;
	u8 *tx;
	u8 *rx;
	u32 len_tx;
	u32 len_rx;
	u8 bytes_per_rw;
	u32 status;
	u32 value;
	u16 timeout;
	u8 cs_change = 0; /* placeholder for cs_change on current xfer */

	rt = spi_to_rt(msg->spi);
	bytes_per_rw = (mode_get_bpw(rt->mode) > 8) ? 2 : 1;
	dev_dbg(&art->pdev->dev, "%s\n", __func__);

	/* submit all transfers to the hardware */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		tx = xfer->tx_buf;
		rx = xfer->rx_buf;
		len_tx = 0;
		len_rx = 0;
		cs_change = xfer->cs_change;
		msg->actual_length += xfer->len;
		dev_dbg(&art->pdev->dev, "%s submitting next xfer %p: %d "
				"bytes: %d bpw, cs_change = %d\n",
				__func__, xfer, xfer->len, bytes_per_rw * 8,
				xfer->cs_change);

		/* continue until all data has been transferred */
		while (len_rx < xfer->len) {
			status = spi_readl(art, SR);
			/* handle overrun errors */
			if (status & SPI_BIT(OVRES)) {
				dev_warn(&art->pdev->dev, "overrun on %s\n", msg->spi->name);

				for (timeout = 1000; timeout; timeout--)
					if (spi_readl(art, SR) & SPI_BIT(TXEMPTY))
						break;
				if (!timeout)
					dev_warn(&art->pdev->dev,
							"timeout waiting for TXEMPTY");
				while (spi_readl(art, SR) & SPI_BIT(RDRF))
					spi_readl(art, RDR);

				/* Clear any overrun happening while cleaning up */
				spi_readl(art, SR);

				atmel_rt_spi_msg_done(art, msg, -EIO, 0);
				return;
			}
			/* check if more data can be written to TDR and transmit */
			if ((len_tx < xfer->len) && (status & SPI_BIT(TDRE))) {
				if (!tx)
					value = 0;
				else {
					value = *tx++;
					if (unlikely(bytes_per_rw == 2))
						value |= (*tx++ << 8);
				}
				len_tx += bytes_per_rw;
				spi_writel(art, TDR, value);
			}
			/* check if data ready to receive and read if possible */
			if (status & SPI_BIT(RDRF)) {
				value = spi_readl(art, RDR);
				if (likely(rx)) {
					*rx++ = value & 0xFF;
					if (unlikely(bytes_per_rw == 2))
						*rx++ = (value >> 8) & 0xFF;
				}
				len_rx += bytes_per_rw;
			}
			if (len_rx == xfer->len) {
				/* delay and toggle cs if necessary */
				if (xfer->delay_usecs) {
					dev_dbg(&art->pdev->dev, "  delaying for %d usecs\n", xfer->delay_usecs);
					rtdm_task_busy_sleep(xfer->delay_usecs * 1000);
				}
				if (xfer->cs_change) {
					dev_dbg(&art->pdev->dev, "  toggling chip select due to cs_change\n");
					cs_deactivate(msg->spi);
					rtdm_task_busy_sleep(500);
					cs_activate(msg->spi);
				}
			}
		} /* end while current transfer */
	} /* end while current message */
	atmel_rt_spi_msg_done(art, msg, 0, cs_change);
}

/**
 * function called from atmel_rt_spi_transfer to transfer a message without
 * using DMA. This is called without the rt SPI device lock held. These
 * transfers need to be as fast as possible so they have a separate code
 * path early in the transaction process to minimize any code delay from
 * conditionals or other unecessary processing that would be done if they
 * used the same methods as the DMA counterparts. Also, queueing messages
 * for transfer on the device does not have a value in non-DMA mode as there
 * is no sense of asynchronous transfer, so this step is avoided altogeter
 * and each message is transfered directly.
 */
static void atmel_rt_spi_xfer_nodma(struct spi_s *spi, struct rt_spi_message *msg)
{
	struct atmel_rt_spi *art = spi_to_atmel(spi);
	rtdm_lockctx_t context;
	u32 imr;

	dev_dbg(&art->pdev->dev, "%s\n", __func__);
	/*
	 * first check to make sure that no DMA messages are in-transfer
	 * and obtain the lock on this SPI bus.
	 */

wait_empty:
//	while (!list_empty(&art->queue));

	rtdm_lock_get_irqsave(&art->lock, context);
	/* in the unlikely event that someone added a message between
	 * checking the queue and retrieving the lock, wait for the message
	 * to complete and try again.
	 */
	if (unlikely(!list_empty(&art->queue))) {
		rtdm_lock_put_irqrestore(&art->lock, context);
		goto wait_empty;
	}

	dev_dbg(&art->pdev->dev, "   lock obtained\n");
	/* disable all SPI interrupts */
	imr = spi_readl(art, IMR);
	spi_writel(art, IDR, imr);

	/* select chip if it's not still active */
	if (art->stay) {
		if (art->stay != spi) {
			cs_deactivate(art->stay);
		}
		art->stay = NULL;
	}
	cs_activate(spi);

	atmel_rt_spi_xfer_msg_nodma(art, msg);

	/* re-enable interrupts and exit */
	spi_writel(art, IER, imr);

	rtdm_lock_put_irqrestore(&art->lock, context);
	dev_dbg(&art->pdev->dev, "   lock released\n");
}

/**
 * function to submit an spi message for transfer to the given device
 */
static int atmel_rt_spi_transfer(struct spi_s *spi, struct rt_spi_message *msg)
{
	struct atmel_rt_spi *art = spi_to_atmel(spi);
	struct rt_spi_transfer *xfer;
	rtdm_lockctx_t context;

	dev_dbg(&art->pdev->dev, "new message %p submitted for %s dma_disabled is %d\n",
			msg, spi->name, msg->dma_disabled);

	if (unlikely(list_empty(&msg->transfers)))
		return -EINVAL;

	if (unlikely(art->stopping))
		return -ESHUTDOWN;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!(xfer->tx_buf || xfer->rx_buf) && xfer->len) {
			dev_dbg(&art->pdev->dev, "missing rx or tx buf\n");
			return -EINVAL;
		}
		/*
		 * DMA map early, for performance (empties dcache ASAP) and
		 * better fault reporting. This is only applicable if the specified
		 * device is in DMA mode.
		 *
		 * NOTE that if dma_unmap_single() ever starts to do work on
		 * platforms supported by this driver, we would need to clean
		 * up mappings for previously-mapped transfers.
		 */
		if (!msg->dma_disabled && !msg->is_dma_mapped) {
			if (atmel_rt_spi_dma_map_xfer(art, xfer) < 0)
				return -ENOMEM;
		}
	}

#ifdef VERBOSE
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		dev_dbg(&art->pdev->dev,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			xfer, xfer->len,
			xfer->tx_buf, xfer->tx_dma,
			xfer->rx_buf, xfer->rx_dma);
	}
#endif

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	if (msg->dma_disabled) {
		atmel_rt_spi_xfer_nodma(spi, msg);
	} else {
		rtdm_lock_get_irqsave(&art->lock, context);
		list_add_tail(&msg->queue, &art->queue);
		if (!art->current_transfer)
			atmel_rt_spi_next_message(art);
		rtdm_lock_put_irqrestore(&art->lock, context);
	}

	return 0;
}

static void atmel_rt_spi_complete(void *event)
{
	rtdm_event_signal(event);
}

static void atmel_rt_spi_empty_complete(void *event)
{
}

/**
 * blocking SPI transfer. Mimics the behavior of the spi_sync() function
 * in the standard Linux SPI API.
 */
static int atmel_rt_spi_sync(struct spi_s *spi, struct rt_spi_message *msg)
{
	rtdm_event_t event;
	int status;

	msg->spi = spi;
	if (!msg->dma_disabled) {
		rtdm_event_init(&event, 0);
		msg->complete = atmel_rt_spi_complete;
		msg->context = &event;
	}
	else
		msg->complete = atmel_rt_spi_empty_complete;

	status = atmel_rt_spi_transfer(spi, msg);
	/*
	 *  If DMA is disabled, the transfer is already complete.
	 * Otherwise, wait until the completion event -- note that
	 * this can cause rescheduling.
	 */
	if (!msg->dma_disabled && (status == 0)) {
		rtdm_event_wait(&event);
	}
	status = (status == 0) ? msg->status : status;
	msg->context = NULL;
	return status;
}

/** SPI Class function definitions **/

int atmel_rt_spi_tip(struct spi_s *spi, int onoff)
{
	return 0;
}

int atmel_rt_spi_confwrite(struct spi_s *spi, spi_control conf)
{
	struct rt_spi_device *rt = spi_to_rt(spi);

	rt->mode = conf;
	atmel_rt_spi_setup(spi);

	return 0;
}

spi_control atmel_rt_spi_confread(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);
	/* return the current mode settings */
	return rt->mode;
}

spi_control atmel_rt_spi_speedread(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);

	return rt->max_speed_hz;
}

int atmel_rt_spi_speedwrite(struct spi_s *spi, spi_control speed)
{
	struct rt_spi_device *rt = spi_to_rt(spi);

	rt->max_speed_hz = speed;
	return atmel_rt_spi_setup(spi);
}


int atmel_rt_spi_xmitmsg(struct spi_s *spi, struct spi_message_s *msg)
{
	struct rt_spi_transfer rt_xfer_1;
	struct rt_spi_transfer rt_xfer_2;
	struct rt_spi_message rt_msg;
	struct atmel_rt_spi *art = spi_to_atmel(spi);

	dev_dbg(&art->pdev->dev, "%s\n", __func__);

	spi_transfer_to_rt(&msg->xfer_1, &rt_xfer_1);
	spi_transfer_to_rt(&msg->xfer_2, &rt_xfer_2);

	rt_xfer_1.cs_change = msg->cs_change;
	rt_xfer_2.cs_change = 0;

	rt_xfer_1.delay_usecs = msg->usec_between_xfer;
	rt_xfer_2.delay_usecs = 0;

	rt_spi_message_init(&rt_msg);
	rt_spi_message_add_tail(&rt_xfer_1, &rt_msg);
	rt_spi_message_add_tail(&rt_xfer_2, &rt_msg);

	rt_msg.dma_disabled = spi_dma_disabled(spi);
	rt_msg.is_dma_mapped = 0;

	return atmel_rt_spi_sync(spi, &rt_msg);
}

int atmel_rt_spi_xmit(struct spi_s *spi, u8 *mosi, u8 *miso, int size)
{
	struct rt_spi_transfer xfer;
	struct rt_spi_message msg;
	struct atmel_rt_spi *art = spi_to_atmel(spi);

	dev_dbg(&art->pdev->dev, "%s\n", __func__);

	/*
	 * no need to check buffer validity -- the transfer function
	 * will handle any NULL buffers appropriately
	 */
	xfer.tx_buf = mosi;
	xfer.rx_buf = miso;
	xfer.len = size;
	xfer.cs_change = 0;
	xfer.delay_usecs = 0;

	rt_spi_message_init(&msg);
	rt_spi_message_add_tail(&xfer, &msg);

	msg.dma_disabled = spi_dma_disabled(spi);
	msg.is_dma_mapped = 0;

	return atmel_rt_spi_sync(spi, &msg);
}

int atmel_rt_spi_register_device(struct spi_s *spi)
{
	struct rt_spi_device *rt = spi_to_rt(spi);

	rt->sync = atmel_rt_spi_sync;
	/* allow SPI class driver to handle custom initialization */
	if (rt->driver_data)
		spi->driver_data = rt->driver_data;
	if (rt->driver_init)
		spi->driver_init = rt->driver_init;

	/* devices may provide their own functions if necessary */
	if (!spi->tip)
		spi->tip = atmel_rt_spi_tip;
	if (!spi->xmit)
		spi->xmit = atmel_rt_spi_xmit;
	if (!spi->xmitmsg)
		spi->xmitmsg = atmel_rt_spi_xmitmsg;
	if (!spi->confwrite)
		spi->confwrite = atmel_rt_spi_confwrite;
	if (!spi->confread)
		spi->confread = atmel_rt_spi_confread;
	if (!spi->speedread)
		spi->speedread = atmel_rt_spi_speedread;
	if (!spi->speedwrite)
		spi->speedwrite = atmel_rt_spi_speedwrite;

	spi->enable_rtdm = 1;
	spi->disable_char = 1;

	/* configure the device per its initial settings */
	atmel_rt_spi_setup(spi);

	return spi_class_register_device(spi) ? 0 : 1;
}

static int __init atmel_rt_spi_probe(struct platform_device *pdev)
{
	struct atmel_rt_spi_plat *pdata;
	struct resource *regs;
	int irq;
	struct clk *clk;
	int ret;
	struct atmel_rt_spi *art;
	int i;
	const char *rt_name[] = { "atmel_rt_spi.0", "atmel_rt_spi.1" };

	pdata = pdev->dev.platform_data;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = -ENOMEM;
	if (!(art = kzalloc(sizeof(struct atmel_rt_spi), GFP_KERNEL)))
		goto out_free;

	/*
	 * Scratch buffer is used for throwaway rx and tx data.
	 * It's coherent to minimize dcache pollution.
	 */
	art->buffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					&art->buffer_dma, GFP_KERNEL);
	if (!art->buffer)
		goto out_free;

	rtdm_lock_init(&art->lock);
	INIT_LIST_HEAD(&art->queue);
	art->pdev = pdev;
	art->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
	if (!art->regs)
		goto out_free_buffer;
	art->irq = irq;
	art->clk = clk;
	art->num_devices = pdata->num_devices;
	if (!cpu_is_at91rm9200())
		art->new_1 = 1;

	platform_set_drvdata(pdev, art);

	/* request the irq for this controller */
	ret = rtdm_irq_request(&art->irq_handle, irq,
			atmel_rt_spi_interrupt, 0, rt_name[pdata->devices[0].bus_num],
			art);
	if (ret < 0)
		goto out_free_buffer;

	/* Hardware initialization */
	clk_enable(clk);

	spi_writel(art, CR, SPI_BIT(SWRST));
	spi_writel(art, CR, SPI_BIT(SWRST)); /* AT91SAM9263 Rev B workaround */
	spi_writel(art, MR, SPI_BIT(MSTR) | SPI_BIT(MODFDIS));
	spi_writel(art, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));
	spi_writel(art, CR, SPI_BIT(SPIEN));

	/*
	 * clk_get_rate() cannot be called from Xenomai domain so
	 * the result must be saved at initialization.
	 */
	art->bus_hz = clk_get_rate(clk);
	/* older chips start out at half peripheral bus speed */
	if (!art->new_1)
		art->bus_hz /= 2;

	dev_info(&pdev->dev, "Atmel SPI Controller at 0x%08lx (irq %d)\n",
			(unsigned long)regs->start, irq);

	for (i = 0; i < art->num_devices; i++) {
		pdata->devices[i].controller_data = art;
		pdata->devices[i].cs_num = i;
		if (atmel_rt_spi_register_device(&(pdata->devices[i].spi)) != 0)
			goto out_reset_hw;
	}

	return 0;

out_reset_hw:
	spi_writel(art, CR, SPI_BIT(SWRST));
	spi_writel(art, CR, SPI_BIT(SWRST)); /* AT91SAM9263 Rev B workaround */
	clk_disable(clk);
	rtdm_irq_free(&art->irq_handle);
out_free_buffer:
	dma_free_coherent(&pdev->dev, BUFFER_SIZE, art->buffer,
			art->buffer_dma);
out_free:
	clk_put(clk);
	kfree(art);
	return ret;
}


static int __exit atmel_rt_spi_remove(struct platform_device *pdev)
{
	struct atmel_rt_spi *rt_spi = platform_get_drvdata(pdev);
	struct rt_spi_message	*msg;

	/* reset the hardware and block queue progress */
	rtdm_lock_get(&rt_spi->lock);
	rt_spi->stopping = 1;
	spi_writel(rt_spi, CR, SPI_BIT(SWRST));
	spi_writel(rt_spi, CR, SPI_BIT(SWRST)); /* AT91SAM9263 Rev B workaround */
	spi_readl(rt_spi, SR);
	rtdm_lock_put(&rt_spi->lock);

	/* Terminate remaining queued transfers */
	list_for_each_entry(msg, &rt_spi->queue, queue) {
		/* REVISIT unmapping the dma is a NOP on ARM and AVR32
		 * but we shouldn't depend on that...
		 */
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}

	dma_free_coherent(&pdev->dev, BUFFER_SIZE, rt_spi->buffer,
			rt_spi->buffer_dma);

	clk_disable(rt_spi->clk);
	clk_put(rt_spi->clk);
	rtdm_irq_free(&rt_spi->irq_handle);
	iounmap(rt_spi->regs);

	kfree(rt_spi);
	return 0;
}


static struct platform_driver atmel_rt_spi_driver = {
	.driver		= {
		.name	= "atmel_rt_spi",
		.owner	= THIS_MODULE,
	},
	.probe = atmel_rt_spi_probe,
	.suspend	= NULL,
	.resume		= NULL,
	.remove		= __exit_p(atmel_rt_spi_remove),
};

static int __init atmel_rt_spi_init(void)
{
	return platform_driver_probe(&atmel_rt_spi_driver, atmel_rt_spi_probe);
}
module_init(atmel_rt_spi_init);

static void __exit atmel_rt_spi_exit(void)
{
	platform_driver_unregister(&atmel_rt_spi_driver);
}
module_exit(atmel_rt_spi_exit);

MODULE_DESCRIPTION("Atmel RTDM SPI Driver for the EMAC SPI Class");
MODULE_AUTHOR("EMAC, Inc <support@emacinc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atmel_rt_spi");


