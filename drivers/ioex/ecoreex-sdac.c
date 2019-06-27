/**
 * drivers/ioex/ecoreex-sdac.c
 *
 * This file provides external register mapping for the SDAC carrier.
 *
 * Copyright (C) 2013 EMAC, Inc. <support@emacinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/ioex/ecoreex.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/class/gpio.h>
#include <linux/class/pwm.h>

#include <linux/moduleparam.h>
#include <linux/module.h>
#include <asm/io.h>

#include "ecoreex-core.h"

#define SDAC_NAME "SDAC GPI/O expansion R1.0"

static int SDAC_map(unsigned long phys_addr, u8 * virt_addr,
				  unsigned long size, const char *name,
				  struct ecoreex_data *data)
{
	gpio_t *gpio_pwr;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}

	gpio_declare();

	gpio_pwr =
	    gpo_device_create_unregistered(&virt_addr[0], "pwr_ctl");
	gpio_pwr->index_read = gpio_index_read;
	gpio_pwr->index_write = gpio_index_write;
	gpio_pwr->range = 2;
	ecoreex_setup_data_access(data->e_access, gpio_pwr);

	gpio_register_device(gpio_pwr);

	return 0;
}

void map_core(unsigned long phys_addr, unsigned long size,
			    struct ecoreex_data *data)
{
	data->phys_addr = phys_addr;
	data->map_size = size;
	data->virt_addr = ioremap_nocache(phys_addr, size);

	if (data->virt_addr == NULL)
		printk("could not remap physical memory at %lx for EMAC core\n",
		       phys_addr);
	else {
		SDAC_map(phys_addr, data->virt_addr, size, SDAC_NAME, data);
	}
}

EXPORT_SYMBOL(map_core);
