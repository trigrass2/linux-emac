/**
 * drivers/ioex/ecoreex-hwms.c
 *
 * This file provides CPLD mapping for the HWMS.
 *
 * Copyright (C) 2015 EMAC, Inc. <support@emacinc.com>
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


#define CPLD_HWMS_NAME "HWMS GPI/O expansion R1.0"
#define CPLD_HWMS 0xC0

static inline int CPLD_HWMS_map(unsigned long phys_addr, u8 *virt_addr,
		unsigned long size, const char *name, struct ecoreex_data *data)
{
	gpio_t *gpio_counter;
	gpio_t *gpio_control;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk("Could not obtain physical memory at %lx for EMAC core\n", phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	gpio_control = gpio_device_create_unregistered(&virt_addr[0], NULL, "control");
	ecoreex_setup_data_access(data->e_access, gpio_control);
	/* no index read or write for the control -- only one register */
	gpio_control->index_write = NULL;
	gpio_control->index_read = NULL;

	gpio_counter = gpio_device_create_unregistered(&virt_addr[1], NULL, "counter");
	gpio_counter->index_write = gpio_index_write;
	gpio_counter->index_read = gpio_index_read;
	gpio_counter->range = 1;
	ecoreex_setup_data_access(data->e_access, gpio_counter);

	gpio_register_device(gpio_control);
	gpio_register_device(gpio_counter);

	return 0;
}

void map_core(unsigned long phys_addr, unsigned long size,
			    struct ecoreex_data *data)
{
	int version = VERSION_KEY;
	data->phys_addr = phys_addr;
	data->map_size = size;
	data->virt_addr = ioremap_nocache(phys_addr, size);

	if (data->virt_addr == NULL)
		printk("could not remap physical memory at %lx for EMAC core\n",
		       phys_addr);
	else {
		if (VERSION_KEY == -1)
			version = ioread8(&data->virt_addr[data->key_offset]);
		printk("EMAC core version %x detected at %lx\n", version,
				phys_addr + data->key_offset);
		switch (version) {
			case CPLD_HWMS:
				CPLD_HWMS_map(phys_addr, data->virt_addr, size,
					CPLD_HWMS_NAME, data);
				break;
			default:
				iounmap(data->virt_addr);
		}
	}
}

EXPORT_SYMBOL(map_core);
