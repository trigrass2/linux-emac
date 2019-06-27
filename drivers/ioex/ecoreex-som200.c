/**
 * drivers/ioex/ecoreex-som200.c
 *
 * This file provides CPLD mapping for the SOM-200ES.
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

#ifdef CONFIG_ECOREEX_SOM200
#define CPLD_SOM200_NAME "EMAC SoM-200ES GPI/O expansion R1.0"
#define CPLD_SOM200R2_NAME "EMAC SoM-200GS GPI/O expansion R2.0"
#define CPLD_SOM200 0xDE
#define CPLD_SOM200R2 0xDF

static inline int CPLD_SOM200_map(unsigned long phys_addr, u8 *virt_addr,
		unsigned long size, const char *name, struct ecoreex_data *data)
{
	gpio_t *gpio_porta;
	gpio_t *gpio_portb;
	gpio_t *gpio_portc;
	gpio_t *gpio_control;


	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk("Could not obtain physical memory at %lx for EMAC core\n", phys_addr);
		iounmap(virt_addr);
	}

	printk("%s detected at %lx\n",name,phys_addr);

	gpio_declare();

	gpio_porta = gpio_device_create_unregistered(&virt_addr[0], &virt_addr[1], "porta");
	gpio_porta->index_write = NULL;
	gpio_porta->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_porta);

	gpio_portb = gpio_device_create_unregistered(&virt_addr[2], &virt_addr[3], "portb");
	gpio_portb->index_write = NULL;
	gpio_portb->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portb);

	/* High Drive */
	gpio_portc = gpio_device_create_unregistered(&virt_addr[4], NULL, "portc");
	gpio_portc->index_write = NULL;
	gpio_portc->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portc);

	gpio_control = gpio_device_create_unregistered(&virt_addr[6], NULL, "control");
	gpio_control->index_write = NULL;
	gpio_control->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_control);

	gpio_register_device(gpio_porta);
	gpio_register_device(gpio_portb);
	gpio_register_device(gpio_portc);
	gpio_register_device(gpio_control);

	return 0;
}

static inline int CPLD_SOM200R2_map(unsigned long phys_addr, u8 *virt_addr,
		unsigned long size, const char *name, struct ecoreex_data *data)
{
	gpio_t *gpio_porta;
	gpio_t *gpio_portb;
	gpio_t *gpio_portc;
	gpio_t *gpio_control;
	gpio_t *gpio_cpldgpio;


	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk("Could not obtain physical memory at %lx for EMAC core\n", phys_addr);
		iounmap(virt_addr);
	}

	printk("%s detected at %lx\n",name,phys_addr);

	gpio_declare();

	gpio_porta = gpio_device_create_unregistered(&virt_addr[0], &virt_addr[1], "porta");
	gpio_porta->index_write = NULL;
	gpio_porta->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_porta);

	gpio_portb = gpio_device_create_unregistered(&virt_addr[2], &virt_addr[3], "portb");
	gpio_portb->index_write = NULL;
	gpio_portb->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portb);

	/* High Drive */
	gpio_portc = gpio_device_create_unregistered(&virt_addr[4], NULL, "portc");
	gpio_portc->index_write = NULL;
	gpio_portc->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portc);

	gpio_control = gpio_device_create_unregistered(&virt_addr[6], NULL, "control");
	gpio_control->index_write = NULL;
	gpio_control->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_control);

	gpio_cpldgpio = gpio_device_create_unregistered(&virt_addr[8], &virt_addr[9], "cpld-gpio");
	gpio_cpldgpio->index_write = NULL;
	gpio_cpldgpio->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_cpldgpio);

	gpio_register_device(gpio_porta);
	gpio_register_device(gpio_portb);
	gpio_register_device(gpio_portc);
	gpio_register_device(gpio_control);
	gpio_register_device(gpio_cpldgpio);

	return 0;
}

#endif

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
		printk("EMAC core version %x detected att %lx\n", version,
				phys_addr + data->key_offset);
		switch (version) {
			case CPLD_SOM200:
				CPLD_SOM200_map(phys_addr, data->virt_addr, size,
						CPLD_SOM200_NAME, data);
				break;
			case CPLD_SOM200R2:
				CPLD_SOM200R2_map(phys_addr, data->virt_addr, size,
						CPLD_SOM200R2_NAME, data);
				break;
			default:
				iounmap(data->virt_addr);
		}
	}
}

EXPORT_SYMBOL(map_core);
