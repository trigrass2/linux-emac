/**
 * drivers/ioex/ecoreex-evc.c
 *
 * This file provides PLD/FPGA mapping for the EVC.
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
#include <linux/class/gpio.h>
#include <linux/class/pwm.h>

#include <linux/moduleparam.h>
#include <linux/module.h>
#include <asm/io.h>

#include "ecoreex-core.h"

int gpio_ddr_write16(gpio_t * gpio, gpio_data data)
{
	iowrite16(data, gpio->ddr + gpio->index);
	return 0;
}

int gpio_data_write16(gpio_t * gpio, gpio_data data)
{
	iowrite16(data, gpio->data + gpio->index);
	gpio->shadow = data;
	return 0;
}

gpio_data gpio_ddr_read16(gpio_t * gpio)
{
	return ioread16(gpio->ddr + gpio->index);
}

gpio_data gpio_data_read16(gpio_t * gpio)
{
	return ioread16(gpio->data + gpio->index);
}

static inline struct device *gpio_device_create16(void *data, void *ddr,
						const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->ddr = ddr;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_data_write16;
	gpio->data_read = gpio_data_read16;

	if (ddr) {
		gpio->ddr_write = gpio_ddr_write16;
		gpio->ddr_read = gpio_ddr_read16;
	}

	printk("registering gpio device: %s\n", name);
	return gpio_register_device(gpio);
}

static inline struct device *pwm_device_create16(void *width, void *period,
						const char *name)
{
	pwm_t *pwm = kmalloc(sizeof(pwm_t), GFP_KERNEL);
	memset(pwm, 0, sizeof(pwm_t));
	pwm->name = name;
	pwm->subclass = PWMD_SUBCLASS;
	pwm->widthus = width;
	pwm->periodus = period;
	pwm->widthus_write = pwm_widthus_write16;
	pwm->widthus_read = pwm_widthus_read16;

	if (period) {
		pwm->periodus_write = pwm_periodus_write16;
		pwm->periodus_read = pwm_periodus_read16;
	}

	printk("registering pwm device: %s\n", name);
	return pwm_register_device(pwm);
}


#define CPLD_BASE_NAME "EVC CPLD GPIO expansion R1.0"
#define CPLD_BASE 0xe001

static inline int CPLD_BASE_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpio_device_create16(&virt_addr[0], &virt_addr[2], "porta");
	gpio_device_create16(&virt_addr[4], NULL, "portb");
#endif

	return 0;
}

#define FPGA_BASE_NAME "EVC FPGA GPIO expansion R1.0"
#define FPGA_BASE 0xf001

static inline int FPGA_BASE_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);

#ifdef CONFIG_PWMCLASS
	pwm_declare();
	pwm_device_create16(&virt_addr[0], &virt_addr[2], "safety");
	pwm_device_create16(&virt_addr[4], &virt_addr[6], "nebulizer");
	pwm_device_create16(&virt_addr[8], &virt_addr[10], "crossover");
	pwm_device_create16(&virt_addr[12], &virt_addr[14], "blower");
	pwm_device_create16(&virt_addr[16], &virt_addr[18], "air");
	pwm_device_create16(&virt_addr[20], &virt_addr[22], "oxygen");
	pwm_device_create16(&virt_addr[24], &virt_addr[26], "exhalation");
	pwm_device_create16(&virt_addr[28], &virt_addr[30], "heater");
#endif

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
			version = ioread16(&data->virt_addr[data->key_offset]);
		printk("EMAC core version %x detected at %lx\n", version,
		       phys_addr + data->key_offset);
		switch (version) {
		case CPLD_BASE:
			CPLD_BASE_map(phys_addr, data->virt_addr, size,
				      CPLD_BASE_NAME);
			break;
		case FPGA_BASE:
			FPGA_BASE_map(phys_addr, data->virt_addr, size,
				      FPGA_BASE_NAME);
			break;
		default:
			iounmap(data->virt_addr);
		}
	}
}

EXPORT_SYMBOL(map_core);
