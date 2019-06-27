/**
 * drivers/ioex/icb-g2l-pld.c
 *
 * This file provides a mapping for the ICB G2L PLD GPIO devices.
 *
 * Copyright (C) 2012 EMAC, Inc. <support@emacinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/class/gpio.h>
#include <asm/io.h>

#define DRV_MODULE_NAME 	"icb-g2l-pld"
#define DRV_MODULE_VERSION 	"1.0"

static ulong mem = 0xD8000;
module_param(mem, ulong, S_IRUGO);
MODULE_PARM_DESC(mem, "PLD base address in memory. GPIO is mapped at "
		"offset 0x100 from this location.");

#define REG_EXT_STAT 0x0
#define REG_EXT_PWR  0x1
#define REG_SYNC     0x2
#define REG_LED_DAT  0x3
#define REG_LED_SEL  0x4
#define REG_ID       0x5
#define MAP_SIZE     0xFF
#define GPIO_BASE    0x100

struct g2l_devices {
	int id;
	void __iomem *vmem;
	struct device *ext_stat;
	struct device *ext_pwr;
	struct device *sync;
	struct device *led_dat;
	struct device *led_sel;
	struct device *pld_id;
};

static struct g2l_devices *devs;

static void icb_g2l_delete(void)
{
	if (devs->ext_stat)
		gpio_erase(devs->ext_stat);
	if (devs->ext_pwr)
		gpio_erase(devs->ext_pwr);
	if (devs->sync)
		gpio_erase(devs->sync);
	if (devs->led_dat)
		gpio_erase(devs->led_dat);
	if (devs->led_sel)
		gpio_erase(devs->led_sel);
	if (devs->pld_id)
		gpio_erase(devs->pld_id);
}

static int __init icb_g2l_register(void)
{
	if (!devs->vmem)
		return -EINVAL;
	devs->ext_stat = gpio_device_create(devs->vmem + REG_EXT_STAT, NULL,
			"ext_stat");
	if (!devs->ext_stat)
		goto err_register;
	devs->ext_pwr = gpio_device_create(devs->vmem + REG_EXT_PWR, NULL,
			"ext_pwr");
	if (!devs->ext_pwr)
		goto err_register;
	
	devs->sync = gpio_device_create(devs->vmem + REG_SYNC, NULL,
			"sync");
	if (!devs->sync)
		goto err_register;

	devs->led_dat = gpio_device_create(devs->vmem + REG_LED_DAT, NULL,
			"led_dat");
	if (!devs->led_dat)
		goto err_register;

	devs->led_sel = gpio_device_create(devs->vmem + REG_LED_SEL, NULL,
			"led_sel");
	if (!devs->led_sel)
		goto err_register;

	devs->pld_id = gpio_device_create(devs->vmem + REG_ID, NULL,
			"pld_id");
	if (!devs->pld_id)
		goto err_register;

	devs->id = ioread8(devs->vmem + REG_ID);
	return 0;

err_register:
	icb_g2l_delete();
	return -ENOMEM;
}

static int __init icb_g2l_pld_init(void)
{
	int ret = 0;

	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION 
			" loading\n");
	if (!(devs = kzalloc(sizeof(struct g2l_devices), GFP_KERNEL)))
		return -ENOMEM;
	if (mem == 0) {
		printk(KERN_ERR "Invalid memory address\n");
		ret = -EINVAL;
		goto out_free;
	}
	if (!request_mem_region(mem + GPIO_BASE, MAP_SIZE, DRV_MODULE_NAME)) {
		ret = -EBUSY;
		goto out_free;
	}
	if (!(devs->vmem = ioremap_nocache(mem + GPIO_BASE, MAP_SIZE))) {
		ret = -EBUSY;
		goto out_release;
	}
	if ((ret = icb_g2l_register()) != 0)
		goto out_unmap;

	printk("ICB G2L PLD ID 0x%X mapped at 0x%p\n", devs->id, (void *)(mem +
			GPIO_BASE));
	return 0;

out_unmap:
	iounmap(devs->vmem);
out_release:
	release_mem_region(mem + GPIO_BASE, MAP_SIZE);
out_free:
	kfree(devs);
	return ret;
}

static void __exit icb_g2l_pld_exit(void)
{
	printk("Removing ICB G2 PLD map\n");
	icb_g2l_delete();
	iounmap(devs->vmem);
	release_mem_region(mem + GPIO_BASE, MAP_SIZE);
	kfree(devs);
}

module_init(icb_g2l_pld_init);
module_exit(icb_g2l_pld_exit);

MODULE_AUTHOR("EMAC, Inc.");
MODULE_LICENSE("GPL");

