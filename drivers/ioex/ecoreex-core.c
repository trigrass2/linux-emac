/**
 * drivers/ioex/ecoreex-core.c
 *
 * This file provides PLD/FPGA memory mapped IO utility for EMAC carriers.
 *
 * Copyright (C) 2007-2013 EMAC, Inc. <support@emacinc.com>
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
#include <linux/of.h>
#include <linux/of_device.h>

#include "ecoreex-core.h"

static int ecoreex_probe(struct platform_device *pdev)
{
	int i;
	struct ecoreex_data * data = (struct ecoreex_data *) pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	u32 key_offset = 0xf;

	if (pdev == NULL)
		return -ENODEV;

	if (np)
		of_property_read_u32(np, "key_offset", &key_offset);

	if (data == NULL) {
		data = kmalloc(sizeof(struct ecoreex_data) * pdev->num_resources, GFP_KERNEL);
		memset(data, 0, sizeof(struct ecoreex_data) * pdev->num_resources);

		for(i = 0; i < pdev->num_resources; i++)
			data[i].key_offset = key_offset;
	}

	for(i = 0; i < pdev->num_resources; i++)
		map_core(pdev->resource[i].start,
			 (pdev->resource[i].end - pdev->resource[i].start + 1),
			 &data[i]);

	return 0;
}

static int ecoreex_remove(struct platform_device *pdev)
{
	int i;
	struct ecoreex_data *edata = pdev->dev.platform_data;

	if (edata->plat_device) {
		platform_device_unregister(edata->plat_device);
		kfree(edata->plat_device);
	}
	if (edata->devices) {
		for (i = 0; i < edata->num_devices; i++)
			if (edata->devices[i])
				gpio_erase(edata->devices[i]);
		kfree(edata->devices);
	}
	if (edata->virt_addr) {
		release_mem_region(edata->phys_addr, edata->map_size);
		iounmap(edata->virt_addr);
	}

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id ecoreex_dt_ids[] = {
	{ .compatible = "ecoreex" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ecoreex_dt_ids);
#endif

static struct platform_driver ecoreex_driver = {
	.probe = ecoreex_probe,
	.remove = ecoreex_remove,
	.driver = {
		.name = "ecoreex",
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(ecoreex_dt_ids),
#endif
	},
};

#define DRV_MODULE_NAME 	"ecoreex"
#define DRV_MODULE_VERSION 	"2.0"

static int __init ecoreex_init_module(void)
{
	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION
	       " loading\n");
	return platform_driver_register(&ecoreex_driver);
}

static void __exit ecoreex_cleanup_module(void)
{
	platform_driver_unregister(&ecoreex_driver);
}

module_init(ecoreex_init_module);
module_exit(ecoreex_cleanup_module);

MODULE_AUTHOR("EMAC Inc.");
MODULE_ALIAS("platform:ecoreex");
