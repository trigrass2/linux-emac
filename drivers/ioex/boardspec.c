/**
 * drivers/ioex/boardspec.c
 *
 * Registration of board specific classes & devices.
 * Registers and calls a function pointer from the arch's platform device.
 * Important in that it allows for a proper module loading order, which may be
 * required as some methods call other modules (specifically Xenomai)
 *
 * Copyright (C) 2007-2015 EMAC, Inc. <support@emacinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/ioex/ecoreex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define DRV_MODULE_NAME		"boardspec"
#define DRV_MODULE_VERSION	"2.1"

#if defined(CONFIG_OF)
static const struct of_device_id boardspec_dt_ids[] = {
	{
		.compatible = "boardspec",
		.data = NULL,
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, boardspec_dt_ids);
#endif

static gpio_data gpio_data_read(struct gpio_s *gpio)
{
	gpio_data value;
	int i;
	value = 0;

	for(i = 0; i < gpio->gpio_cnt; i++)
		value |= (gpio_get_value(gpio->gpio_set[i]) << i);

	return value;
}

static int gpio_data_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	for(i = 0; i < gpio->gpio_cnt; i++)
		gpio_set_value(gpio->gpio_set[i], (data >> i) & 1);

	return 0;
}

static gpio_data gpio_ddr_read(struct gpio_s *gpio)
{
	return gpio->ddr_shadow;
}

static int gpio_ddr_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	gpio->ddr_shadow = data;

	for(i = 0; i < gpio->gpio_cnt; i++)
	{
		if (data & (1 << i))
			gpio_direction_output(gpio->gpio_set[i], 0);
		else
			gpio_direction_input(gpio->gpio_set[i]);
	}
	return 0;
}

struct device * gpio_class_create(char * name, int * gpio_array, int gpio_count,
				  int direction, int value)
{
	gpio_t *gpio = kzalloc(sizeof(gpio_t), GFP_KERNEL);

	if (gpio == NULL)
		return NULL;

	gpio->name = name;
	gpio->data_write = gpio_data_write;
	gpio->data_read  = gpio_data_read;
	gpio->ddr_write  = gpio_ddr_write;
	gpio->ddr_read   = gpio_ddr_read;
	gpio->gpio_set   = gpio_array;
	gpio->gpio_cnt   = gpio_count;

	gpio_ddr_write(gpio, direction);
	gpio_data_write(gpio, value);

	pr_info("registering gpio device: %s\n", gpio->name);
	return gpio_register_device(gpio);
}

int boardspec_generic_init_of(struct device_node * np)
{
	struct property * prop;
	const void * tprop;
	int *curr_gpio;
	int curr_count;
	int direction = 0;
	int value = 0;
	int i;
	char tmpstr[128];

	for_each_property_of_node(np, prop)
	{
		if (strstr(prop->name, "gpio-dir") || strstr(prop->name, "gpio-val"))
			continue;
		else if (strstr(prop->name, "gpio")) {
			curr_count = of_gpio_named_count(np, prop->name);

			curr_gpio = kzalloc(sizeof(int)*curr_count, GFP_KERNEL);
			if (curr_gpio == NULL)
				return -ENOMEM;

			for(i = 0; i < curr_count; i++)
			{
				curr_gpio[i] = of_get_named_gpio(np, prop->name, i);

				if (!gpio_is_valid(curr_gpio[i]) ||
				    !(gpio_request(curr_gpio[i], prop->name) == 0)) {
					pr_err("invalid GPIO specified\n");
				}
			}

			sprintf(tmpstr, "%.*s-dir", 123, prop->name);
			tprop = of_get_property(np, tmpstr, NULL);
			if (tprop)
				direction = of_read_ulong(tprop, 1);

			sprintf(tmpstr, "%.*s-val", 123, prop->name);
			tprop = of_get_property(np, tmpstr, NULL);
			if (tprop)
				value = of_read_ulong(tprop, 1);

			if (gpio_class_create(prop->name, curr_gpio, curr_count,
					     direction, value) == NULL)
				return -ENOMEM;
		}

	}

	return 0;
}

static int boardspec_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int (*device_init_of) (struct device_node * np);
	int (*device_init) (void);
	struct device_node *np;

	if (!pdev)
		return -ENODEV;

	np = pdev->dev.of_node;

	if (np)	{
		match = of_match_device(boardspec_dt_ids, &pdev->dev);

		if (match) {
			device_init_of = match->data;
			if (device_init_of)
				return device_init_of(np);
			else
				return boardspec_generic_init_of(np);
		}
	}

	if (pdev->dev.platform_data) {
		device_init = pdev->dev.platform_data;
		return device_init();
	}

	return -ENODEV;
}

static int boardspec_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver boardspec_driver = {
	.probe = boardspec_probe,
	.remove = boardspec_remove,
	.driver = {
		   .name = "boardspec",
		   .owner = THIS_MODULE,
#if defined(CONFIG_OF)
		   .of_match_table = of_match_ptr(boardspec_dt_ids),
#endif
		   },
};

static int __init boardspec_init_module(void)
{
	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION
	       " loading\n");
	return platform_driver_register(&boardspec_driver);
}

static void __exit boardspec_cleanup_module(void)
{
	platform_driver_unregister(&boardspec_driver);
}

module_init(boardspec_init_module);
module_exit(boardspec_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("EMAC Inc.");
MODULE_ALIAS("platform:boardspec");
