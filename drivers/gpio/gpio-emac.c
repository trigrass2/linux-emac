/*
 * Driver for EMAC CPLD based GPIO
 *
 * Copyright (C) 2015. 2016 EMAC Inc.
 * Copyright (C) 2016 QWERTY Embedded Design, LLC
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>

static const struct of_device_id emac_gpio_ids[] = {
	{
		.compatible	= "emac,cpld-gpio",
		.data		= (void *)0,
	},
	{
		.compatible	= "emac,cpld-gpo",
		.data		= (void *)1,
	},
	{
		.compatible	= "emac,gpi",
		.data		= (void *)2,
	},
	{
		.compatible	= "emac,gpo",
		.data		= (void *)3,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, emac_gpio_ids);

static int __init emac_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct gpio_chip *gc;
	void __iomem *regs;
	void __iomem *dat_reg = NULL;
	void __iomem *dir_reg = NULL;
	int port_type;
	int ret;
	int flags = 0;

	of_id = of_match_device(emac_gpio_ids, &pdev->dev);
	port_type = (int) of_id->data;

	gc = devm_kzalloc(&pdev->dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;

	regs = of_iomap(pdev->dev.of_node, 0);
	if (!regs)
		return -ENOMEM;

	if (port_type == 0) {
		dat_reg = regs;
		dir_reg = regs+1;
	}
	else if (port_type == 1) {
		dat_reg = regs;
	}
	else if (port_type == 2) {
		dat_reg = regs;
		flags |= BGPIOF_NO_OUTPUT;
	}
	else if (port_type == 3) {
		dat_reg = regs;
	}

	ret = bgpio_init(gc, &pdev->dev,
			 1, dat_reg, NULL,
			 NULL, dir_reg, NULL,
			 flags);

	if (ret) {
		dev_err(&pdev->dev, "bgpio_init failed\n");
		goto err0;
	}

	/* Setup pointers to chip functions */
	gc->label = devm_kstrdup(&pdev->dev, pdev->dev.of_node->full_name,
				     GFP_KERNEL);
	if (!gc->label) {
		ret = -ENOMEM;
		goto err0;
	}

	gc->base = -1;
	gc->ngpio = 8;
	gc->of_gpio_n_cells = 2;
	gc->of_node = pdev->dev.of_node;

	/* This function adds a memory mapped GPIO chip */
	ret = devm_gpiochip_add_data(&pdev->dev, gc, NULL);
	if (ret)
		goto err0;

	return 0;
err0:
	iounmap(regs);
	pr_err("%s: GPIO chip registration failed\n",
			pdev->dev.of_node->full_name);
	return ret;
}

static int emac_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver emac_gpio_driver = {
	.driver = {
		.name		= "emac-gpio",
		.of_match_table	= emac_gpio_ids,
	},
	.probe = emac_gpio_probe,
	.remove = emac_gpio_remove,
};

module_platform_driver(emac_gpio_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("EMAC CPLD GPIO driver");
MODULE_LICENSE("GPL");
