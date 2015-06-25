/*
 * Driver for EMAC CPLD based GPIO
 *
 * Author: Michael Welling <mwelling@ieee.org>
 *
 * 2015 (c) EMAC Inc.
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
#include <linux/basic_mmio_gpio.h>

static const struct of_device_id emac_gpio_ids[] = {
	{
		.compatible	= "emac,cpld-gpio",
		.data		= (void *)0,
	},
	{
		.compatible	= "emac,cpld-gpo",
		.data		= (void *)1,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, emac_gpio_ids);

static int __init emac_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct bgpio_chip *bgc;
	void __iomem *regs;
	void __iomem *dir_reg = NULL;
	int port_type;
	int ret;

	of_id = of_match_device(emac_gpio_ids, &pdev->dev);
	port_type = (int) of_id->data;

	bgc = devm_kzalloc(&pdev->dev, sizeof(*bgc), GFP_KERNEL);
	if (!bgc)
		return -ENOMEM;

	regs = of_iomap(pdev->dev.of_node, 0);
	if (!regs)
		return -ENOMEM;

	if (port_type == 0)
		dir_reg = regs+1;

	ret = bgpio_init(bgc, &pdev->dev, 1, regs, NULL, NULL, dir_reg, NULL,
			BGPIOF_BIG_ENDIAN_BYTE_ORDER);

	if (ret) {
		dev_err(&pdev->dev, "bgpio_init failed\n");
		goto err0;
	}

	/* Setup pointers to chip functions */
	bgc->gc.label = devm_kstrdup(&pdev->dev, pdev->dev.of_node->full_name,
				     GFP_KERNEL);
	if (!bgc->gc.label) {
		ret = -ENOMEM;
		goto err0;
	}

	bgc->gc.base = -1;
	bgc->gc.ngpio = 8;
	bgc->gc.of_gpio_n_cells = 2;
	bgc->gc.of_node = pdev->dev.of_node;

	/* This function adds a memory mapped GPIO chip */
	ret = gpiochip_add(&bgc->gc);
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

MODULE_DESCRIPTION("EMAC CPLD GPIO driver");
MODULE_AUTHOR("Michael Welling <mwelling@emacinc.com>");
MODULE_LICENSE("GPL");
