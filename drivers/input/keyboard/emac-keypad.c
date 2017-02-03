/*
 * linux/drivers/input/keyboard/emac-keypad.c
 *
 * EMAC Inc. CPLD Keypad Driver
 *
 * Copyright (C) 2014 EMAC Inc.
 * Written by Michael Welling <mwelling@ieee.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

struct emac_keypad {
	struct input_dev *input;
	void __iomem *base;
	unsigned int	nrows;
	unsigned int	ncols;
	unsigned int	row_shift;
	unsigned int	irq;
	unsigned int	lcode;
	bool		wakeup;
	unsigned short	*keymap;
	struct clk	*cpldclk;
};

static irqreturn_t emac_keypad_interrupt(int irq, void *dev_id)
{
	struct emac_keypad *keypad_data = dev_id;
	struct input_dev *input_dev = keypad_data->input;
	unsigned int row, col;
	unsigned int code;
	unsigned int pressed;
	unsigned int raw_code;

	raw_code = __raw_readb(keypad_data->base);
	row = raw_code & 0x03;
	col = raw_code >> 4;
	pressed = (raw_code >> 2) & 0x01;

	code = MATRIX_SCAN_CODE(row, col, keypad_data->row_shift);

	input_event(input_dev, EV_MSC, MSC_SCAN, code);

	if (pressed) {
		input_report_key(input_dev, keypad_data->keymap[code], 1);
		keypad_data->lcode = code;
	}
	else
		input_report_key(input_dev,
				 keypad_data->keymap[keypad_data->lcode], 0);

	input_sync(input_dev);

	return IRQ_HANDLED;
}

static int emac_keypad_probe(struct platform_device *pdev)
{
	struct resource *res;
	const struct matrix_keymap_data *keymap_data = NULL;
	struct device *dev = &pdev->dev;
	struct emac_keypad *keypad_data;
	struct input_dev *input_dev;
	size_t max_keys;
	int gpio_irq;
	int error;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory resource\n");
		return -EINVAL;
	}

	keypad_data = devm_kzalloc(dev, sizeof(struct emac_keypad), GFP_KERNEL);
	if (!keypad_data) {
		dev_err(dev, "keypad_data memory allocation failed\n");
		return -ENOMEM;
	}

	error = matrix_keypad_parse_of_params(dev, &keypad_data->nrows,
					    &keypad_data->ncols);

	if (error)
		return error;

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(dev, "can't request mem region\n");
		return -EBUSY;
	}

	keypad_data->base = ioremap(res->start, resource_size(res));
	if (!keypad_data->base) {
		dev_err(dev, "can't ioremap mem resource\n");
		error = -ENOMEM;
		goto err_release_mem;
	}

	keypad_data->input = input_dev = input_allocate_device();
	if (!input_dev) {
		error = -ENOMEM;
		goto err_unmap;
	}

	keypad_data->cpldclk = devm_clk_get(dev, NULL);
	if (IS_ERR(keypad_data->cpldclk)) {
		dev_err(dev, "can't get clk\n");
		error = PTR_ERR(keypad_data->cpldclk);
		goto err_free_input;
	}

	clk_prepare_enable(keypad_data->cpldclk);

	input_dev->name = pdev->name;
	input_dev->dev.parent = &pdev->dev;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_set_drvdata(input_dev, keypad_data);

	keypad_data->row_shift = get_count_order(keypad_data->ncols);
	max_keys = keypad_data->nrows << keypad_data->row_shift;

	keypad_data->keymap = devm_kzalloc(dev,
			max_keys * sizeof(keypad_data->keymap[0]),
			GFP_KERNEL);

	if (!keypad_data->keymap) {
		dev_err(dev, "keymap memory allocation failed\n");
		error = -ENOMEM;
		goto err_free_input;
	}

	error = matrix_keypad_build_keymap(keymap_data, NULL,
					   keypad_data->nrows,
					   keypad_data->ncols,
					   keypad_data->keymap, input_dev);
	if (error) {
		dev_err(dev, "failed to build keymap\n");
		goto err_free_keymap;
	}

	gpio_irq = of_get_named_gpio(dev->of_node, "gpio-interrupt", 0);

	if (!gpio_is_valid(gpio_irq)) {
		dev_err(dev, "invalid interrupt gpio\n");
		error = -EINVAL;
		goto err_free_keymap;
	}

	error = gpio_request(gpio_irq, "cpld-irq");
	if (error) {
		dev_err(dev, "failed to request gpio\n");
		goto err_free_keymap;
	}

	keypad_data->irq = gpio_to_irq(gpio_irq);

	error = request_irq(keypad_data->irq, emac_keypad_interrupt,
		     IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		     "emac-keypad", keypad_data);
	if (error) {
		dev_err(dev, "failed to register interrupt\n");
		goto err_free_keymap;
	}

	error = input_register_device(keypad_data->input);
	if (error < 0) {
		dev_err(dev, "failed to register input device\n");
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, keypad_data);

	return 0;

err_free_irq:
	free_irq(keypad_data->irq, keypad_data);
err_free_keymap:
	kfree(keypad_data->keymap);
err_free_input:
	input_free_device(input_dev);
err_unmap:
	iounmap(keypad_data->base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
	return error;
}

static int emac_keypad_remove(struct platform_device *pdev)
{
	struct emac_keypad *keypad_data = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(keypad_data->irq, keypad_data);

	input_unregister_device(keypad_data->input);

	iounmap(keypad_data->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(keypad_data->keymap);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id emac_keypad_of_match[] = {
	{ .compatible = "emac,emac-keypad" },
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static int emac_keypad_suspend(struct device * dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct emac_keypad *keypad_data = platform_get_drvdata(pdev);

	/* shut off the clock */
	clk_disable(keypad_data->cpldclk);
	return 0;
}

static int emac_keypad_resume(struct device * dev)
{
	struct platform_device *pdev = to_platform_device(dev);
        struct emac_keypad *keypad_data = platform_get_drvdata(pdev);

	/* turn on the clock */
	clk_enable(keypad_data->cpldclk);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(emac_keypad_pm_ops,
			 emac_keypad_suspend, emac_keypad_resume);

static struct platform_driver emac_keypad_device_driver = {
	.probe		= emac_keypad_probe,
	.remove		= emac_keypad_remove,
	.driver		= {
		.name	= "emac-keypad",
		.owner	= THIS_MODULE,
		.pm	= &emac_keypad_pm_ops,
		.of_match_table = of_match_ptr(emac_keypad_of_match),
	}
};

module_platform_driver(emac_keypad_device_driver);

MODULE_AUTHOR("EMAC Inc.");
MODULE_DESCRIPTION("EMAC CPLD Keypad Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:emac-keypad");
