/*
 * IIO driver for the memory mapped CPLD counter
 *
 * Copyright (C) 2016 EMAC Inc.
 * Copyright (C) 2016 QWERTY Embedded Design, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/module.h>

struct emac_counter_iio {
	void __iomem *base;
	unsigned int scale;
};

static int emac_counter_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct emac_counter_iio *const priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = readb(priv->base + 1) << 8;
		*val |= readb(priv->base);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = priv->scale;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static const struct iio_info emac_counter_info = {
	.driver_module = THIS_MODULE,
	.read_raw = emac_counter_read_raw
};

static const struct of_device_id emac_counter_ids[] = {
	{
		.compatible	= "emac,counter16",
		.data		= (void *)0,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, emac_counter_ids);

#define CPLD_COUNT_CHAN(_chan) {					\
	.type = IIO_COUNT,						\
	.channel = (_chan),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_SCALE),\
	.indexed = 1							\
}

static const struct iio_chan_spec cpld_channels[] = {
	CPLD_COUNT_CHAN(0)
};

static int __init emac_counter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct emac_counter_iio *priv;
	void __iomem *regs;
	struct clk *clk = NULL;
	int ret;

	clk = devm_clk_get(dev, "cntclk");
	if (IS_ERR(clk))
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	regs = of_iomap(pdev->dev.of_node, 0);
	if (!regs)
		return -ENOMEM;

	indio_dev->info = &emac_counter_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(cpld_channels);
	indio_dev->channels = cpld_channels;
	indio_dev->name = dev_name(dev);

	priv = iio_priv(indio_dev);
	priv->base = regs;

	return devm_iio_device_register(dev, indio_dev);
}

static struct platform_driver emac_counter_driver = {
	.driver = {
		.name		= "emac-counter",
		.of_match_table	= emac_counter_ids,
	},
	.probe = emac_counter_probe,
};

module_platform_driver(emac_counter_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("EMAC CPLD Counter driver");
MODULE_LICENSE("GPL v2");
