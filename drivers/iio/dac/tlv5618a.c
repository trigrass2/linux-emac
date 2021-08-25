/*
 * Driver for Texas Instruments TLV5618A Digital to Analog Converter
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/bitops.h>

#define TLV5618A_NUM_CHAN	2

#define TLV5618A_R1		(1 << 15)
#define TLV5618A_R0		(1 << 12)
#define TLV5618A_SPD_FAST	(1 << 14)
#define TLV5618A_PWR_OFF	(1 << 13)


enum tlv5618a_supported_device_ids {
	ID_TLV5618A,
};

struct tlv5618a_state {
	struct spi_device *spi;
	unsigned int value[TLV5618A_NUM_CHAN];
	unsigned int vref_mv;
	struct regulator *vref_reg;
	u8 mosi[2] ____cacheline_aligned;
};

#define TLV5618A_CHAN(chan, bits) {			\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = chan,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = (bits),			\
		.storagebits = 16,			\
		.shift = 12 - (bits),			\
	},						\
}

static int tlv5618a_spi_write(struct tlv5618a_state *state, u8 addr, u32 val)
{
	u16 data = TLV5618A_SPD_FAST;

	if (addr == 0)
		data |= TLV5618A_R1;

	data |= val;

	state->mosi[0] = data >> 8;
	state->mosi[1] = data & 0xff;

	return spi_write(state->spi, state->mosi, 2);
}

static int tlv5618a_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask)
{
	struct tlv5618a_state *state = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = state->value[chan->channel];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = state->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int tlv5618a_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int val,
		int val2,
		long mask)
{
	struct tlv5618a_state *state = iio_priv(indio_dev);

	if (val2 != 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > GENMASK(chan->scan_type.realbits-1, 0))
			return -EINVAL;
		val <<= chan->scan_type.shift;
		state->value[chan->channel] = val;
		return tlv5618a_spi_write(state, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec tlv5618a_channels[3][TLV5618A_NUM_CHAN] = {
	[ID_TLV5618A] = { TLV5618A_CHAN(0, 12),	TLV5618A_CHAN(1, 12) },
};

static const struct iio_info tlv5618a_info = {
	.read_raw = &tlv5618a_read_raw,
	.write_raw = &tlv5618a_write_raw,
};

static int tlv5618a_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct tlv5618a_state *state;
	const struct spi_device_id *id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));
	if (indio_dev == NULL)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	state->spi = spi;
	state->vref_reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(state->vref_reg)) {
		dev_err(&spi->dev, "Vref regulator not specified\n");
		return PTR_ERR(state->vref_reg);
	}

	ret = regulator_enable(state->vref_reg);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable vref regulator: %d\n",
				ret);
		return ret;
	}

	ret = regulator_get_voltage(state->vref_reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to read vref regulator: %d\n",
				ret);
		goto error_disable_reg;
	}
	state->vref_mv = ret / 1000;

	spi_set_drvdata(spi, indio_dev);
	id = spi_get_device_id(spi);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &tlv5618a_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = tlv5618a_channels[id->driver_data];
	indio_dev->num_channels = TLV5618A_NUM_CHAN;
	indio_dev->name = id->name;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device: %d\n",
				ret);
		goto error_disable_reg;
	}

	return 0;

error_disable_reg:
	regulator_disable(state->vref_reg);

	return ret;
}

static int tlv5618a_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct tlv5618a_state *state;

	iio_device_unregister(indio_dev);
	state = iio_priv(indio_dev);
	regulator_disable(state->vref_reg);

	return 0;
}

static const struct spi_device_id tlv5618a_id[] = {
	{"tlv5618a", ID_TLV5618A},
	{}
};
MODULE_DEVICE_TABLE(spi, tlv5618a_id);

static struct spi_driver tlv5618a_driver = {
	.driver = {
		   .name = "tlv5618a",
		   },
	.probe = tlv5618a_probe,
	.remove = tlv5618a_remove,
	.id_table = tlv5618a_id,
};
module_spi_driver(tlv5618a_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("Texas Instruments TLC5618A DAC");
MODULE_LICENSE("GPL v2");
