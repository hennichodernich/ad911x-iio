// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD911x Digital to Analog Converter driver.
 *
 * Copyright 2023 Henning Paul
 *  Author: Henning Paul <hnch@gmx.net>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD911X_READ_OPERATION 0x80
#define AD911X_AUXDAC_ADDR(chan) (0x09+(chan*2))
#define AD911X_AUXDAC_ENABLE 0x80
#define AD911X_AUXDAC_TOP1V 0x0
#define AD911X_AUXDAC_TOP2V 0x2
#define AD911X_AUXDAC_RANGE2V 0x0
#define AD911X_GAINCTL_ADDR(chan) (0x03+(chan*3))
#define AD911X_INTERNAL_RSET_ENABLE 0x80
#define AD911X_CLKMODE_ADDR 0x14
#define AD911X_CLKMODE_ENABLE 0x04

#undef AD911X_USE_BIDI_SPI

/**
 * struct ad911x_chip_info - chip specific information
 * @channels:		Channel specification
 */
struct ad911x_chip_info {
	const struct iio_chan_spec *channels;
};

/**
 * struct ad911x - driver instance specific data
 * @spi:		the SPI device for this driver instance
 * @chip_info:		chip model specific constants, available modes etc
 * @dac_cache:		Cache for the DAC values
 * @data:		spi transfer buffers
 */
struct ad911x {
	struct spi_device		*spi;
	const struct ad911x_chip_info	*chip_info;

	uint16_t dac_cache[2];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__u8 data[2] ____cacheline_aligned;
	__u8 shadowregs[32];
};

enum ad911x_type {
	ID_AD911X,
};

static int ad911x_write(struct iio_dev *indio_dev, u8 addr,	u8 val)
{
	struct ad911x *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	st->data[0] = (addr & 0x1F) & (~AD911X_READ_OPERATION);
	st->data[1] = val & 0xFF;
	ret = spi_write_then_read(st->spi, st->data, 2, NULL, 0);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

#ifdef AD911X_USE_BIDI_SPI
static int ad911x_read(struct iio_dev *indio_dev, u8 addr, u8 *val)
{
	struct ad911x *st = iio_priv(indio_dev);
	int ret;
	u8 tmp;

	mutex_lock(&indio_dev->mlock);
	st->data[0] = (addr & 0x1F) | AD911X_READ_OPERATION;
	ret = spi_write_then_read(st->spi, st->data, 1, &tmp, 1);
	if (ret < 0)
		goto out_unlock;

	*val = tmp;

out_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
#endif

static int ad911x_write_auxdac(struct iio_dev *indio_dev, unsigned int chan, unsigned int val)
{
	struct ad911x *st = iio_priv(indio_dev);
	u8 ctrl_byte;
	int ret;

#ifdef AD911X_USE_BIDI_SPI
	ret = ad911x_read(indio_dev, AD911X_AUXDAC_ADDR(chan) + 1, &ctrl_byte);
	if (ret)
		return ret;
#else
	ctrl_byte = st->shadowregs[AD911X_AUXDAC_ADDR(chan) + 1];
#endif
	ctrl_byte |= AD911X_AUXDAC_ENABLE | (AD911X_AUXDAC_RANGE2V << 5) | (AD911X_AUXDAC_TOP2V << 2);
	ctrl_byte &= ~0x03;
	ctrl_byte |= ((val >> 8) & 0x03);
	ret = ad911x_write(indio_dev, AD911X_AUXDAC_ADDR(chan) + 1, ctrl_byte);
	if (ret)
		return ret;
	st->shadowregs[AD911X_AUXDAC_ADDR(chan) + 1] = ctrl_byte;
	ret = ad911x_write(indio_dev, AD911X_AUXDAC_ADDR(chan), val & 0xFF);
	if (ret)
		return ret;
	st->shadowregs[AD911X_AUXDAC_ADDR(chan)] = val & 0xFF;
	return 0;
}

static int ad911x_read_auxdac(struct iio_dev *indio_dev, unsigned int chan, unsigned int *val)
{
	struct ad911x *st = iio_priv(indio_dev);
	u8 ctrl_byte, tmp;
	int ret;

#ifdef AD911X_USE_BIDI_SPI
	ret = ad911x_read(indio_dev, AD911X_AUXDAC_ADDR(chan) + 1, &ctrl_byte);
	if (ret)
		return ret;
	ret = ad911x_read(indio_dev, AD911X_AUXDAC_ADDR(chan), &tmp);
	if (ret)
		return ret;
#else
	ctrl_byte = st->shadowregs[AD911X_AUXDAC_ADDR(chan) + 1];
	tmp = st->shadowregs[AD911X_AUXDAC_ADDR(chan)];
#endif

	*val = ((ctrl_byte & 0x03) << 8) | tmp;
	return 0;
}

static int ad911x_write_dacgain(struct iio_dev *indio_dev, unsigned int chan, unsigned int val)
{
	struct ad911x *st = iio_priv(indio_dev);
	int ret;

	ret = ad911x_write(indio_dev, AD911X_GAINCTL_ADDR(chan), val & 0x3F);
	if (ret)
		return ret;
	st->shadowregs[AD911X_GAINCTL_ADDR(chan)] = val & 0x3F;
	return 0;
}

static int ad911x_read_dacgain(struct iio_dev *indio_dev, unsigned int chan, unsigned int *val)
{
	struct ad911x *st = iio_priv(indio_dev);
	u8 tmp;
	int ret;

#ifdef AD911X_USE_BIDI_SPI
	ret = ad911x_read(indio_dev, AD911X_GAINCTL_ADDR(chan), &tmp);
	if (ret)
		return ret;
#else
	tmp = st->shadowregs[AD911X_GAINCTL_ADDR(chan)];
#endif

	*val = tmp & 0x3F;
	return 0;
}


static int ad911x_write_rset(struct iio_dev *indio_dev, unsigned int chan, unsigned int val)
{
	struct ad911x *st = iio_priv(indio_dev);
	int ret;

	ad911x_write(indio_dev, AD911X_GAINCTL_ADDR(chan) + 1, AD911X_INTERNAL_RSET_ENABLE | (val & 0x3F));
	if (ret)
		return ret;
	st->shadowregs[AD911X_GAINCTL_ADDR(chan) + 1] = AD911X_INTERNAL_RSET_ENABLE | (val & 0x3F);
	return 0;
}

static int ad911x_read_rset(struct iio_dev *indio_dev, unsigned int chan, unsigned int *val)
{
	struct ad911x *st = iio_priv(indio_dev);
	u8 tmp;
	int ret;

#ifdef AD911X_USE_BIDI_SPI
	ret = ad911x_read(indio_dev, AD911X_GAINCTL_ADDR(chan) + 1, &tmp);
	if (ret)
		return ret;
#else
	tmp = st->shadowregs[AD911X_GAINCTL_ADDR(chan) + 1];
#endif

	*val = tmp & 0x3F;
	return 0;
}


static int ad911x_fill_shadowregs(struct iio_dev *indio_dev)
{
	struct ad911x *st = iio_priv(indio_dev);
	memset(st->shadowregs, 0x00, 32);
	st->shadowregs[0x01] = 0x40;
	st->shadowregs[0x02] = 0x34;
	st->shadowregs[0x11] = 0x3F;
	st->shadowregs[0x1F] = 0x0A;

	return 0;
}

static int ad911x_init_registers(struct iio_dev *indio_dev)
{
	struct ad911x *st = iio_priv(indio_dev);
	u8 ctrl_byte;
	int ret, ii;

	for(ii=0; ii<2; ii++)
	{
		ret = ad911x_write(indio_dev, AD911X_GAINCTL_ADDR(ii) + 1, AD911X_INTERNAL_RSET_ENABLE | 0x20);
		if (ret)
			return ret;
		st->shadowregs[AD911X_GAINCTL_ADDR(ii) + 1] = AD911X_INTERNAL_RSET_ENABLE | 0x20;
	}
	ret = ad911x_write(indio_dev, AD911X_CLKMODE_ADDR, AD911X_CLKMODE_ENABLE | 0x00);
	if (ret)
		return ret;
	st->shadowregs[AD911X_CLKMODE_ADDR] = AD911X_CLKMODE_ENABLE | 0x00;

	return 0;
}


static int ad911x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad911x *st = iio_priv(indio_dev);
	struct regulator_bulk_data *reg;
	int scale_uv;
	int ret;
	unsigned int tmp;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad911x_read_auxdac(indio_dev, chan->address, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ad911x_read_dacgain(indio_dev, chan->address, &tmp);
		if (ret)
			return ret;
		*val = 1;
		*val2 = tmp*675;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ad911x_read_rset(indio_dev, chan->address, &tmp);
		if (ret)
			return ret;
		tmp -= 32;
		tmp *= 62500;
		*val = tmp/1000000;
		*val2 = tmp - ((*val)*1000000);
		if (*val==0)
			*val2 *= -1;
		else
			*val *= -1;
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_SCALE:
	default:
		break;
	}

	return -EINVAL;
}

static int ad911x_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct ad911x *st = iio_priv(indio_dev);
	int ret, tmp;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if ((val < 0) || (val >= (2<<10)))
			return -EINVAL;
		ret = ad911x_write_auxdac(indio_dev, chan->address, val);
		if (ret == 0)
			st->dac_cache[chan->address] = val;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val != 1)
			return -EINVAL;
		if (val2 > 42525)
			return -EINVAL;
		ret = ad911x_write_dacgain(indio_dev, chan->address, (val2/675));
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val == 0)
		{
			if (val2 > 0)
				return -EINVAL;
			else	// val2 is negative
			{
				tmp = val2 / -62500;
			}
		}
		else if (val == -1)
		{
			if (val2 > 0)
			{
				tmp = val2 / 62500;
			}
			else
			{
				tmp = val2 / -62500;
			}
			tmp += 16;	// absolute value was >=1dB
		}
		else
			return -EINVAL;
		if (tmp > 31)
			tmp = 31;
		tmp += 32;

		ret = ad911x_write_rset(indio_dev, chan->address, tmp);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info ad911x_info = {
	.read_raw = ad911x_read_raw,
	.write_raw = ad911x_write_raw,
};

#define AD911X_CHANNEL(chan, bits) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_CALIBSCALE) | BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
	.address = (chan),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (bits),				\
		.storagebits = 16,				\
		.shift = 0,				\
	},							\
}

#define DECLARE_AD911X_CHANNELS(name, bits) \
const struct iio_chan_spec name[] = { \
	AD911X_CHANNEL(0, bits), \
	AD911X_CHANNEL(1, bits), \
}

static DECLARE_AD911X_CHANNELS(ad911x_channels, 10);

static const struct ad911x_chip_info ad911x_chip_info[] = {
	[ID_AD911X] = {
		.channels = ad911x_channels,
	},
};

static int ad911x_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct ad911x *st;
	unsigned int i;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->chip_info = &ad911x_chip_info[id->driver_data];
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = id->name;
	indio_dev->info = &ad911x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = 2;

	ad911x_fill_shadowregs(indio_dev);
	ad911x_init_registers(indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	return 0;
}

static int ad911x_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad911x *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ad911x_spi_ids[] = {
	{ "ad911x", ID_AD911X },
	{}
};
MODULE_DEVICE_TABLE(spi, ad911x_spi_ids);

static struct spi_driver ad911x_spi_driver = {
	.driver = {
		.name = "ad911x",
	},
	.probe = ad911x_spi_probe,
	.remove = ad911x_spi_remove,
	.id_table = ad911x_spi_ids,
};
module_spi_driver(ad911x_spi_driver);

MODULE_AUTHOR("Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("Analog Devices AD911x DACs");
MODULE_LICENSE("GPL v2");
