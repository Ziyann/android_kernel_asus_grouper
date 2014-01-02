/*
 * ltr659ps.c -- LTR-659PS-01 proximity sensor driver
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <linux/ltr659ps.h>

#include "../iio.h"
#include "../sysfs.h"

#define DEVICE_NAME "ltr659ps"

/*
 * Debug Utility
 *
 */
/* TODO disable debug info when bring-up done */
#define PROX_SENSOR_DEBUG 1
#define PROX_SENSOR_VERBOSE_DEBUG 1

#if PROX_SENSOR_DEBUG
#define PROX_DEBUG(format, arg...)      \
	printk(KERN_INFO "LTR659PS: [%s] " format, __func__, ## arg)
#else
#define PROX_DEBUG(format, arg...)
#endif

#define PROX_INFO(format, arg...)       \
	printk(KERN_INFO "LTR659PS: [%s] " format, __func__, ## arg)
#define PROX_ERROR(format, arg...)      \
	printk(KERN_ERR "LTR659PS: [%s] " format, __func__, ## arg)

/*
 * FUNCTION DECLARATION
 */
struct ltr659ps_data;
static int __devinit ltr659ps_probe(
	struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit ltr659ps_remove(struct i2c_client *client);
static int ltr659ps_suspend(struct i2c_client *client, pm_message_t mesg);
static int ltr659ps_resume(struct i2c_client *client);
static s32 ltr659ps_read_reg(struct i2c_client *client, u8 command);
static s32 ltr659ps_write_reg(struct i2c_client *client, u8 command, u8 value);
static s32 ltr659ps_masked_write_reg(
	struct i2c_client *client, u8 command, u8 value, u8 mask);
static int __init ltr659ps_init(void);
static void __exit ltr659ps_exit(void);
static irqreturn_t ltr659ps_interrupt_handler(int irq, void *dev);
static void ltr659ps_work_function(struct work_struct *work);
static int ltr659ps_setup(struct ltr659ps_data *prox);
static int ltr659ps_init_sensor(struct i2c_client *client);
static int ltr659ps_check_id(struct i2c_client *client);
static int ltr659ps_config_irq(struct i2c_client *client);
static void ltr659ps_enable_sensor(struct i2c_client *client, int enable);
static void ltr659ps_report_input_event(struct ltr659ps_data *prox);
/*
 * I2C Driver Structure
 */
static const struct i2c_device_id ltr659ps_id[] = {
	{DEVICE_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ltr659ps_id);

/*
 * Register map
 */
#define LTR659PS_REG_SW_RESET 0x80
#define LTR659PS_REG_PS_CONTR 0x81
#define LTR659PS_REG_PS_LED 0x82
#define LTR659PS_REG_PS_N_PULSES 0x83
#define LTR659PS_REG_PS_MEAS_RATE 0x84

#define LTR659PS_REG_PART_ID 0x86
#define LTR659PS_PART_ID 0x92

#define LTR659PS_REG_MANUFAC_ID 0x87
#define LTR659PS_MANUFAC_ID 0x05

#define LTR659PS_REG_PS_STATUS 0x8c
#define LTR659PS_REG_PS_DATA_0 0x8d
#define LTR659PS_REG_PS_DATA_1 0x8e
#define LTR659PS_REG_INTERRUPT 0x8f
#define LTR659PS_REG_PS_THRES_UP_0 0x90
#define LTR659PS_REG_PS_THRES_UP_1 0x91
#define LTR659PS_REG_PS_THRES_LOW_0 0x92
#define LTR659PS_REG_PS_THRES_LOW_1 0x93
#define LTR659PS_REG_PS_OFFSET_1 0x94
#define LTR659PS_REG_PS_OFFSET_0 0x95
#define LTR659PS_REG_INTERRUPT_PERSIST 0x9e

#define PS_MIN_MEASURE_VAL      0
#define PS_MAX_MEASURE_VAL      2047

/*
 * Global Variable
 */
struct ltr659ps_data {
	struct workqueue_struct *prox_wq;

	struct i2c_client *client;
	struct input_dev *ps_input_dev;

	struct delayed_work work;

	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;

	int irq;
	int gpio;
	int enable;

	struct mutex prox_mtx;
};


static s32 ltr659ps_read_reg(struct i2c_client *client, u8 command)
{
	return i2c_smbus_read_byte_data(client, command);
}

static s32 ltr659ps_write_reg(struct i2c_client *client, u8 command, u8 value)
{
	return i2c_smbus_write_byte_data(client, command, value);
}

static s32 ltr659ps_masked_write_reg(
	struct i2c_client *client, u8 command, u8 value, u8 mask)
{
	u8 data = i2c_smbus_read_byte_data(client, command);

	data = (data & ~mask) | (value & mask);

	return i2c_smbus_write_byte_data(client, command, data);
}

/*
 * Device Attributes Sysfs Show/Store
 */
static ssize_t show_ps_gain(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;
	char *gain_string[] = {"x16", "x64", "x32", "x64"};

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(data->client, LTR659PS_REG_PS_CONTR);
		ret = sprintf(buf, "%s\n", gain_string[(value>>2)&0x3]);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_ps_gain(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid PS gain: 16 : x16; 32 : x32; ");
		PROX_ERROR("64 : x64; others : 16 (default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if (value == 16)
			value = 0x00;
		else if (value == 32)
			value = 0x02;
		else if (value == 64)
			value = 0x01;
		else
			value = 0x00;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_PS_CONTR, value, 0x0c);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_led_pulse_mod_freq(
	struct device *dev
	, struct device_attribute *devattr
	, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;
	char *freq_string[] = {"30kHz", "40kHz", "50kHz", "60kHz"
		, "70kHz", "80kHz", "90kHz", "100kHz"};

	PROX_DEBUG("\n");
	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(data->client, LTR659PS_REG_PS_LED);
		ret = sprintf(buf, "%s\n", freq_string[(value>>5)&0x7]);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_led_pulse_mod_freq(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid LED pulse mod freq: 30 -- 100 : ");
		PROX_ERROR("30kHz -- 100kHz, others : 60kHz (default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if ((value >= 30) && (value <= 100))
			value = (value - 30) / 10;
		else
			value = (60 - 30) / 10;
		value <<= 5;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_PS_LED, value, 0xe0);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_led_current_duty(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;
	char *duty_string[] = {"25%", "50%", "75%", "100%"};

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(data->client, LTR659PS_REG_PS_LED);
		ret = sprintf(buf, "%s\n", duty_string[(value>>3)&0x3]);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_led_current_duty(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid LED current duty: 25 : 25%%; 50 : 50%%;");
		PROX_ERROR(" 75 : 75%%; 100 : 100%%; others : 100%%(def)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if ((value >= 25) && (value <= 100))
			value = (value / 25) - 1;
		else
			value = (100 / 25) - 1;
		value <<= 3;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_PS_LED, value, 0x18);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_led_current(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;
	char *current_string[] = {"5mA", "10mA", "20mA", "50mA"
				, "100mA", "100mA", "100mA", "100mA"};

	PROX_DEBUG("\n");
	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(data->client, LTR659PS_REG_PS_LED);
		ret = sprintf(buf, "%s\n", current_string[value&0x7]);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_led_current(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid LED current 5 : 5mA; 10 : 10mA;");
		PROX_ERROR(" 20 : 20mA; 50 : 50mA; 100 : 100mA; ");
		PROX_ERROR("others : 100mA(default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if (value == 5)
			value = 0x00;
		else if (value == 10)
			value = 0x01;
		else if (value == 20)
			value = 0x02;
		else if (value == 50)
			value = 0x03;
		else
			value = 0x04;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_PS_LED, value, 0x07);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_ps_number_of_led_pulse(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(
			data->client, LTR659PS_REG_PS_N_PULSES);
		ret = sprintf(buf, "%d\n", value & 0x0f);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_ps_number_of_led_pulse(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid PS number of LED pulse 1--15,");
		PROX_ERROR(" others : 1 (default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if ((value > 0) && (value <= 15))
			value = value;
		else
			value = 0x01;
		ltr659ps_write_reg(
			data->client, LTR659PS_REG_PS_N_PULSES, value);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_ps_measurement_rate(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;
	char *meas_rate_string[] = {"50ms", "70ms", "100ms", "200ms"
				, "500ms", "1000ms", "2000ms", "2000ms"};

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(
			data->client, LTR659PS_REG_PS_MEAS_RATE);
		value &= 0x0f;
		if (value < 8)
			ret = sprintf(buf, "%s\n", meas_rate_string[value]);
		else
			ret = sprintf(buf, "10ms\n");
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_ps_measurement_rate(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid PS measurement rate 50 : 50ms; ");
		PROX_ERROR("70 : 70ms; 100 : 100ms; 200 : 200ms; ");
		PROX_ERROR("500 : 500ms; 1000 : 1000ms; 2000 : 2000ms; ");
		PROX_ERROR("10 : 10ms; others : 100ms(default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if (value == 50)
			value = 0x00;
		else if (value == 70)
			value = 0x01;
		else if (value == 200)
			value = 0x03;
		else if (value == 500)
			value = 0x04;
		else if (value == 1000)
			value = 0x05;
		else if (value == 2000)
			value = 0x06;
		else if (value == 10)
			value = 0x08;
		else
			value = 0x02;
		ltr659ps_write_reg(
			data->client, LTR659PS_REG_PS_MEAS_RATE, value);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_interrupt_polarity(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(data->client, LTR659PS_REG_INTERRUPT);
		ret = sprintf(buf, "%s\n", (((value & 0x04) == 0)
			 ? ("INT pin low active") : ("INT pin high active")));
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_interrupt_polarity(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid interrtupt polarity 0 : 0 active;");
		PROX_ERROR(" 1 : 1 active; others : 0 active(default\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if (value == 1)
			value = 1 << 2;
		else
			value = 0;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_INTERRUPT, value, (1<<2));
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_interrupt_mode(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(
			data->client, LTR659PS_REG_INTERRUPT);
		ret = sprintf(buf, "%s\n", ((value & 0x01)
			 ? ("enabled") : ("disabled")));
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_interrupt_mode(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid interrupt mode 0 : disabled; ");
		PROX_ERROR("1 : enabled; others : disabled(default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		if (value != 1)
			value = 0;
		ltr659ps_masked_write_reg(
			data->client, LTR659PS_REG_INTERRUPT, value, 0x01);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_ps_threashold(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int hvalue, lvalue;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		hvalue = (ltr659ps_read_reg(data->client
				, LTR659PS_REG_PS_THRES_UP_1) << 8) |
			ltr659ps_read_reg(data->client
				, LTR659PS_REG_PS_THRES_UP_0);
		lvalue = (ltr659ps_read_reg(data->client
				, LTR659PS_REG_PS_THRES_LOW_1) << 8) |
			ltr659ps_read_reg(data->client
				, LTR659PS_REG_PS_THRES_LOW_0);
		ret = sprintf(buf, "high threshold 0x%x, low threshold 0x%x\n"
			, hvalue, lvalue);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_ps_threashold(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	/* TODO check the format */
	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid PS threshold is 32-bit value, in higher 16");
		PROX_ERROR("bits, 11-bit for upper and lower 16 bits,");
		PROX_ERROR(" 11-bit for lower.\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		int hvalue = (value >> 16) & 0x3ff;
		int lvalue = value & 0x3ff;
		ltr659ps_write_reg(data->client
			, LTR659PS_REG_PS_THRES_UP_0, hvalue & 0xff);
		ltr659ps_write_reg(data->client
			, LTR659PS_REG_PS_THRES_UP_1, hvalue >> 8);
		ltr659ps_write_reg(data->client
			, LTR659PS_REG_PS_THRES_LOW_0, lvalue & 0xff);
		ltr659ps_write_reg(data->client
			, LTR659PS_REG_PS_THRES_LOW_1, lvalue >> 8);
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_ps_offset(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = (ltr659ps_read_reg(
				data->client, LTR659PS_REG_PS_OFFSET_1) << 8) |
			ltr659ps_read_reg(
				data->client, LTR659PS_REG_PS_OFFSET_0);
		ret = sprintf(buf, "0x%x\n", value);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_ps_offset(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid PS offset is a 10-bit value.\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		ltr659ps_write_reg(
			data->client
			, LTR659PS_REG_PS_OFFSET_1
			, ((value >> 8) & 0x3));
		ltr659ps_write_reg(
			data->client
			, LTR659PS_REG_PS_OFFSET_1
			, (value & 0xff));
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_interrupt_persist(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	PROX_DEBUG("\n");

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value = ltr659ps_read_reg(
			data->client, LTR659PS_REG_INTERRUPT_PERSIST);
		ret = sprintf(buf, "0x%x\n", value);
	} else {
		ret = sprintf(buf, "-1\n");
	}
	mutex_unlock(&data->prox_mtx);

	return ret;
}

static ssize_t store_interrupt_persist(
	struct device *dev
	, struct device_attribute *attr
	, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	long value;

	PROX_DEBUG("\n");

	if (kstrtol(buf, 16, &value)) {
		PROX_ERROR("valid interrupt persist 0 -- 15 : every 1 -- 16");
		PROX_ERROR(" values out of threshold.(0 is default)\n");
		return -EINVAL;
	}

	mutex_lock(&data->prox_mtx);
	if (data->enable) {
		value &= 0xf;
		ltr659ps_write_reg(
			data->client
			, LTR659PS_REG_INTERRUPT_PERSIST
			, (value << 4));
	}
	mutex_unlock(&data->prox_mtx);

	return strnlen(buf, count);
}

static ssize_t show_dump_reg(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int i;
	int value;
	int ret = 0;

	mutex_lock(&data->prox_mtx);
	for (i = 0x80; i < 0x9f; i++) {
		value = ltr659ps_read_reg(data->client, i);
		PROX_DEBUG("reg : 0x%02x  value : 0x%02x\n", i, value);
	}
	mutex_unlock(&data->prox_mtx);

	ret = sprintf(buf, "Done\n");

	return ret;
}

static ssize_t show_dump_gpio(
	struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ltr659ps_data *data = iio_priv(indio_dev);
	int value;
	int ret = 0;

	mutex_lock(&data->prox_mtx);
	value = gpio_get_value(data->gpio);
	mutex_unlock(&data->prox_mtx);

	PROX_DEBUG("GPIO (%d) level %s\n"
		, data->gpio, ((value) ? ("high") : ("low")));

	ret = sprintf(buf, "Done\n");

	return ret;
}

IIO_DEVICE_ATTR(ps_gain, 0644, show_ps_gain, store_ps_gain, 0);
IIO_DEVICE_ATTR(led_pulse_mod_freq, 0644
	, show_led_pulse_mod_freq, store_led_pulse_mod_freq, 0);
IIO_DEVICE_ATTR(led_current_duty, 0644
	, show_led_current_duty, store_led_current_duty, 0);
IIO_DEVICE_ATTR(led_current, 0644, show_led_current, store_led_current, 0);
IIO_DEVICE_ATTR(ps_number_of_led_pulse, 0644
	, show_ps_number_of_led_pulse, store_ps_number_of_led_pulse, 0);
IIO_DEVICE_ATTR(ps_measurement_rate, 0644
	, show_ps_measurement_rate, store_ps_measurement_rate, 0);
IIO_DEVICE_ATTR(interrupt_polarity, 0644
	, show_interrupt_polarity, store_interrupt_polarity, 0);
IIO_DEVICE_ATTR(interrupt_mode, 0644
	, show_interrupt_mode, store_interrupt_mode, 0);
IIO_DEVICE_ATTR(ps_threashold, 0644
	, show_ps_threashold, store_ps_threashold, 0);
IIO_DEVICE_ATTR(ps_offset, 0644, show_ps_offset, store_ps_offset, 0);
IIO_DEVICE_ATTR(interrupt_persist, 0644
	, show_interrupt_persist, store_interrupt_persist, 0);
IIO_DEVICE_ATTR(dump_reg, 0444, show_dump_reg, NULL, 0);
IIO_DEVICE_ATTR(dump_gpio, 0444, show_dump_gpio, NULL, 0);

static struct attribute *ltr659ps_attr[] = {
	&iio_dev_attr_ps_gain.dev_attr.attr,
	&iio_dev_attr_led_pulse_mod_freq.dev_attr.attr,
	&iio_dev_attr_led_current_duty.dev_attr.attr,
	&iio_dev_attr_led_current.dev_attr.attr,
	&iio_dev_attr_ps_number_of_led_pulse.dev_attr.attr,
	&iio_dev_attr_ps_measurement_rate.dev_attr.attr,
	&iio_dev_attr_interrupt_polarity.dev_attr.attr,
	&iio_dev_attr_interrupt_mode.dev_attr.attr,
	&iio_dev_attr_ps_threashold.dev_attr.attr,
	&iio_dev_attr_ps_offset.dev_attr.attr,
	&iio_dev_attr_interrupt_persist.dev_attr.attr,
	&iio_dev_attr_dump_reg.dev_attr.attr,
	&iio_dev_attr_dump_gpio.dev_attr.attr,
	NULL
};

static const struct attribute_group ltr659ps_group = {
	.attrs = ltr659ps_attr,
};

static void ltr659ps_report_input_event(struct ltr659ps_data *prox)
{
	/* TODO */

	return;
}


static void ltr659ps_work_function(struct work_struct *work)
{
	struct ltr659ps_data *data = container_of(
		work, struct ltr659ps_data, work.work);
	struct i2c_client *client = data->client;
	u8 status;

	status = i2c_smbus_read_byte_data(client, LTR659PS_REG_PS_STATUS);
	if ((status & 0x03) == 0x03)
		ltr659ps_report_input_event(data);
	enable_irq(data->irq);
	return;
}

static int ltr659ps_check_id(struct i2c_client *client)
{
	u8 part_id, manufac_id;

	part_id = i2c_smbus_read_byte_data(client, LTR659PS_REG_PART_ID);
	manufac_id = i2c_smbus_read_byte_data(client, LTR659PS_REG_MANUFAC_ID);

	if ((part_id != LTR659PS_PART_ID)
		 || (manufac_id != LTR659PS_MANUFAC_ID)) {
		PROX_ERROR("part ID (0x%x) or manufac ID (0x%x) mismatch!\n"
			, part_id, manufac_id);
		return -EINVAL;
	}

	return 0;
}

static int ltr659ps_setup(struct ltr659ps_data *prox)
{
	int ret;

	prox->ps_input_dev = input_allocate_device();
	if (!prox->ps_input_dev) {
		PROX_ERROR("fail to input_allocate_device\n");
		return -ENOMEM;
	}
	prox->ps_input_dev->name = "ltr659_ps";
	set_bit(EV_ABS, prox->ps_input_dev->evbit);
	input_set_abs_params(prox->ps_input_dev,
		ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	ret = input_register_device(prox->ps_input_dev);
	if (ret < 0) {
		PROX_ERROR("fail to input_register_device\n");
		input_free_device(prox->ps_input_dev);
		return ret;
	}

	return 0;
}

static int ltr659ps_init_sensor(struct i2c_client *client)
{
	int ret = 0;
	int i;
	u8 reg, val;
	u8 reg_init[] = {
		/* SW_RESET Register reset = 1 */
		LTR659PS_REG_SW_RESET, 0x02,
		/* PS_CONTR register Gain = 16 */
		LTR659PS_REG_PS_CONTR, 0x03,
		/* PS_LED Pulse Freq = 30kHz duty cyc 100% peak curr 100mA */
		LTR659PS_REG_PS_LED,   0x1f,
		/* PS_MEAS_RATE Meas rate = 50ms */
		LTR659PS_REG_PS_MEAS_RATE, 0x00,
		/* Interrupt Register Interrupt is Active Low, PS trigger */
		LTR659PS_REG_INTERRUPT, 0x01,
		/* PS Upper Threshold Low Byte Register 1000 */
		LTR659PS_REG_PS_THRES_UP_0, 1000 & 0xff,
		/* PS Upper Threshold High Byte Register 1000 */
		LTR659PS_REG_PS_THRES_UP_1, 1000 >> 8,
		/* PS Lower Threshold Low Byte Register 200 */
		LTR659PS_REG_PS_THRES_LOW_0, 200 & 0xff,
		/* PS Lower Threshold High Byte Register 200 */
		LTR659PS_REG_PS_THRES_LOW_1, 200 >> 8,
		/* SW_RESET Register reset = 0 */
		LTR659PS_REG_SW_RESET, 0x00,
		0xff, 0xff,
	};

	i = 0;
	while (reg_init[i] != 0xff) {
		reg = reg_init[i++];
		val = reg_init[i++];
		ltr659ps_write_reg(client, reg, val);
	}

	return ret;
}

static irqreturn_t ltr659ps_interrupt_handler(int irq, void *dev)
{
	struct ltr659ps_data *data = i2c_get_clientdata(dev);

	disable_irq_nosync(data->irq);

	queue_delayed_work(data->prox_wq, &data->work, 0);

	return IRQ_HANDLED;
}

static void ltr659ps_enable_sensor(struct i2c_client *client, int enable)
{
	struct ltr659ps_data *data = i2c_get_clientdata(client);

	if (data->enable != enable) {
		if (enable) {
			queue_delayed_work(data->prox_wq, &data->work
				, msecs_to_jiffies(200));
			/* TODO enable irq */
			/* enable_irq(client->irq); */
			ltr659ps_masked_write_reg(client
				, LTR659PS_REG_PS_CONTR, 0x10, 0x03);
		} else {
			/* TODO disable irq */
			/* disable_irq(client->irq); */
			ltr659ps_masked_write_reg(client
				, LTR659PS_REG_PS_CONTR, 0x00, 0x03);
			cancel_delayed_work_sync(&data->work);
			flush_workqueue(data->prox_wq);
		}
	}

	data->enable = enable;

	return;
}

static int ltr659ps_config_irq(struct i2c_client *client)
{
	struct ltr659ps_data *data = i2c_get_clientdata(client);
	int rc = 0;
	const char *label = "ltr659ps_irq";

	data->irq = client->irq;
	/* TODO disable interrupt for debug */
	rc = request_irq(client->irq,
		ltr659ps_interrupt_handler,
		IRQF_TRIGGER_LOW | IRQF_DISABLED,
		label,
		client);
	if (rc)
		PROX_ERROR("request_irq fail irq = %d\n", client->irq);

	return rc;
}

static const struct iio_info ltr659ps_info = {
	.attrs = &ltr659ps_group,
	.driver_module = THIS_MODULE,
};

static struct i2c_driver ltr659ps_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name   = DEVICE_NAME,
		.owner  = THIS_MODULE,
	},
	.probe = ltr659ps_probe,
	.remove = __devexit_p(ltr659ps_remove),
	.resume = ltr659ps_resume,
	.suspend = ltr659ps_suspend,
	.id_table = ltr659ps_id,
};

static int __devinit ltr659ps_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	int rc = 0;
	struct ltr659ps_data *prox_data;
	struct iio_dev *indio_dev;
	struct ltr659_platform_data *pdata = client->dev.platform_data;

	PROX_DEBUG("\n");

	/* data memory allocation */
	indio_dev = iio_allocate_device(sizeof(*prox_data));
	if (indio_dev == NULL) {
		PROX_ERROR("iio_allocate_device failed!\n");
		return -ENOMEM;
	}
	prox_data = iio_priv(indio_dev);

	prox_data->prox_wq = create_singlethread_workqueue("ltr659ps_wq");
	if (!prox_data->prox_wq) {
		PROX_ERROR("create_singlethread_workqueue failed!\n");
		rc = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	mutex_init(&prox_data->prox_mtx);

	INIT_DELAYED_WORK(&prox_data->work, ltr659ps_work_function);

	i2c_set_clientdata(client, indio_dev);
	prox_data->client = client;
	prox_data->client->flags = 0;
	strlcpy(prox_data->client->name, DEVICE_NAME, I2C_NAME_SIZE);
	prox_data->enable = 0;
	prox_data->gpio = pdata->gpio;

	rc = gpio_request(prox_data->gpio, DEVICE_NAME);
	if (rc < 0) {
		PROX_ERROR("gpio_request failed!\n");
		goto err_gpio_request;
	}
	rc = gpio_direction_input(prox_data->gpio);
	if (rc < 0) {
		PROX_ERROR("gpio_direction_input failed!\n");
		goto err_gpio_direction_input;
	}

	rc = ltr659ps_check_id(prox_data->client);
	if (rc) {
		PROX_ERROR("ltr659ps_check_id failed!\n");
		goto err_check_id_failed;
	}
	PROX_DEBUG("Device LTR659ps found.\n");

	rc = ltr659ps_setup(prox_data);
	if (rc) {
		PROX_ERROR("ltr659ps_setup failed!\n");
		goto err_setup_failed;
	}

	rc = ltr659ps_init_sensor(prox_data->client);
	if (rc) {
		PROX_ERROR("Sensor initialization failed!\n");
		goto err_init_sensor_failed;
	}

	rc = ltr659ps_config_irq(prox_data->client);
	if (rc) {
		PROX_ERROR("Sensor INT configuration failed!\n");
		goto err_config_irq_failed;
	}

	prox_data->enable = 1;
	/* TODO check the threshold, the following values come from datasheet */
	prox_data->default_ps_lowthresh = pdata->default_ps_lowthreshold;
	prox_data->default_ps_highthresh = pdata->default_ps_highthreshold;

	indio_dev->info = &ltr659ps_info;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	rc = iio_device_register(indio_dev);
	if (rc) {
		PROX_ERROR("iio_device_register failed!\n");
		goto err_iio_device_register;
	}

	queue_delayed_work(prox_data->prox_wq
		, &prox_data->work, msecs_to_jiffies(200));

	return 0;

err_iio_device_register:
	free_irq(client->irq, client);
err_config_irq_failed:
err_init_sensor_failed:
	input_unregister_device(prox_data->ps_input_dev);
	input_free_device(prox_data->ps_input_dev);
err_setup_failed:
err_check_id_failed:
err_gpio_direction_input:
	gpio_free(pdata->gpio);
err_gpio_request:
	destroy_workqueue(prox_data->prox_wq);
err_create_singlethread_workqueue_failed:
	iio_free_device(indio_dev);
	return rc;
}


static int __devexit ltr659ps_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ltr659ps_data *data = iio_priv(indio_dev);

	PROX_DEBUG("\n");
	iio_device_unregister(indio_dev);
	free_irq(client->irq, client);
	input_unregister_device(data->ps_input_dev);
	input_free_device(data->ps_input_dev);
	gpio_free(data->gpio);
	cancel_delayed_work(&data->work);
	destroy_workqueue(data->prox_wq);
	iio_free_device(indio_dev);
	return 0;
}

static int ltr659ps_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ltr659ps_data *data = iio_priv(indio_dev);

	PROX_DEBUG("+\n");
	mutex_lock(&data->prox_mtx);
	ltr659ps_enable_sensor(client, 0);
	mutex_unlock(&data->prox_mtx);
	PROX_DEBUG("-\n");
	return 0;
}

static int ltr659ps_resume(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ltr659ps_data *data = iio_priv(indio_dev);

	PROX_DEBUG("+\n");
	mutex_lock(&data->prox_mtx);
	ltr659ps_enable_sensor(client, 1);
	mutex_unlock(&data->prox_mtx);
	PROX_DEBUG("-\n");
	return 0;
}



static int __init ltr659ps_init(void)
{
	int rc;

	PROX_DEBUG("\n");

	rc = i2c_add_driver(&ltr659ps_driver);
	if (rc) {
		PROX_ERROR("i2c_add_driver failed!\n");
		return rc;
	}

	PROX_INFO("Driver intialization done\n");

	return 0;
}

static void __exit ltr659ps_exit(void)
{
	PROX_DEBUG("\n");

	i2c_del_driver(&ltr659ps_driver);

	return;
}

module_init(ltr659ps_init);
module_exit(ltr659ps_exit);

MODULE_DESCRIPTION("LITEON Proximity Sensor LTR-659PS-01 Driver");
MODULE_LICENSE("GPL");

