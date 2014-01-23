/*
 * arch/arm/mach-tegra/board-tegranote7c-sensors.c
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <linux/cm3217.h>
#include <linux/ltr659ps.h>
#include <mach/edp.h>
#include <linux/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#ifndef CONFIG_USE_OF
#include <media/ov5693.h>
#include <media/ad5823.h>
#endif
#include <media/mt9m114.h>
#include <media/ov7695.h>
#include <generated/mach-types.h>
#include <linux/power/sbs-battery.h>
#include <linux/generic_adc_thermal.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-tegranote7c.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

static struct throttle_table tj_throttle_table[] = {
		/* CPU_THROT_LOW cannot be used by other than CPU */
		/* NO_CAP cannot be used by CPU */
		/*    CPU,   C2BUS,   C3BUS,    SCLK,     EMC */
		{ { 1530000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1428000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1224000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  564000,  564000,  NO_CAP,  NO_CAP } },
		{ {  612000,  528000,  528000,  NO_CAP,  NO_CAP } },
		{ {  612000,  492000,  492000,  NO_CAP,  NO_CAP } },
		{ {  612000,  420000,  420000,  NO_CAP,  NO_CAP } },
		{ {  612000,  408000,  408000,  NO_CAP,  NO_CAP } },
		{ {  612000,  360000,  360000,  NO_CAP,  NO_CAP } },
		{ {  612000,  360000,  360000,  312000,  NO_CAP } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  468000,  360000,  360000,  312000,  480000 } },
		{ {  468000,  276000,  276000,  208000,  480000 } },
		{ {  372000,  276000,  276000,  208000,  204000 } },
		{ {  288000,  276000,  276000,  208000,  204000 } },
		{ {  252000,  276000,  228000,  208000,  102000 } },
		{ {  204000,  276000,  228000,  208000,  102000 } },
		{ {  102000,  276000,  228000,  208000,  102000 } },
	  { { CPU_THROT_LOW,  276000,  228000,  208000,  102000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init tegranote7c_throttle_init(void)
{
	if (machine_is_tegranote7c())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(tegranote7c_throttle_init);

static struct nct1008_platform_data tegranote7c_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 120, /* C */
};

static struct i2c_board_info tegranote7c_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &tegranote7c_nct1008_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
}

#ifndef CONFIG_USE_OF
static struct tegra_pingroup_config mclk_disable =
	VI_PINMUX(CAM_MCLK, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);
#endif

static struct tegra_pingroup_config pbb0_disable =
	VI_PINMUX(GPIO_PBB0, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

#ifndef CONFIG_USE_OF
/*
 * As a workaround, tegranote7c_vcmvdd need to be allocated to activate the
 * sensor devices. This is due to the focuser device(AD5823) will hook up
 * the i2c bus if it is not powered up.
*/
static struct regulator *tegranote7c_vcmvdd;

static int tegranote7c_get_vcmvdd(void)
{
	if (!tegranote7c_vcmvdd) {
		tegranote7c_vcmvdd = regulator_get(NULL, "vdd_af_cam1");
		if (unlikely(WARN_ON(IS_ERR(tegranote7c_vcmvdd)))) {
			pr_err("%s: can't get regulator vcmvdd: %ld\n",
				__func__, PTR_ERR(tegranote7c_vcmvdd));
			tegranote7c_vcmvdd = NULL;
			return -ENODEV;
		}
	}
	return 0;
}

static int tegranote7c_ov5693_power_on(struct ov5693_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->dovdd))
		return -EFAULT;

	if (tegranote7c_get_vcmvdd())
		goto ov5693_poweron_fail;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	err = regulator_enable(tegranote7c_vcmvdd);
	if (unlikely(err))
		goto ov5693_vcmvdd_fail;

	tegra_pinmux_config_table(&mclk_enable, 1);
	usleep_range(300, 310);

	return 0;

ov5693_vcmvdd_fail:
	regulator_disable(pw->dovdd);

ov5693_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

ov5693_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int tegranote7c_ov5693_power_off(struct ov5693_power_rail *pw)
{
	if (unlikely(!pw || !tegranote7c_vcmvdd || !pw->avdd || !pw->dovdd))
		return -EFAULT;

	usleep_range(21, 25);
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(tegranote7c_vcmvdd);
	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static struct nvc_gpio_pdata ov5693_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM1_POWER_DWN_GPIO, true, 0, },
};
static struct ov5693_platform_data tegranote7c_ov5693_pdata = {
	.num		= 5693,
	.dev_name	= "camera",
	.gpio_count	= ARRAY_SIZE(ov5693_gpio_pdata),
	.gpio		= ov5693_gpio_pdata,
	.power_on	= tegranote7c_ov5693_power_on,
	.power_off	= tegranote7c_ov5693_power_off,
};

static int tegranote7c_ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);

	gpio_set_value_cansleep(pdata->gpio, 1);

	return err;
}

static int tegranote7c_ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);
	return 0;
}

static struct ad5823_platform_data tegranote7c_ad5823_pdata = {
	.gpio = CAM_AF_PWDN,
	.power_on	= tegranote7c_ad5823_power_on,
	.power_off	= tegranote7c_ad5823_power_off,
};
#endif

static int tegranote7c_mt9m114_power_on(struct mt9m114_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	gpio_set_value(CAM_RSTN, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto mt9m114_iovdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto mt9m114_avdd_fail;

	usleep_range(1000, 1020);
	gpio_set_value(CAM_RSTN, 1);

	usleep_range(1000, 1020);
	tegra_pinmux_config_table(&pbb0_enable, 1);
	usleep_range(200, 220);

	/* return 1 to skip the in-driver power_on swquence */
	return 1;

mt9m114_avdd_fail:
	regulator_disable(pw->iovdd);

mt9m114_iovdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	return -ENODEV;
}

static int tegranote7c_mt9m114_power_off(struct mt9m114_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	usleep_range(100, 120);
	tegra_pinmux_config_table(&pbb0_disable, 1);
	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(100, 120);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	return 1;
}

struct mt9m114_platform_data tegranote7c_mt9m114_pdata = {
	.power_on = tegranote7c_mt9m114_power_on,
	.power_off = tegranote7c_mt9m114_power_off,
};

static int tegranote7c_ov7695_power_on(struct ov7695_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ov7695_iovdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov7695_avdd_fail;
	usleep_range(1000, 1020);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);
	usleep_range(1000, 1020);

	tegra_pinmux_config_table(&pbb0_enable, 1);
	usleep_range(200, 220);

	return 0;

ov7695_avdd_fail:
	regulator_disable(pw->iovdd);

ov7695_iovdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	return -ENODEV;
}

static int tegranote7c_ov7695_power_off(struct ov7695_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;
	usleep_range(100, 120);

	tegra_pinmux_config_table(&pbb0_disable, 1);
	usleep_range(100, 120);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	usleep_range(100, 120);

	regulator_disable(pw->avdd);
	usleep_range(100, 120);

	regulator_disable(pw->iovdd);

	return 0;
}

struct ov7695_platform_data tegranote7c_ov7695_pdata = {
	.power_on = tegranote7c_ov7695_power_on,
	.power_off = tegranote7c_ov7695_power_off,
};

static struct i2c_board_info tegranote7c_i2c_board_info_e1599[] = {
#ifndef CONFIG_USE_OF
	{
		I2C_BOARD_INFO("ov5693", 0x10),
		.platform_data = &tegranote7c_ov5693_pdata,
	},
	{
		I2C_BOARD_INFO("ad5823", 0x0c),
		.platform_data = &tegranote7c_ad5823_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("mt9m114", 0x48),
		.platform_data = &tegranote7c_mt9m114_pdata,
	},
	{
		I2C_BOARD_INFO("ov7695", 0x21),
		.platform_data = &tegranote7c_ov7695_pdata,
	},
};

static int tegranote7c_camera_init(void)
{
#ifndef CONFIG_USE_OF
	tegra_pinmux_config_table(&mclk_disable, 1);
#endif
	tegra_pinmux_config_table(&pbb0_disable, 1);

	i2c_register_board_info(2, tegranote7c_i2c_board_info_e1599,
		ARRAY_SIZE(tegranote7c_i2c_board_info_e1599));
	return 0;
}

/* MPU board file definition	*/
static struct mpu_platform_data mpu6050_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

static struct i2c_board_info __initdata inv_mpu6050_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6050_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu6050_i2c2_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);

	i2c_register_board_info(gyro_bus_num, inv_mpu6050_i2c2_board_info,
		ARRAY_SIZE(inv_mpu6050_i2c2_board_info));
}

static int __maybe_unused tegranote7c_nct1008_init(void)
{
	int nct1008_port;
	int ret = 0;

	nct1008_port = TEGRA_GPIO_PO4;

	tegra_add_cdev_trips(tegranote7c_nct1008_pdata.trips,
				&tegranote7c_nct1008_pdata.num_trips);

	tegranote7c_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: tegranote7c nct1008 irq %d", __func__, \
				tegranote7c_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* tegranote7c has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, tegranote7c_i2c4_nct1008_board_info,
		ARRAY_SIZE(tegranote7c_i2c4_nct1008_board_info));

	return ret;
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 45000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			6, 1, 0, -2,
			-1, -2, -3, -2,
			-2, -2, -3, -2,
			-3, -2, -3, -4,
			-5, -5, -6, -12
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			35, 11, 3, 4,
			-2, 3, 6, -2,
			5, 8, 14, 12,
			16, 17, 15, 9,
			-3, -23, -24, 36
		},
	},
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
	.polling_period = 1100,
	.passive_delay = 15000,
	.toffset = 1480,
	.tc1 = 10,
	.tc2 = 1,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC */
	{ { 1650000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1500000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1400000, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300000, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1200000, 564000, NO_CAP, NO_CAP, 624000 } },
	{ { 1100000, 528000, NO_CAP, NO_CAP, 624000 } },
	{ {  994500, 528000, NO_CAP, NO_CAP, 624000 } },
	{ {  969000, 528000, NO_CAP, NO_CAP, 624000 } },
	{ {  943500, 468000, 324000, NO_CAP, 528000 } },
	{ {  918000, 468000, 324000, NO_CAP, 528000 } },
	{ {  892500, 468000, 252000, 208000, 528000 } },
	{ {  867000, 468000, 252000, 208000, 528000 } },
	{ {  841500, 468000, 252000, 136000, 528000 } },
	{ {  816000, 468000, 252000, 136000, 528000 } },
	{ {  790500, 384000, 204000, 102000, 528000 } },
	{ {  765000, 384000, 204000, 102000, 528000 } },
	{ {  739500, 384000, 204000, 102000, 528000 } },
	{ {  714000, 384000, 204000, 102000, 528000 } },
	{ {  688500, 384000, 204000, 102000, 528000 } },
	{ {  663000, 384000, 204000, 102000, 528000 } },
	{ {  637500, 384000, 204000, 102000, 408000 } },
	{ {  612000, 384000, 204000, 102000, 408000 } },
	{ {  586500, 300000, 204000,  81600, 408000 } },
	{ {  561000, 300000, 204000,  81600, 408000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init tegranote7c_skin_init(void)
{
	if (machine_is_tegranote7c()) {
		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(tegranote7c_skin_init);
#endif

struct ntc_thermistor_adc_table {
	int temp; /* degree C */
	int adc;
};

/* This values are only for tegranote7c platform. */
static struct ntc_thermistor_adc_table thermistor_table[] = {
	{ -40, 2726 }, { -39, 2723 }, { -38, 2720 }, { -37, 2716 },
	{ -36, 2712 }, { -35, 2708 }, { -34, 2703 }, { -33, 2699 },
	{ -32, 2694 }, { -31, 2688 }, { -30, 2683 }, { -29, 2677 },
	{ -28, 2670 }, { -27, 2664 }, { -26, 2657 }, { -25, 2649 },
	{ -24, 2641 }, { -23, 2633 }, { -22, 2624 }, { -21, 2614 },
	{ -20, 2605 }, { -19, 2594 }, { -18, 2583 }, { -17, 2572 },
	{ -16, 2560 }, { -15, 2547 }, { -14, 2534 }, { -13, 2520 },
	{ -12, 2506 }, { -11, 2491 }, { -10, 2475 }, {  -9, 2459 },
	{  -8, 2442 }, {  -7, 2424 }, {  -6, 2405 }, {  -5, 2386 },
	{  -4, 2366 }, {  -3, 2345 }, {  -2, 2324 }, {  -1, 2302 },
	{   0, 2279 }, {   1, 2256 }, {   2, 2231 }, {   3, 2206 },
	{   4, 2181 }, {   5, 2154 }, {   6, 2127 }, {   7, 2099 },
	{   8, 2071 }, {   9, 2042 }, {  10, 2012 }, {  11, 1982 },
	{  12, 1952 }, {  13, 1921 }, {  14, 1889 }, {  15, 1857 },
	{  16, 1824 }, {  17, 1792 }, {  18, 1758 }, {  19, 1725 },
	{  20, 1691 }, {  21, 1658 }, {  22, 1623 }, {  23, 1589 },
	{  24, 1555 }, {  25, 1521 }, {  26, 1487 }, {  27, 1453 },
	{  28, 1419 }, {  29, 1385 }, {  30, 1351 }, {  31, 1318 },
	{  32, 1284 }, {  33, 1252 }, {  34, 1219 }, {  35, 1187 },
	{  36, 1155 }, {  37, 1123 }, {  38, 1092 }, {  39, 1062 },
	{  40, 1032 }, {  41, 1002 }, {  42,  973 }, {  43,  945 },
	{  44,  917 }, {  45,  889 }, {  46,  863 }, {  47,  836 },
	{  48,  811 }, {  49,  786 }, {  50,  761 }, {  51,  737 },
	{  52,  714 }, {  53,  692 }, {  54,  670 }, {  55,  648 },
	{  56,  627 }, {  57,  607 }, {  58,  587 }, {  59,  568 },
	{  60,  549 }, {  61,  531 }, {  62,  514 }, {  63,  497 },
	{  64,  480 }, {  65,  464 }, {  66,  449 }, {  67,  434 },
	{  68,  420 }, {  69,  406 }, {  70,  392 }, {  71,  379 },
	{  72,  366 }, {  73,  354 }, {  74,  342 }, {  75,  330 },
	{  76,  319 }, {  77,  309 }, {  78,  298 }, {  79,  288 },
	{  80,  279 }, {  81,  269 }, {  82,  260 }, {  83,  252 },
	{  84,  243 }, {  85,  235 }, {  86,  227 }, {  87,  220 },
	{  88,  213 }, {  89,  206 }, {  90,  199 }, {  91,  192 },
	{  92,  186 }, {  93,  180 }, {  94,  174 }, {  95,  168 },
	{  96,  163 }, {  97,  158 }, {  98,  152 }, {  99,  147 },
	{ 100,  143 }, { 101,  138 }, { 102,  134 }, { 103,  129 },
	{ 104,  125 }, { 105,  121 }, { 106,  117 }, { 107,  114 },
	{ 108,  110 }, { 109,  107 }, { 110,  103 }, { 111,  100 },
	{ 112,   97 }, { 113,   94 }, { 114,   91 }, { 115,   88 },
	{ 116,   86 }, { 117,   83 }, { 118,   80 }, { 119,   78 },
	{ 120,   76 }, { 121,   73 }, { 122,   71 }, { 123,   69 },
	{ 124,   67 }, { 125,   65 },
};

static int gadc_thermal_thermistor_adc_to_temp(
		struct gadc_thermal_platform_data *pdata, int *val, int *val2)
{
	int table_size = ARRAY_SIZE(thermistor_table);
	int temp = 0, adc_hi, adc_lo;
	int i;

	for (i = 0; i < table_size; i++)
		if (*val >= thermistor_table[i].adc)
			break;

	if (i == 0) {
		temp = thermistor_table[i].temp * 1000;
	} else if (i >= (table_size - 1)) {
		temp = thermistor_table[table_size - 1].temp * 1000;
	} else {
		adc_hi = thermistor_table[i - 1].adc;
		adc_lo = thermistor_table[i].adc;
		temp = thermistor_table[i].temp * 1000;
		temp -= ((*val - adc_lo) * 1000 / (adc_hi - adc_lo));
	}

	return temp;
};

#define TDIODE_PRECISION_MULTIPLIER	1000000000LL
#define TDIODE_MIN_TEMP			-25000LL
#define TDIODE_MAX_TEMP			125000LL

static int gadc_thermal_tdiode_adc_to_temp(
		struct gadc_thermal_platform_data *pdata, int *val, int *val2)
{
	/*
	 * Series resistance cancellation using multi-current ADC measurement.
	 * diode temp = ((adc2 - k * adc1) - (b2 - k * b1)) / (m2 - k * m1)
	 * - adc1 : ADC raw with current source 400uA
	 * - m1, b1 : calculated with current source 400uA
	 * - adc2 : ADC raw with current source 800uA
	 * - m2, b2 : calculated with current source 800uA
	 * - k : 2 (= 800uA / 400uA)
	 */
	const s64 m1 = -0.00571005 * TDIODE_PRECISION_MULTIPLIER;
	const s64 b1 = 2524.29891 * TDIODE_PRECISION_MULTIPLIER;
	const s64 m2 = -0.005519811 * TDIODE_PRECISION_MULTIPLIER;
	const s64 b2 = 2579.354349 * TDIODE_PRECISION_MULTIPLIER;
	s64 temp = TDIODE_PRECISION_MULTIPLIER;

	temp *= (s64)((*val2) - 2 * (*val));
	temp -= (b2 - 2 * b1);
	temp = div64_s64(temp, (m2 - 2 * m1));
	temp = min_t(s64, max_t(s64, temp, TDIODE_MIN_TEMP), TDIODE_MAX_TEMP);

	return temp;
};

static struct gadc_thermal_platform_data gadc_thermal_thermistor_pdata = {
	.iio_channel_name = "thermistor",
	.tz_name = "Tboard",
	.temp_offset = 0,
	.adc_to_temp = gadc_thermal_thermistor_adc_to_temp,
};

static struct gadc_thermal_platform_data gadc_thermal_tdiode_pdata = {
	.iio_channel_name = "tdiode",
	.tz_name = "Tdiode",
	.temp_offset = 0,
	.adc_to_temp = gadc_thermal_tdiode_adc_to_temp,
};

static struct platform_device gadc_thermal_thermistor = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev	= {
		.platform_data = &gadc_thermal_thermistor_pdata,
	},
};

static struct platform_device gadc_thermal_tdiode = {
	.name   = "generic-adc-thermal",
	.id     = 1,
	.dev	= {
		.platform_data = &gadc_thermal_tdiode_pdata,
	},
};

static struct platform_device *gadc_thermal_devices[] = {
	&gadc_thermal_thermistor,
	&gadc_thermal_tdiode,
};

static struct ltr659_platform_data tegranote7c_ltr659ps_pdata = {
	.gpio = TEGRA_GPIO_PX3,
	.default_ps_lowthreshold = 20,
	.default_ps_highthreshold = 30,
};

static struct i2c_board_info tegratab_i2c1_ltr659ps_board_info[] = {
	{
		I2C_BOARD_INFO("ltr659ps", 0x23),
		.platform_data = &tegranote7c_ltr659ps_pdata,
		.irq = -1,
	},
};

int __init tegranote7c_sensors_init(void)
{
	int err;

	err = platform_add_devices(gadc_thermal_devices,
				   ARRAY_SIZE(gadc_thermal_devices));
	if (err) {
		pr_err("%s: gadc_thermal register failed\n", __func__);
		return err;
	}

	tegratab_i2c1_ltr659ps_board_info[0].irq =
		gpio_to_irq(tegranote7c_ltr659ps_pdata.gpio);
	i2c_register_board_info(0, tegratab_i2c1_ltr659ps_board_info,
		ARRAY_SIZE(tegratab_i2c1_ltr659ps_board_info));

	tegranote7c_camera_init();
	mpuirq_init();

	return 0;
}
