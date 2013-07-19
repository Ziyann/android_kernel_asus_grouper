/*
 * linux/drivers/video/backlight/of_pwm_bl.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>

#include <mach/latency_allowance.h>
#include <mach/iomap.h>

#include <linux/pwm_backlight.h>

#ifdef CONFIG_OF
/*#define OF_PWMBL_DEBUG  1*/

#undef OF_PWMBL_LOG
#ifdef OF_PWMBL_DEBUG
#define OF_PWMBL_LOG(fmt, args...) pr_info("OF_PWMBL_LOG: " fmt, ## args)
#else
#define OF_PWMBL_LOG(fmt, args...)
#endif

struct platform_pwm_backlight_data
	*of_pwm_bl_parse_platform_data(struct platform_device *ndev)
{
	struct platform_pwm_backlight_data *pdata;
	struct platform_pwm_backlight_data *temp_pdata;
	u32 temp;
	bool is_charged = false;
	unsigned int temp_dft_bri = 0;
	unsigned int temp_dft_charge_bri = 0;
	int pwm_gpio;
	enum of_gpio_flags flags;
	struct property *prop;
	const __be32 *p;
	u32 u;
	int edp_stat_count = 0;
	unsigned int *edp_states;
	unsigned int *edp_brightness;
	struct device_node *np = ndev->dev.of_node;

	temp_pdata = (struct platform_pwm_backlight_data *)
			ndev->dev.platform_data;
	if (temp_pdata && temp_pdata->is_charged) {
		OF_PWMBL_LOG("charging mode\n");
		is_charged = true;
	} else
		OF_PWMBL_LOG("NOT charging mode\n");

	pdata = devm_kzalloc(&ndev->dev,
		sizeof(struct platform_pwm_backlight_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&ndev->dev, "not enough memory\n");
		goto fail_parse;
	}
	if (!of_property_read_u32(np, "nvidia,pwm-id", &temp)) {
		pdata->pwm_id = (int)temp;
		OF_PWMBL_LOG("pwm_id %d\n", pdata->pwm_id);
	}
	if (!of_property_read_u32(np, "nvidia,max-brightness", &temp)) {
		pdata->max_brightness = (unsigned int)temp;
		OF_PWMBL_LOG("max_brightness %d\n", pdata->max_brightness);
	}
	if (!of_property_read_u32(np, "nvidia,dft-brightness", &temp))
		temp_dft_bri = (unsigned int)temp;

	if (!of_property_read_u32(np, "nvidia,dft-charge-brightness", &temp))
		temp_dft_charge_bri = (unsigned int)temp;
	if (is_charged)
		pdata->dft_brightness = temp_dft_charge_bri;
	else
		pdata->dft_brightness = temp_dft_bri;
	OF_PWMBL_LOG("dft_brightness %d\n", pdata->dft_brightness);

	if (!of_property_read_u32(np, "nvidia,lth-brightness", &temp)) {
		pdata->lth_brightness = (unsigned int)temp;
		OF_PWMBL_LOG("lth_brightness %d\n", pdata->lth_brightness);
	}
	if (!of_property_read_u32(np, "nvidia,pwm-period-ns", &temp)) {
		pdata->pwm_period_ns = (unsigned int)temp;
		OF_PWMBL_LOG("pwm_period_ns %d\n", pdata->pwm_period_ns);
	}
	pwm_gpio = of_get_named_gpio_flags(np, "nvidia,pwm-gpio", 0, &flags);
	if (pwm_gpio != 0) {
		pdata->pwm_gpio = pwm_gpio;
		OF_PWMBL_LOG("pwm gpio %d\n", pdata->pwm_gpio);
	}

	edp_states = &(pdata->edp_states[0]);
	edp_brightness = &(pdata->edp_brightness[0]);

	of_property_for_each_u32(np, "nvidia,edp-states", prop, p, u)
		edp_stat_count++;
	if (edp_stat_count > TEGRA_PWM_BL_EDP_NUM_STATES) {
		pr_err("edp states overflow\n");
		goto fail_parse;
	} else {
		of_property_for_each_u32(np, "nvidia,edp-states",
			prop, p, u) {
			OF_PWMBL_LOG("edp states %d\n", u);
			*(edp_states++) = (unsigned int)u;
		}
	}
	edp_stat_count = 0;
	of_property_for_each_u32(np, "nvidia,edp-brightness", prop, p, u)
		edp_stat_count++;
	if (edp_stat_count > TEGRA_PWM_BL_EDP_NUM_STATES) {
		pr_err("edp brightness overflow\n");
		goto fail_parse;
	} else {
		of_property_for_each_u32(np, "nvidia,edp-brightness",
			prop, p, u) {
			OF_PWMBL_LOG("edp brightness %d\n", u);
			*(edp_brightness++) = (unsigned int)u;
		}
	}
	return pdata;
fail_parse:
	return NULL;
}
#else
struct platform_pwm_backlight_data
	*of_pwm_bl_parse_platform_data(struct platform_device *ndev)
{
	return NULL;
}
#endif

