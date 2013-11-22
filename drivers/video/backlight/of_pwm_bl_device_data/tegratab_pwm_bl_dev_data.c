/*
 * Copyright (c) 2013, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <mach/dc.h>

/*#define OF_PANEL_PWMBL_DEBUG  1*/

#undef OF_PANEL_PWMBL_LOG
#ifdef OF_PANEL_PWMBL_DEBUG
#define OF_PANEL_PWMBL_LOG(fmt, args...)	\
	pr_info("OF_PANEL_PWMBL_LOG: " fmt, ## args)
#else
#define OF_PANEL_PWMBL_LOG(fmt, args...)
#endif

extern atomic_t sd_brightness;

static struct platform_driver tegratab_pwm_bl_devdata_gen_drv;
static struct device_node *gp_node;

/* pwm bl list*/
struct of_tegra_pwm_bl_devdata tegratab_lgd_pwm_bl;

static bool tegratab_valid_bl_out;

static tegra_dc_bl_output dsi_lgd_wxga_7_0_bl_output_measured = {
	0,
};

static int tegratab_pwm_bl_output_parse(void)
{
	struct property *prop;
	const __be32 *p;
	u32 u;
	u8 *parse_bl_output;
	int bl_output_cnt = 0;

	parse_bl_output = &(dsi_lgd_wxga_7_0_bl_output_measured[0]);
	of_property_for_each_u32(gp_node, "nvidia,bl-output", prop, p, u)
		bl_output_cnt++;
	if (bl_output_cnt > sizeof(dsi_lgd_wxga_7_0_bl_output_measured)/
		sizeof(dsi_lgd_wxga_7_0_bl_output_measured[0])) {
		pr_err("bl_output_cnt overflow\n");
		return false;
	} else {
		of_property_for_each_u32(gp_node,
			"nvidia,bl-output", prop, p, u) {
			OF_PANEL_PWMBL_LOG("bl-output %d\n", u);
			*(parse_bl_output++) = (u8)u;
		}
		return true;
	}
}

static int dsi_lgd_wxga_7_0_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	if (!tegratab_valid_bl_out) {
		pr_info("Error: invalid bl_out parsed!\n");
		if (brightness > 255)
			pr_info("Error: Brightness > 255!\n");
		return brightness;
	}

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_lgd_wxga_7_0_bl_output_measured[brightness];

	return brightness;
}

static int dsi_lgd_wxga_7_0_check_fb(struct device *dev, struct fb_info *info)
{
	struct platform_device *disp_device;
	disp_device = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &disp_device->dev;
}

static void tegratab_lgd_pwm_bl_devdata
	(struct of_tegra_pwm_bl_devdata *pwm_bl_dev_data)
{
	tegratab_valid_bl_out = tegratab_pwm_bl_output_parse();
	pwm_bl_dev_data->notify = dsi_lgd_wxga_7_0_bl_notify;
	pwm_bl_dev_data->check_fb = dsi_lgd_wxga_7_0_check_fb;
}

static int tegra_pwm_bl_match(struct device *dev, void *data)
{
	struct device_node *dn = data;

	return (dev->of_node == dn) ? 1 : 0;
}

struct device *tegratab_pwm_bl_devdata_to_pwm_bl(struct device_node *dn)
{
	struct device *dev;
	dev = driver_find_device(&tegratab_pwm_bl_devdata_gen_drv.driver,
		NULL, dn, tegra_pwm_bl_match);
	if (!dev)
		return NULL;
	return dev;
}

static int tegratab_pwm_bl_devdata_gen_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct of_tegra_pwm_bl_devdata *pwm_bl_devdata;

	gp_node = np;

	pwm_bl_devdata = devm_kzalloc(dev, sizeof(*pwm_bl_devdata), GFP_KERNEL);
	if (!pwm_bl_devdata)
		return -ENOMEM;


	if (of_find_compatible_node(NULL, NULL, "lgd-pwm-bl,tegratab")||
		of_find_compatible_node(NULL, NULL, "lgd-pwm-bl,tegranote7c")) {
		tegratab_lgd_pwm_bl_devdata(&tegratab_lgd_pwm_bl);
		memcpy(pwm_bl_devdata, &tegratab_lgd_pwm_bl,
				sizeof(struct of_tegra_pwm_bl_devdata));
	}

	platform_set_drvdata(pdev, pwm_bl_devdata);
	pwm_bl_devdata_set_callback(tegratab_pwm_bl_devdata_to_pwm_bl);
	return 0;
}

static int tegratab_pwm_bl_devdata_gen_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id tegratab_pwm_bl_of_match[] = {
	{ .compatible = "lgd-pwm-bl,tegratab", },
	{ .compatible = "lgd-pwm-bl,tegranote7c", },
	{ },
};

static struct platform_driver tegratab_pwm_bl_devdata_gen_drv = {
	.probe = tegratab_pwm_bl_devdata_gen_probe,
	.remove = tegratab_pwm_bl_devdata_gen_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "of_tegratab_pwm_bl_devdata_generate",
		.of_match_table = tegratab_pwm_bl_of_match,
	},
};

int __init of_tegratab_pwm_bl_devdata_init(void)
{
	return platform_driver_register(&tegratab_pwm_bl_devdata_gen_drv);
}

void __exit of_tegratab_pwm_bl_devdata_exit(void)
{
	platform_driver_unregister(&tegratab_pwm_bl_devdata_gen_drv);
}

subsys_initcall(of_tegratab_pwm_bl_devdata_init);
module_exit(of_tegratab_pwm_bl_devdata_exit);
MODULE_DESCRIPTION("tegratab pwm_bl devdata generate driver with device tree info");
