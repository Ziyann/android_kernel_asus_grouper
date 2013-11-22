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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/dc.h>
#include <linux/delay.h>

static struct platform_driver tegratab_lcd_devdata_gen_drv;
static struct device_node *gp_node;

/* LCDs list */
struct of_tegra_lcd_devdata tegratab_lgd_lcd;

#define LGD_LCD_REGULATORS_COUNT		3

#define WORKAROUND_TO_REMOVE_LGD7_T4_TIME	1

static bool lgd_lcd_reg_requested;

#if WORKAROUND_TO_REMOVE_LGD7_T4_TIME
extern atomic_t touch_dvdd_on;
#endif

static struct regulator *lgd_lcd_regs[LGD_LCD_REGULATORS_COUNT];

static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *dvdd_lcd_1v8;

static struct tegra_dsi_cmd lgd_wxga_7_0_init_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x01, 0x0),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(0x15, 0xAE, 0x0B),
	DSI_CMD_SHORT(0x15, 0xEE, 0xEA),
	DSI_CMD_SHORT(0x15, 0xEF, 0x5F),
	DSI_CMD_SHORT(0x15, 0xF2, 0x68),
	DSI_CMD_SHORT(0x15, 0xEE, 0x0),
	DSI_CMD_SHORT(0x15, 0xEF, 0x0),
};

static struct tegra_dsi_cmd lgd_wxga_7_0_late_resume_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x10, 0x0),
	DSI_DLY_MS(120),
};

static struct tegra_dsi_cmd lgd_wxga_7_0_early_suspend_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x11, 0x0),
	DSI_DLY_MS(160),
};

static struct tegra_dsi_cmd lgd_wxga_7_0_suspend_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x11, 0x0),
	DSI_DLY_MS(160),
};

static int lgd_lcd_regulators_get(struct device *dev)
{
	int err = 0;
	struct property *prop;
	const char *regulator;
	int reg_i = 0;

	if (lgd_lcd_reg_requested)
		return 0;
	of_property_for_each_string(gp_node, "nvidia,regulators",
			prop, regulator) {
		lgd_lcd_regs[reg_i] =
			regulator_get(dev, regulator);
		if (IS_ERR_OR_NULL(lgd_lcd_regs[reg_i])) {
			err = PTR_ERR(lgd_lcd_regs[reg_i]);
			lgd_lcd_regs[reg_i] = NULL;
			return err;
		} else {
			if (reg_i == 0)
				avdd_lcd_3v3 = lgd_lcd_regs[reg_i];
			else if (reg_i == 1)
				vdd_lcd_bl_en = lgd_lcd_regs[reg_i];
			else if (reg_i == 2)
				dvdd_lcd_1v8 = lgd_lcd_regs[reg_i];
			reg_i++;
		}
	}
	lgd_lcd_reg_requested = true;
	return 0;
}

static int lgd_wxga_7_0_enable(struct device *dev)
{
	int err = 0;

	err = lgd_lcd_regulators_get(dev);
	if (err < 0) {
		pr_err("lgd lcd regulator get failed\n");
		goto fail;
	}
#if WORKAROUND_TO_REMOVE_LGD7_T4_TIME
	/*
	 * LGD WXGA 7" panel spec requests 1s delay between
	 * "panel 3v3 off" and "3v3 on" if all panel
	 * related power rails (1v8, 3v3) are not turned off.

	 * precondition for work around
	 *  - In power off, panel off then touch off.
	 *  - In power on, touch on then panel on.

	 * why work around is necessary?
	 *  - Same 1v8 rail is shared by touch and panel.
	 *  - In panel off, 3v3 rail off and 1v8 rail off
	 *    are requested, but 1v8 rail isn't turned off
	 *    in panel off timeframe because of touch module.
	 *    It is possible to get panel on request without
	 *    touch off/on control. In this case, 1S delay
	 *    is necessary per spec. If 1v8 is turned off
	 *    and on by touch module before panel on request,
	 *    then, we don't need 1S delay. If 1v8 is turned
	 *    off in panel on request time, we don't need
	 *    1S delay, either.

	 * In device power on, we don't need to obey this
	 * precondition, since there's no power rail off.
	 * This can be accomplished by setting touch_dvdd_on
	 * initial state to true.
	 */

	if ((!atomic_read(&touch_dvdd_on)) &&
		regulator_is_enabled(dvdd_lcd_1v8)) {
		msleep(1000);
	}
	/*
	 * Clean touch_dvdd_on
	 */
	atomic_set(&touch_dvdd_on, 0);
#endif

	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("%s: dvdd_lcd_1v8 en failed\n", __func__);
			goto fail;
		}
	}

	msleep(20); /*Turn on 1.8V, then, AVDD after 20ms */

	if (avdd_lcd_3v3) {
		err = regulator_enable(avdd_lcd_3v3);
		if (err < 0) {
			pr_err("%s: avdd_lcd_3v3 en failed\n", __func__);
			goto fail;
		}
	}

	/* VDD to MIPI > 100ms based on the spec.
	 * Driver already take 50ms, so having 50ms delay here.
	 */
	msleep(50);

	return 0;
fail:
	return err;
}

static int lgd_wxga_7_0_postpoweron(struct device *dev)
{
	int err = 0;

	if (vdd_lcd_bl_en) {
		/*If rail is already enabled, we don't need delay*/
		/*
		 * MIPI to VLED > 200ms, based on the spec
		 * init_cmd cares 20ms delay already.
		*/
		if (!regulator_is_enabled(vdd_lcd_bl_en))
			msleep(180);

		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("%s: vdd_lcd_bl_en en failed\n", __func__);
			goto fail;
		}
	}
	return 0;
fail:
	return err;
}

static int lgd_wxga_7_0_disable(void)
{
	msleep(50); /*MIPI off to VDD off needs to be 50~150ms per spec*/

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);
	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);

#if WORKAROUND_TO_REMOVE_LGD7_T4_TIME
	/*
	 * Clean touch_dvdd_on here.
	 * pre condition for the work around mentions
	 * the sequence in power off is "panel off => touch off".
	 */
	atomic_set(&touch_dvdd_on, 0);
#else
	msleep(1000); /*LCD panel VDD on needs to be 1000>ms after it's off*/
#endif
	return 0;
}

static int lgd_wxga_7_0_prepoweroff(void)
{
	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	/*
	 * VLED off to MIPI off > 200ms per spec
	 * suspend_cmd cares 160ms delay already.
	 */
	msleep(40);
	return 0;
}

static int lgd_wxga_7_0_postsuspend(void)
{
	return 0;
}

static void tegratab_lgd_lcd_devdata(struct of_tegra_lcd_devdata *lcd_dev_data)
{
	lcd_dev_data->enable = lgd_wxga_7_0_enable;
	lcd_dev_data->disable = lgd_wxga_7_0_disable;
	lcd_dev_data->postpoweron = lgd_wxga_7_0_postpoweron;
	lcd_dev_data->postsuspend = lgd_wxga_7_0_postsuspend;
	lcd_dev_data->prepoweroff = lgd_wxga_7_0_prepoweroff;
	lcd_dev_data->dsi_init_cmd =
		lgd_wxga_7_0_init_cmd;
	lcd_dev_data->n_init_cmd =
		ARRAY_SIZE(lgd_wxga_7_0_init_cmd);
	lcd_dev_data->dsi_early_suspend_cmd =
		lgd_wxga_7_0_early_suspend_cmd;
	lcd_dev_data->n_early_suspend_cmd =
		ARRAY_SIZE(lgd_wxga_7_0_early_suspend_cmd);
	lcd_dev_data->dsi_late_resume_cmd =
		lgd_wxga_7_0_late_resume_cmd;
	lcd_dev_data->n_late_resume_cmd =
		ARRAY_SIZE(lgd_wxga_7_0_late_resume_cmd);
	lcd_dev_data->dsi_suspend_cmd =
		lgd_wxga_7_0_suspend_cmd;
	lcd_dev_data->n_suspend_cmd =
		ARRAY_SIZE(lgd_wxga_7_0_suspend_cmd);
}

static int tegra_lcd_match_in_dc(struct device *dev, void *data)
{
	struct device_node *dn = data;

	return (dev->of_node == dn) ? 1 : 0;
}

struct device *tegratab_lcd_devdata_to_dc(struct device_node *dn)
{
	struct device *dev;
	dev = driver_find_device(&tegratab_lcd_devdata_gen_drv.driver,
		NULL, dn, tegra_lcd_match_in_dc);
	if (!dev)
		return NULL;
	return dev;
}

static int tegratab_lcd_devdata_gen_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct of_tegra_lcd_devdata *lcd_devdata;

	gp_node = np;

	lcd_devdata = devm_kzalloc(dev, sizeof(*lcd_devdata), GFP_KERNEL);
	if (!lcd_devdata)
		return -ENOMEM;


	if (of_find_compatible_node(NULL, NULL, "lgd,tegratab") ||
		of_find_compatible_node(NULL, NULL, "lgd,tegranote7c")) {
		tegratab_lgd_lcd_devdata(&tegratab_lgd_lcd);
		memcpy(lcd_devdata, &tegratab_lgd_lcd,
				sizeof(struct of_tegra_lcd_devdata));
	}

	platform_set_drvdata(pdev, lcd_devdata);
	lcd_devdata_to_dc_set_callback(tegratab_lcd_devdata_to_dc);
	return 0;
}

static int tegratab_lcd_devdata_gen_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id tegratab_lcd_of_match[] = {
	{ .compatible = "lgd,tegratab", },
	{ .compatible = "lgd,tegranote7c", },
	{ },
};

static struct platform_driver tegratab_lcd_devdata_gen_drv = {
	.probe = tegratab_lcd_devdata_gen_probe,
	.remove = tegratab_lcd_devdata_gen_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "of_tegratab_lcd_devdata_generate",
		.of_match_table = tegratab_lcd_of_match,
	},
};

int __init of_tegratab_lcd_devdata_init(void)
{
	return platform_driver_register(&tegratab_lcd_devdata_gen_drv);
}

void __exit of_tegratab_lcd_devdata_exit(void)
{
	platform_driver_unregister(&tegratab_lcd_devdata_gen_drv);
}

subsys_initcall(of_tegratab_lcd_devdata_init);
module_exit(of_tegratab_lcd_devdata_exit);
MODULE_DESCRIPTION("tegratab lcd devdata generate driver with device tree info");
