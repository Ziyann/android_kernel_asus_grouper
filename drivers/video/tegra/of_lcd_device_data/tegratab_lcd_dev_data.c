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

static bool lgd_lcd_reg_requested;
static struct regulator *lgd_lcd_regs[LGD_LCD_REGULATORS_COUNT];

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
		} else
			reg_i++;
	}
	lgd_lcd_reg_requested = true;
	return 0;
}

static int lgd_wxga_7_0_enable(struct device *dev)
{
	int err = 0;
	int reg_i;

	err = lgd_lcd_regulators_get(dev);
	if (err < 0) {
		pr_err("lgd lcd regulator get failed\n");
		goto fail;
	}

	msleep(20);
	for (reg_i = 0; reg_i < LGD_LCD_REGULATORS_COUNT; reg_i++) {
		if (lgd_lcd_regs[reg_i]) {
			err = regulator_enable(lgd_lcd_regs[reg_i]);
			if (err < 0) {
				pr_err("%s: reg en failed\n", __func__);
				goto fail;
			}
		}
		if (reg_i == 0)
			msleep(150);
		else
			msleep(100);
	}
	return 0;
fail:
	return err;
}

static int lgd_wxga_7_0_disable(void)
{
	int reg_i;

	for (reg_i = 0; reg_i < LGD_LCD_REGULATORS_COUNT; reg_i++)
		regulator_disable(
			lgd_lcd_regs[LGD_LCD_REGULATORS_COUNT-reg_i-1]);

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
	lcd_dev_data->postsuspend = lgd_wxga_7_0_postsuspend;
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


	if (of_find_compatible_node(NULL, NULL, "lgd,tegratab")) {
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
