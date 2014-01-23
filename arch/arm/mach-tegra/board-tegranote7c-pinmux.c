/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>
#include "board.h"
#include "board-tegranote7c.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra-board-id.h"

#include <mach/pinmux-t11.h>

static __initdata struct tegra_drive_pingroup_config tegranote7c_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 36, 20, SLOW, SLOW),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 22, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 2, 2, FASTEST,
								FASTEST, 1),
};

#include "board-tegranote7c-pinmux-t11x.h"

/* THIS IS FOR TESTING OR WORKAROUND PURPOSES. ANYTHING INSIDE THIS TABLE
 * SHOULD BE PUSHED TO PINMUX SPREADSHEET FOR AUTOGEN OR FIXED
 * */
static __initdata struct tegra_pingroup_config manual_config_pinmux[] = {

	/* ULPI SFIOs are not supposed to be supported.
	 * This setting is only for tegranote7c. */
	DEFAULT_PINMUX(ULPI_DATA0,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA1,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA5,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA6,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA7,    ULPI,        NORMAL,    NORMAL,   INPUT),
};

static __initdata struct tegra_pingroup_config e2542_uart_config_pinmux[] = {
	DEFAULT_PINMUX(SDMMC3_CMD,    UARTA,       NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,   UARTA,       NORMAL,    NORMAL,   OUTPUT),
};

static void __init tegranote7c_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_tegranote7c_common);
	pins_info = init_gpio_mode_tegranote7c_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

#ifdef CONFIG_PM_SLEEP
/* pinmux settings during low power mode for special purpose */
static struct tegra_pingroup_config tegranote7c_sleep_pinmux[] = { };

/* gpio settings during low power mode for special purpose */
static struct gpio_init_pin_info tegranote7c_sleep_gpio[] = {
    /* Config RT6154 to PSM mode to save modem power */
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PQ1, false, 0),
};
#endif

int __init tegranote7c_pinmux_init(void)
{
	tegranote7c_gpio_init_configure();

	tegra_drive_pinmux_config_table(tegranote7c_drive_pinmux,
					ARRAY_SIZE(tegranote7c_drive_pinmux));

	tegra_pinmux_config_table(tegranote7c_pinmux_common,
				  ARRAY_SIZE(tegranote7c_pinmux_common));
	tegra_pinmux_config_table(tegranote7c_unused_pins_lowpower,
				  ARRAY_SIZE(tegranote7c_unused_pins_lowpower));

	tegra_pinmux_config_table(manual_config_pinmux,
		ARRAY_SIZE(manual_config_pinmux));
	if (get_tegra_uart_debug_port_id() == UART_FROM_SDCARD)
		tegra_pinmux_config_table(e2542_uart_config_pinmux,
			ARRAY_SIZE(e2542_uart_config_pinmux));
#ifdef CONFIG_PM_SLEEP
	tegra11x_set_sleep_pinmux(tegranote7c_sleep_pinmux,
		ARRAY_SIZE(tegranote7c_sleep_pinmux));
	tegra11x_set_sleep_gpio(tegranote7c_sleep_gpio,
		ARRAY_SIZE(tegranote7c_sleep_gpio));
#endif

	return 0;
}
