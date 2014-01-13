/*
 * arch/arm/mach-tegra/board-tegranote7c-powermon.c
 *
 * Copyright (c) 2014, NVIDIA Corporation. All Rights Reserved.
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
#include <linux/i2c.h>
#include <linux/ina219.h>
#include <linux/platform_data/ina230.h>

#include "board.h"
#include "board-tegranote7c.h"
#include "tegra-board-id.h"

#define PRECISION_MULTIPLIER_TEGRANOTE7C 1000

enum {
	VD_CPU,
	VD_SOC,
	VS_DDR0,
	VS_DDR1,
	VD_LCD_HV,
	VS_SYS_1V8,
	VD_AP_1V8,
	VD_AP_RTC,
	VS_AUD_SYS,
	VD_DDR0,
	VD_DDR1,
	VD_AP_VBUS,
	VS_SYS_2V9,
	VA_PLLX,
	VA_AP_1V2,
};

enum {
	INA_I2C_ADDR_40,
	INA_I2C_ADDR_41,
	INA_I2C_ADDR_42,
	INA_I2C_ADDR_43,
	INA_I2C_ADDR_45,
	INA_I2C_ADDR_46,
	INA_I2C_ADDR_47,
	INA_I2C_ADDR_48,
	INA_I2C_ADDR_49,
	INA_I2C_ADDR_4A,
	INA_I2C_ADDR_4B,
	INA_I2C_ADDR_4C,
	INA_I2C_ADDR_4D,
	INA_I2C_ADDR_4E,
	INA_I2C_ADDR_4F,
};

enum {
	VDD_CELL
};

static struct ina230_platform_data power_mon_ina230_info[] = {
	[VDD_CELL] = {
		.calibration_data  = 0x20c4,
		.power_lsb = 3.051757813 * PRECISION_MULTIPLIER_TEGRANOTE7C,
		.rail_name = "VDD_CELL",
		.resistor = 5,
		.min_cores_online = 2,
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_TEGRANOTE7C,
		/* Current threshold for detect system overcurrent.
		   Yoku(4100mA/3.7V) suggest 4.5A for max continuous
		   discharge current. It is 6.5A in datasheet. */
		.current_threshold = TEGRANOTE7C_BATTERY_MAX_CURRENT,
		.shunt_polarity_inverted = 1,
	}
};

enum {
	INA_I2C_ADDR_44
};

static struct i2c_board_info tegranote7c_i2c1_ina230_board_info[] = {
	[INA_I2C_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &power_mon_ina230_info[VDD_CELL],
		.irq = -1,
	}
};

int __init tegranote7c_pmon_init(void)
{
	/* P1988 has ina230 for checking power at VDD_CELL */
	i2c_register_board_info(1, tegranote7c_i2c1_ina230_board_info,
		ARRAY_SIZE(tegranote7c_i2c1_ina230_board_info));

	return 0;
}
