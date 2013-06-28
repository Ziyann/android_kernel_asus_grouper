/*
 * Gas Gauge driver for SBS Compliant Gas Gauges
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __LINUX_POWER_SBS_BATTERY_H_
#define __LINUX_POWER_SBS_BATTERY_H_

#include <linux/power_supply.h>
#include <linux/types.h>

/**
 * struct sbs_platform_data - platform data for sbs devices
 * @battery_detect:		GPIO which is used to detect battery presence
 * @battery_detect_present:	gpio state when battery is present (0 / 1)
 * @i2c_retry_count:		# of times to retry on i2c IO failure
 * @poll_retry_count:		# of times to retry looking for new status after
 *				external change notification
 */
struct sbs_platform_data {
	int battery_detect;
	int battery_detect_present;
	int i2c_retry_count;
	int poll_retry_count;
};

#define SBS_REG_DATA_FLASH_SUBCLASS_ID		0x77
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE1	0x78
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE2	0x79
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE3	0x7a
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE4	0x7b
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE5	0x7c
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE6	0x7d
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE7	0x7e
#define SBS_REG_DATA_FLASH_SUBCLASS_PAGE8	0x7f
#define SBS_REG_MANUFACTURER_ACCESS		0x00
#define SBS_REG_UNSEAL_KEY			0x60
#define SBS_REG_FULLACCESS_KEY			0x61

#define SBS_DATA_FLASH_MAX_BUFFER_SIZE		256

#define SBS_CONFIGURATION_SUBCLASS_ID		64
#define SBS_CONFIGURATION_SUBCLASS_SIZE_BYTES	10
#define SBS_CONFIGURATION_CFG_C_OFFSET		4
#define SBS_CONFIGURATION_RSOCL_MASK		0x01

extern int sbs_battery_detect(void);
extern void sbs_update(void);
#endif
