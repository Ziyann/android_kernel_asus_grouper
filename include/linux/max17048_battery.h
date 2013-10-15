/*
 * Copyright (C) 2009 Samsung Electronics
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17048_BATTERY_H_
#define __MAX17048_BATTERY_H_
#include <linux/smb349-charger.h>

#define MAX17048_DATA_SIZE 64
#define MAX17048_MAX_SOC_STEP 10

struct max17048_battery_model {
	uint8_t rcomp;
	uint8_t soccheck_A;
	uint8_t soccheck_B;
	uint8_t bits;
	uint8_t alert_threshold;
	uint8_t one_percent_alerts;
	uint8_t alert_on_reset;
	uint16_t rcomp_seg;
	uint16_t hibernate;
	uint16_t vreset;
	uint16_t valert;
	uint16_t ocvtest;
	int t_co_hot;
	int t_co_cold;
	uint8_t data_tbl[MAX17048_DATA_SIZE];
};

struct max17048_platform_data {
	struct max17048_battery_model *model_data;

	u32 read_batt_id;

	s32 (*set_current_threshold)(s32 current_threshold, int min_cpu);
	int current_normal;
	int current_threshold_num;
	int current_threshold_soc[MAX17048_MAX_SOC_STEP];
	int current_threshold[MAX17048_MAX_SOC_STEP];

	void (*sysedp_throttle)(unsigned int power);
	int sysedp_throttle_num;
	int sysedp_throttle_soc[MAX17048_MAX_SOC_STEP];
	unsigned int sysedp_throttle_power[MAX17048_MAX_SOC_STEP];
};

void max17048_battery_status(int status, int chrg_type);
int max17048_check_battery(void);
int max17048_check_soc(void);
int max17048_check_vcell(void);
#endif
