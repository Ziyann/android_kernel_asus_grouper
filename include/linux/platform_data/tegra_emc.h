/*
 * Copyright (C) 2011 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Olof Johansson <olof@lixom.net>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_EMC_H_
#define __TEGRA_EMC_H_

#define TEGRA_EMC_NUM_REGS 46

struct tegra_emc_table {
	unsigned long rate;
	u32 regs[TEGRA_EMC_NUM_REGS];
};

struct tegra_emc_pdata {
	const char *description;
	int mem_manufacturer_id; /* LPDDR2 MR5 or -1 to ignore */
	int mem_revision_id1;    /* LPDDR2 MR6 or -1 to ignore */
	int mem_revision_id2;    /* LPDDR2 MR7 or -1 to ignore */
	int mem_pid;             /* LPDDR2 MR8 or -1 to ignore */
	int num_tables;
	struct tegra_emc_table *tables;
};

/* !!!FIXME!!! Need actual Tegra11x values */
#define TEGRA11_EMC_MAX_NUM_REGS	110

struct tegra11_emc_table {
	u8 rev;
	unsigned long rate;
	int emc_min_mv;
	const char *src_name;

	/* unconditionally updated in one burst shot */
	u32 burst_regs[TEGRA11_EMC_MAX_NUM_REGS];

	/* updated separately under some conditions */
	u32 emc_zcal_cnt_long;
	u32 emc_acal_interval;
	u32 emc_periodic_qrst;
	u32 emc_mode_reset;
	u32 emc_mode_1;
	u32 emc_mode_2;
	u32 emc_dsr;
};

struct tegra11_emc_pdata {
	const char *description;
	int num_tables;
	struct tegra11_emc_table *tables;
};

#endif