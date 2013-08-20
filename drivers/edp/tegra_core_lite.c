/*
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

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/edp.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_data/tegra_edp.h>

struct freqcap {
	unsigned int cpu;
	unsigned int gpu;
	unsigned int emc;
};

static unsigned int online_cpu_count;
static struct tegra_sysedp_corecap *cur_corecap;
static struct clk *emc_cap_clk;
static struct clk *gpu_cap_clk;
static struct pm_qos_request cpufreq_qos;
static struct tegra_sysedp_platform_data *core_platdata;
static struct freqcap core_policy;
static struct freqcap cur_caps;
static DEFINE_MUTEX(core_lock);

/* To save some cycles from a linear search */
static unsigned int cpu_lut_match(unsigned int power,
		struct tegra_system_edp_entry *lut, unsigned int lutlen)
{
	unsigned int fv;
	unsigned int lv;
	unsigned int step;
	unsigned int i;

	if (lutlen == 1)
		return 0;

	fv = lut[0].power_limit_100mW * 100;
	lv = lut[lutlen - 1].power_limit_100mW * 100;
	step = (lv - fv) / (lutlen - 1);

	i = (power - fv + step - 1) / step;
	i = min_t(unsigned int, i, lutlen - 1);
	if (lut[i].power_limit_100mW * 100 >= power)
		return i;

	/* Didn't work, search back from the end */
	return lutlen - 1;
}

static unsigned int get_cpufreq_lim(unsigned int power)
{
	struct tegra_system_edp_entry *p;
	int i;

	i = cpu_lut_match(power, core_platdata->cpufreq_lim,
			core_platdata->cpufreq_lim_size);
	p = core_platdata->cpufreq_lim + i;

	return p->freq_limits[online_cpu_count - 1];
}

static void pr_caps(struct freqcap *old, struct freqcap *new,
		unsigned int cpu_power)
{
	if (!IS_ENABLED(CONFIG_DEBUG_KERNEL))
		return;

	if (new->cpu == old->cpu &&
			new->gpu == old->gpu &&
			new->emc == old->emc)
		return;

	pr_debug("sysedp: ncpus %u, core %5u mW," \
			" cpu %5u mW %u kHz, gpu %u kHz, emc %u kHz\n",
			online_cpu_count, cur_corecap->power,
			cpu_power, new->cpu, new->gpu, new->emc);
}

static void apply_caps(struct tegra_sysedp_devcap *devcap)
{
	struct freqcap new;
	int r;

	core_policy.cpu = get_cpufreq_lim(devcap->cpu_power);
	core_policy.gpu = devcap->gpufreq * 1000;
	core_policy.emc = devcap->emcfreq * 1000;

	new.cpu = core_policy.cpu;
	new.gpu = core_policy.gpu;
	new.emc = core_policy.emc;

	if (new.cpu != cur_caps.cpu)
		pm_qos_update_request(&cpufreq_qos, new.cpu);

	if (new.emc != cur_caps.emc) {
		r = clk_set_rate(emc_cap_clk, new.emc * 1000);
		WARN_ON(r);
	}

	if (new.gpu != cur_caps.gpu) {
		r = clk_set_rate(gpu_cap_clk, new.gpu * 1000);
		WARN_ON(r);
	}

	pr_caps(&cur_caps, &new, devcap->cpu_power);
	cur_caps = new;
}

static void __do_cap_control(void)
{
	struct tegra_sysedp_devcap *cap;

	if (!cur_corecap)
		return;

	cap = &cur_corecap->cpupri;
	apply_caps(cap);
}

static void do_cap_control(void)
{
	mutex_lock(&core_lock);
	__do_cap_control();
	mutex_unlock(&core_lock);
}

static void update_cur_corecap(unsigned int power)
{
	struct tegra_sysedp_corecap *cap;
	int i;

	if (!core_platdata)
		return;

	i = core_platdata->corecap_size - 1;
	cap = core_platdata->corecap + i;

	for (; i >= 0; i--, cap--) {
		if (cap->power <= power) {
			cur_corecap = cap;
			return;
		}
	}

	WARN_ON(1);
	cur_corecap = core_platdata->corecap;
}

void sysedp_lite_throttle(unsigned int power)
{
	mutex_lock(&core_lock);
	update_cur_corecap(power);
	__do_cap_control();
	mutex_unlock(&core_lock);
}
EXPORT_SYMBOL_GPL(sysedp_lite_throttle);

static int tegra_edp_cpu_notify(struct notifier_block *nb,
		unsigned long action, void *data)
{
	switch (action) {
	case CPU_UP_PREPARE:
		online_cpu_count = num_online_cpus() + 1;
		break;
	case CPU_DEAD:
		online_cpu_count = num_online_cpus();
		break;
	default:
		return NOTIFY_OK;
	}

	do_cap_control();
	return NOTIFY_OK;
}

static struct notifier_block tegra_edp_cpu_nb = {
	.notifier_call = tegra_edp_cpu_notify
};

static __devinit int init_clks(void)
{
	emc_cap_clk = clk_get_sys("battery_edp", "emc");
	if (IS_ERR(emc_cap_clk))
		return -ENODEV;

	gpu_cap_clk = clk_get_sys("battery_edp", "gpu");
	if (IS_ERR(gpu_cap_clk)) {
		clk_put(emc_cap_clk);
		return -ENODEV;
	}

	return 0;
}

static __devinit int tegra_sysedp_lite_probe(struct platform_device *pdev)
{
	int r;

	if (!pdev->dev.platform_data)
		return -EINVAL;

	online_cpu_count = num_online_cpus();
	pm_qos_add_request(&cpufreq_qos, PM_QOS_CPU_FREQ_MAX,
			PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

	r = register_cpu_notifier(&tegra_edp_cpu_nb);
	if (r)
		return r;

	r = init_clks();
	if (r)
		return r;

	mutex_lock(&core_lock);
	core_platdata = pdev->dev.platform_data;
	/* no limit */
	update_cur_corecap(ULONG_MAX);
	__do_cap_control();
	mutex_unlock(&core_lock);

	return 0;
}

static struct platform_driver tegra_sysedp_lite_driver = {
	.probe = tegra_sysedp_lite_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra_sysedp_lite"
	}
};

static __init int tegra_sysedp_lite_init(void)
{
	return platform_driver_register(&tegra_sysedp_lite_driver);
}
late_initcall(tegra_sysedp_lite_init);
