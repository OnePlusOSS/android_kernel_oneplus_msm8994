/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <soc/qcom/core_ctl.h>
#ifdef CONFIG_SCHED_HMP
#include <../kernel/sched/sched.h>
#endif

void core_ctl_block_hotplug(void)
{
	get_online_cpus();
}
EXPORT_SYMBOL(core_ctl_block_hotplug);

void core_ctl_unblock_hotplug(void)
{
	put_online_cpus();
}
EXPORT_SYMBOL(core_ctl_unblock_hotplug);

s64 core_ctl_get_time(void)
{
	return ktime_to_ms(ktime_get());
}
EXPORT_SYMBOL(core_ctl_get_time);

struct cpufreq_policy *core_ctl_get_policy(int cpu)
{
	return cpufreq_cpu_get(cpu);
}
EXPORT_SYMBOL(core_ctl_get_policy);

void core_ctl_put_policy(struct cpufreq_policy *policy)
{
	cpufreq_cpu_put(policy);
}
EXPORT_SYMBOL(core_ctl_put_policy);

struct device *core_ctl_find_cpu_device(unsigned cpu)
{
	return get_cpu_device(cpu);
}
EXPORT_SYMBOL(core_ctl_find_cpu_device);

int __ref core_ctl_online_core(unsigned int cpu)
{
	return cpu_up(cpu);
}
EXPORT_SYMBOL(core_ctl_online_core);

#ifdef VENDOR_EDIT
/* ic, extend core ctrl helper functionalities */
int __ref core_ctl_is_online_idle_core(unsigned int cpu)
{
	int ret = idle_cpu(cpu) && !nr_iowait_cpu(cpu);
	return ret;
}
EXPORT_SYMBOL(core_ctl_is_online_idle_core);

unsigned int __ref core_ctl_get_freq_cost(unsigned int cpu, unsigned int freq)
{
#ifdef CONFIG_SCHED_HMP
	return power_cost_at_freq(cpu, freq);
#else
	/* NOT supported */
	return 0;
#endif
}
EXPORT_SYMBOL(core_ctl_get_freq_cost);

unsigned int core_ctl_get_cost_at_temp(
	unsigned int cpu,
	unsigned int freq,
	long temp)
{
#ifdef CONFIG_SCHED_HMP
	return power_cost_at_freq_at_temp(cpu, freq, temp);
#else
	/* NOT supported */
	return 0;
#endif
}
EXPORT_SYMBOL(core_ctl_get_cost_at_temp);

struct cpufreq_frequency_table *core_ctl_get_freq_tbl(unsigned int cpu)
{
	return cpufreq_frequency_get_table(cpu);
}
EXPORT_SYMBOL(core_ctl_get_freq_tbl);
#endif
