/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/stddef.h>

#ifdef CONFIG_SCHED_HMP
#include <../kernel/sched/sched.h>
#endif

#include "op_gpl_helper.h"

void op_block_hotplug(void)
{
	get_online_cpus();
}
EXPORT_SYMBOL(op_block_hotplug);

void op_unblock_hotplug(void)
{
	put_online_cpus();
}
EXPORT_SYMBOL(op_unblock_hotplug);

s64 op_get_time(void)
{
	return ktime_to_ms(ktime_get());
}
EXPORT_SYMBOL(op_get_time);

struct cpufreq_policy *op_get_policy(int cpu)
{
	return cpufreq_cpu_get(cpu);
}
EXPORT_SYMBOL(op_get_policy);

void op_put_policy(struct cpufreq_policy *policy)
{
	cpufreq_cpu_put(policy);
}
EXPORT_SYMBOL(op_put_policy);

struct device *op_find_cpu_device(unsigned cpu)
{
	return get_cpu_device(cpu);
}
EXPORT_SYMBOL(op_find_cpu_device);

int __ref op_online_core(unsigned int cpu)
{
	return cpu_up(cpu);
}
EXPORT_SYMBOL(op_online_core);

/* ic, extend hcube helper functionalities */
int __ref op_is_online_idle_core(unsigned int cpu)
{
	int ret = idle_cpu(cpu) && !nr_iowait_cpu(cpu);
	return ret;
}
EXPORT_SYMBOL(op_is_online_idle_core);

unsigned int __ref op_get_freq_cost(unsigned int cpu, unsigned int freq)
{
#ifdef CONFIG_SCHED_HMP
	return power_cost_at_freq(cpu, freq);
#else
	/* NOT supported */
	return 0;
#endif
}
EXPORT_SYMBOL(op_get_freq_cost);

unsigned int op_get_cost_at_temp(
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
EXPORT_SYMBOL(op_get_cost_at_temp);

struct cpufreq_frequency_table *op_get_freq_tbl(unsigned int cpu)
{
	return cpufreq_frequency_get_table(cpu);
}
EXPORT_SYMBOL(op_get_freq_tbl);

/* ted, add sysfs_create/remove_link
   for compatible with userspace perfd */
int op_sysfs_create_link(struct kobject *kobj, struct kobject *target,
      const char *name)
{
	return sysfs_create_link(kobj, target, name);
}
EXPORT_SYMBOL(op_sysfs_create_link);

void op_sysfs_remove_link(struct kobject *kobj, const char *name)
{
	sysfs_remove_link(kobj, name);
}
EXPORT_SYMBOL(op_sysfs_remove_link);

/* ted, retrieve msm soc info */
extern enum msm_cpu op_socinfo_get_msm_cpu(void)
{
	return socinfo_get_msm_cpu();
}
EXPORT_SYMBOL(op_socinfo_get_msm_cpu);

extern bool op_is_msm8994(void)
{
	return MSM_CPU_8994 == socinfo_get_msm_cpu();
}
EXPORT_SYMBOL(op_is_msm8994);

extern bool op_is_msm8996(void)
{
	return MSM_CPU_8996 == socinfo_get_msm_cpu();
}
EXPORT_SYMBOL(op_is_msm8996);
