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

#ifndef __SOC_QCOM_CORE_CTL_H
#define __SOC_QCOM_CORE_CTL_H

extern void core_ctl_block_hotplug(void);
extern void core_ctl_unblock_hotplug(void);
extern s64 core_ctl_get_time(void);
extern struct cpufreq_policy *core_ctl_get_policy(int cpu);
extern void core_ctl_put_policy(struct cpufreq_policy *policy);
extern struct device *core_ctl_find_cpu_device(unsigned cpu);
extern int core_ctl_online_core(unsigned int cpu);
#ifdef VENDOR_EDIT
/* ic, declarations for core ctrl extension */
extern int core_ctl_is_online_idle_core(unsigned int cpu);
extern unsigned int core_ctl_get_freq_cost(
	unsigned int cpu,
	unsigned int freq
	);
extern unsigned int core_ctl_get_cost_at_temp(
	unsigned int cpu,
	unsigned int freq,
	long temp
	);
extern struct cpufreq_frequency_table *core_ctl_get_freq_tbl(unsigned int cpu);
#endif
#endif
