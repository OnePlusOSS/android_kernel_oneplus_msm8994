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
#ifndef _OP_GPL_HELPER_INC_
#define _OP_GPL_HELPER_INC_

#include <soc/qcom/socinfo.h>

extern void op_block_hotplug(void);
extern void op_unblock_hotplug(void);

extern s64  op_get_time(void);

extern struct cpufreq_policy *
			op_get_policy(int cpu);
extern void op_put_policy(struct cpufreq_policy *policy);
extern struct cpufreq_frequency_table *
			op_get_freq_tbl(unsigned int cpu);

extern struct device *
			op_find_cpu_device(unsigned cpu);

extern int  op_online_core(unsigned int cpu);

extern int  op_is_online_idle_core(unsigned int cpu);

extern unsigned int op_get_freq_cost(
	unsigned int cpu,
	unsigned int freq
	);
extern unsigned int op_get_cost_at_temp(
	unsigned int cpu,
	unsigned int freq,
	long temp
	);

extern int  op_sysfs_create_link(
	struct kobject *kobj,
	struct kobject *target,
	const char *name);
extern void op_sysfs_remove_link(
	struct kobject *kobj,
	const char *name);

extern enum msm_cpu op_socinfo_get_msm_cpu(void);
extern bool op_is_msm8994(void);
extern bool op_is_msm8996(void);
#endif //_OP_GPL_HELPER_INC_
