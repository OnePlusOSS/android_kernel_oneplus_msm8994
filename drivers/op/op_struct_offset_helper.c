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

#include <linux/stddef.h>
#include <linux/cpufreq.h>

#include "op_struct_offset_helper.h"
#include "../kernel/sched/sched.h"

/* struct cpufreq_policy */
int _cpufreq_policy_offset[__CPUFREQ_POLICY_MAX] = {
	[CPUFREQ_POLICY_MIN] = offsetof(struct cpufreq_policy, min),
	[CPUFREQ_POLICY_MAX] = offsetof(struct cpufreq_policy, max),
	[CPUFREQ_POLICY_CUR] = offsetof(struct cpufreq_policy, cur),
	[CPUFREQ_POLICY_RELATED_CPUS] = offsetof(struct cpufreq_policy, related_cpus),
	[CPUFREQ_POLICY_CPUINFO] = offsetof(struct cpufreq_policy, cpuinfo)
};
gen_type_offset_impl(cpufreq_policy);

/* struct cpufreq_govinfo */
int _cpufreq_govinfo_offset[__CPUFREQ_GOVINFO_MAX] = {
	[CPUFREQ_GOVINFO_CPU] = offsetof(struct cpufreq_govinfo, cpu),
	[CPUFREQ_GOVINFO_LOAD] = offsetof(struct cpufreq_govinfo, load)
};
gen_type_offset_impl(cpufreq_govinfo);

/* struct cpufreq_cpuinfo */
int _cpufreq_cpuinfo_offset[__CPUFREQ_CPUINFO_MAX] = {
	[CPUFREQ_CPUINFO_MAX_FREQ] = offsetof(struct cpufreq_cpuinfo, max_freq)
};
gen_type_offset_impl(cpufreq_cpuinfo);

/* struct task_struct */
int task_struct_offset[__TASK_OFFSET_MAX] = {
	[TASK_OFFSET_PID] = offsetof(struct task_struct, pid),
	[TASK_OFFSET_TGID] = offsetof(struct task_struct, tgid),
	[TASK_OFFSET_GROUP_LEADER] = offsetof(struct task_struct, group_leader),
	[TASK_OFFSET_COMM] = offsetof(struct task_struct, comm),
};

int get_task_struct_offset(int member)
{
	return task_struct_offset[member];
}
EXPORT_SYMBOL(get_task_struct_offset);

/* struct rq */
int rq_struct_offset[__RQ_OFFSET_MAX] = {
	[RQ_OFFSET_CLOCK] = offsetof(struct rq, clock)
};

int get_rq_struct_offset(int member)
{
	return rq_struct_offset[member];
}
EXPORT_SYMBOL(get_rq_struct_offset);
