/*
 * Read-Copy Update mechanism for mutual exclusion (RT implementation)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) IBM Corporation, 2006
 *
 * Author:  Paul McKenney <paulmck@us.ibm.com>
 *
 * Based on the original work by Paul McKenney <paul.mckenney@us.ibm.com>
 * and inputs from Rusty Russell, Andrea Arcangeli and Andi Kleen.
 * Papers:
 * http://www.rdrop.com/users/paulmck/paper/rclockpdcsproof.pdf
 * http://lse.sourceforge.net/locking/rclock_OLS.2001.05.01c.sc.pdf (OLS2001)
 *
 * For detailed explanation of Read-Copy Update mechanism see -
 * 		Documentation/RCU
 *
 */

#ifndef __LINUX_RCUPREEMPT_H
#define __LINUX_RCUPREEMPT_H

#include <linux/cache.h>
#include <linux/spinlock.h>
#include <linux/threads.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/seqlock.h>

struct rcu_dyntick_sched {
	int dynticks;
	int dynticks_snap;
	int sched_qs;
	int sched_qs_snap;
	int sched_dynticks_snap;
};

DECLARE_PER_CPU(struct rcu_dyntick_sched, rcu_dyntick_sched);

static inline void rcu_qsctr_inc(int cpu)
{
	struct rcu_dyntick_sched *rdssp = &per_cpu(rcu_dyntick_sched, cpu);

	rdssp->sched_qs++;
}
#define rcu_bh_qsctr_inc(cpu)

/*
 * Someone might want to pass call_rcu_bh as a function pointer.
 * So this needs to just be a rename and not a macro function.
 *  (no parentheses)
 */
#define call_rcu_bh	 	call_rcu

#ifdef CONFIG_PREEMPT_RCU_BOOST
/*
 * Task state with respect to being RCU-boosted.  This state is changed
 * by the task itself in response to the following three events:
 * 1. Preemption (or block on lock) while in RCU read-side critical section.
 * 2. Outermost rcu_read_unlock() for blocked RCU read-side critical section.
 *
 * The RCU-boost task also updates the state when boosting priority.
 */
enum rcu_boost_state {
	RCU_BOOST_IDLE = 0,	   /* Not yet blocked if in RCU read-side. */
	RCU_BOOST_BLOCKED = 1,	   /* Blocked from RCU read-side. */
	RCU_BOOSTED = 2,	   /* Boosting complete. */
	RCU_BOOST_INVALID = 3,	   /* For bogus state sightings. */
};

#define N_RCU_BOOST_STATE (RCU_BOOST_INVALID + 1)

int __init rcu_preempt_boost_init(void);

#else /* CONFIG_PREEPMT_RCU_BOOST */

#define rcu_preempt_boost_init() do { } while (0)

#endif /* CONFIG_PREEMPT_RCU_BOOST */

/**
 * call_rcu_sched - Queue RCU callback for invocation after sched grace period.
 * @head: structure to be used for queueing the RCU updates.
 * @func: actual update function to be invoked after the grace period
 *
 * The update function will be invoked some time after a full
 * synchronize_sched()-style grace period elapses, in other words after
 * all currently executing preempt-disabled sections of code (including
 * hardirq handlers, NMI handlers, and local_irq_save() blocks) have
 * completed.
 */
extern void call_rcu_sched(struct rcu_head *head,
			   void (*func)(struct rcu_head *head));

extern void __rcu_read_lock(void)	__acquires(RCU);
extern void __rcu_read_unlock(void)	__releases(RCU);
extern int rcu_pending(int cpu);
extern int rcu_needs_cpu(int cpu);

#define __rcu_read_lock_bh()	{ rcu_read_lock(); local_bh_disable(); }
#define __rcu_read_unlock_bh()	{ local_bh_enable(); rcu_read_unlock(); }

extern void __synchronize_sched(void);

extern void __rcu_init(void);
extern void rcu_init_sched(void);
extern void rcu_check_callbacks(int cpu, int user);
extern void rcu_restart_cpu(int cpu);
extern long rcu_batches_completed(void);

/*
 * Return the number of RCU batches processed thus far. Useful for debug
 * and statistic. The _bh variant is identifcal to straight RCU
 */
static inline long rcu_batches_completed_bh(void)
{
	return rcu_batches_completed();
}

#ifdef CONFIG_RCU_TRACE
struct rcupreempt_trace;
extern long *rcupreempt_flipctr(int cpu);
extern long rcupreempt_data_completed(void);
extern int rcupreempt_flip_flag(int cpu);
extern int rcupreempt_mb_flag(int cpu);
extern char *rcupreempt_try_flip_state_name(void);
extern struct rcupreempt_trace *rcupreempt_trace_cpu(int cpu);
#endif

struct softirq_action;

#ifdef CONFIG_NO_HZ

static inline void rcu_enter_nohz(void)
{
	smp_mb(); /* CPUs seeing ++ must see prior RCU read-side crit sects */
	__get_cpu_var(rcu_dyntick_sched).dynticks++;
	if (unlikely(__get_cpu_var(rcu_dyntick_sched).dynticks & 0x1)) {
		printk("BUG: bad accounting of dynamic ticks\n");
		printk("   will try to fix, but it is best to reboot\n");
		WARN_ON(1);
		/* try to fix it */
		__get_cpu_var(rcu_dyntick_sched).dynticks++;
	}
}

static inline void rcu_exit_nohz(void)
{
	__get_cpu_var(rcu_dyntick_sched).dynticks++;
	smp_mb(); /* CPUs seeing ++ must see later RCU read-side crit sects */
	if (unlikely(!(__get_cpu_var(rcu_dyntick_sched).dynticks & 0x1))) {
		printk("BUG: bad accounting of dynamic ticks\n");
		printk("   will try to fix, but it is best to reboot\n");
		WARN_ON(1);
		/* try to fix it */
		__get_cpu_var(rcu_dyntick_sched).dynticks++;
	}
}

#else /* CONFIG_NO_HZ */
#define rcu_enter_nohz()	do { } while (0)
#define rcu_exit_nohz()		do { } while (0)
#endif /* CONFIG_NO_HZ */

#endif /* __LINUX_RCUPREEMPT_H */
