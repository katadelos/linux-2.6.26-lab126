#ifndef _ASMi386_TIMER_H
#define _ASMi386_TIMER_H
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/percpu.h>

#define TICK_SIZE (tick_nsec / 1000)

unsigned long long native_sched_clock(void);
unsigned long native_calculate_cpu_khz(void);

extern int timer_ack;
extern int no_timer_check;
extern int recalibrate_cpu_khz(void);

#ifndef CONFIG_PARAVIRT
#define calculate_cpu_khz() native_calculate_cpu_khz()
#endif

/* Accelerators for sched_clock()
 * convert from cycles(64bits) => nanoseconds (64bits)
 *  basic equation:
 *		ns = cycles / (freq / ns_per_sec)
 *		ns = cycles * (ns_per_sec / freq)
 *		ns = cycles * (10^9 / (cpu_khz * 10^3))
 *		ns = cycles * (10^6 / cpu_khz)
 *
 *	Then we use scaling math (suggested by george@mvista.com) to get:
 *		ns = cycles * (10^6 * SC / cpu_khz) / SC
 *		ns = cycles * cyc2ns_scale / SC
 *
 *	And since SC is a constant power of two, we can convert the div
 *  into a shift.
 *
 *  We can use khz divisor instead of mhz to keep a better precision, since
 *  cyc2ns_scale is limited to 10^6 * 2^10, which fits in 32 bits.
 *  (mathieu.desnoyers@polymtl.ca)
 *
 *			-johnstul@us.ibm.com "math is hard, lets go shopping!"
 */

DECLARE_PER_CPU(unsigned long, cyc2ns);

#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */

static inline notrace unsigned long long __cycles_2_ns(unsigned long long cyc)
{
	return cyc * per_cpu(cyc2ns, smp_processor_id()) >> CYC2NS_SCALE_FACTOR;
}

static inline notrace unsigned long long cycles_2_ns(unsigned long long cyc)
{
	unsigned long long ns;
	unsigned long flags;

	raw_local_irq_save(flags);
	ns = __cycles_2_ns(cyc);
	raw_local_irq_restore(flags);

	return ns;
}

#endif
