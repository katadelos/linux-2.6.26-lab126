/*
 *	x86 SMP booting functions
 *
 *	(c) 1995 Alan Cox, Building #3 <alan@redhat.com>
 *	(c) 1998, 1999, 2000 Ingo Molnar <mingo@redhat.com>
 *	Copyright 2001 Andi Kleen, SuSE Labs.
 *
 *	Much of the core SMP work is based on previous work by Thomas Radke, to
 *	whom a great many thanks are extended.
 *
 *	Thanks to Intel for making available several different Pentium,
 *	Pentium Pro and Pentium-II/Xeon MP machines.
 *	Original development of Linux SMP code supported by Caldera.
 *
 *	This code is released under the GNU General Public License version 2 or
 *	later.
 *
 *	Fixes
 *		Felix Koop	:	NR_CPUS used properly
 *		Jose Renau	:	Handle single CPU case.
 *		Alan Cox	:	By repeated request 8) - Total BogoMIPS report.
 *		Greg Wright	:	Fix for kernel stacks panic.
 *		Erich Boleyn	:	MP v1.4 and additional changes.
 *	Matthias Sattler	:	Changes for 2.1 kernel map.
 *	Michel Lespinasse	:	Changes for 2.1 kernel map.
 *	Michael Chastain	:	Change trampoline.S to gnu as.
 *		Alan Cox	:	Dumb bug: 'B' step PPro's are fine
 *		Ingo Molnar	:	Added APIC timers, based on code
 *					from Jose Renau
 *		Ingo Molnar	:	various cleanups and rewrites
 *		Tigran Aivazian	:	fixed "0.00 in /proc/uptime on SMP" bug.
 *	Maciej W. Rozycki	:	Bits for genuine 82489DX APICs
 *	Andi Kleen		:	Changed for SMP boot into long mode.
 *		Martin J. Bligh	: 	Added support for multi-quad systems
 *		Dave Jones	:	Report invalid combinations of Athlon CPUs.
 *		Rusty Russell	:	Hacked into shape for new "hotplug" boot process.
 *      Andi Kleen              :       Converted to new state machine.
 *	Ashok Raj		: 	CPU hotplug support
 *	Glauber Costa		:	i386 and x86_64 integration
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/percpu.h>
#include <linux/bootmem.h>
#include <linux/err.h>
#include <linux/nmi.h>

#include <asm/acpi.h>
#include <asm/desc.h>
#include <asm/nmi.h>
#include <asm/irq.h>
#include <asm/smp.h>
#include <asm/trampoline.h>
#include <asm/cpu.h>
#include <asm/numa.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/mtrr.h>
#include <asm/nmi.h>
#include <asm/vmi.h>
#include <asm/genapic.h>
#include <linux/mc146818rtc.h>

#include <mach_apic.h>
#include <mach_wakecpu.h>
#include <smpboot_hooks.h>

/*
 * FIXME: For x86_64, those are defined in other files. But moving them here,
 * would make the setup areas dependent on smp, which is a loss. When we
 * integrate apic between arches, we can probably do a better job, but
 * right now, they'll stay here -- glommer
 */

/* which logical CPU number maps to which CPU (physical APIC ID) */
u16 x86_cpu_to_apicid_init[NR_CPUS] __initdata =
			{ [0 ... NR_CPUS-1] = BAD_APICID };
void *x86_cpu_to_apicid_early_ptr;

u16 x86_bios_cpu_apicid_init[NR_CPUS] __initdata
				= { [0 ... NR_CPUS-1] = BAD_APICID };
void *x86_bios_cpu_apicid_early_ptr;

#ifdef CONFIG_X86_32
u8 apicid_2_node[MAX_APICID];
static int low_mappings;
#endif

/* State of each CPU */
DEFINE_PER_CPU(int, cpu_state) = { 0 };

/* Store all idle threads, this can be reused instead of creating
* a new thread. Also avoids complicated thread destroy functionality
* for idle threads.
*/
#ifdef CONFIG_HOTPLUG_CPU
/*
 * Needed only for CONFIG_HOTPLUG_CPU because __cpuinitdata is
 * removed after init for !CONFIG_HOTPLUG_CPU.
 */
static DEFINE_PER_CPU(struct task_struct *, idle_thread_array);
#define get_idle_for_cpu(x)      (per_cpu(idle_thread_array, x))
#define set_idle_for_cpu(x, p)   (per_cpu(idle_thread_array, x) = (p))
#else
struct task_struct *idle_thread_array[NR_CPUS] __cpuinitdata ;
#define get_idle_for_cpu(x)      (idle_thread_array[(x)])
#define set_idle_for_cpu(x, p)   (idle_thread_array[(x)] = (p))
#endif

/* Number of siblings per CPU package */
int smp_num_siblings = 1;
EXPORT_SYMBOL(smp_num_siblings);

/* Last level cache ID of each logical CPU */
DEFINE_PER_CPU(u16, cpu_llc_id) = BAD_APICID;

/* bitmap of online cpus */
cpumask_t cpu_online_map __read_mostly;
EXPORT_SYMBOL(cpu_online_map);

cpumask_t cpu_callin_map;
cpumask_t cpu_callout_map;
cpumask_t cpu_possible_map;
EXPORT_SYMBOL(cpu_possible_map);

/* representing HT siblings of each logical CPU */
DEFINE_PER_CPU(cpumask_t, cpu_sibling_map);
EXPORT_PER_CPU_SYMBOL(cpu_sibling_map);

/* representing HT and core siblings of each logical CPU */
DEFINE_PER_CPU(cpumask_t, cpu_core_map);
EXPORT_PER_CPU_SYMBOL(cpu_core_map);

/* Per CPU bogomips and other parameters */
DEFINE_PER_CPU_SHARED_ALIGNED(struct cpuinfo_x86, cpu_info);
EXPORT_PER_CPU_SYMBOL(cpu_info);

static atomic_t init_deasserted;

static int boot_cpu_logical_apicid;

/* representing cpus for which sibling maps can be computed */
static cpumask_t cpu_sibling_setup_map;

/* Set if we find a B stepping CPU */
int __cpuinitdata smp_b_stepping;

#if defined(CONFIG_NUMA) && defined(CONFIG_X86_32)

/* which logical CPUs are on which nodes */
cpumask_t node_to_cpumask_map[MAX_NUMNODES] __read_mostly =
				{ [0 ... MAX_NUMNODES-1] = CPU_MASK_NONE };
EXPORT_SYMBOL(node_to_cpumask_map);
/* which node each logical CPU is on */
int cpu_to_node_map[NR_CPUS] __read_mostly = { [0 ... NR_CPUS-1] = 0 };
EXPORT_SYMBOL(cpu_to_node_map);

/* set up a mapping between cpu and node. */
static void map_cpu_to_node(int cpu, int node)
{
	printk(KERN_INFO "Mapping cpu %d to node %d\n", cpu, node);
	cpu_set(cpu, node_to_cpumask_map[node]);
	cpu_to_node_map[cpu] = node;
}

/* undo a mapping between cpu and node. */
static void unmap_cpu_to_node(int cpu)
{
	int node;

	printk(KERN_INFO "Unmapping cpu %d from all nodes\n", cpu);
	for (node = 0; node < MAX_NUMNODES; node++)
		cpu_clear(cpu, node_to_cpumask_map[node]);
	cpu_to_node_map[cpu] = 0;
}
#else /* !(CONFIG_NUMA && CONFIG_X86_32) */
#define map_cpu_to_node(cpu, node)	({})
#define unmap_cpu_to_node(cpu)	({})
#endif

#ifdef CONFIG_X86_32
u8 cpu_2_logical_apicid[NR_CPUS] __read_mostly =
					{ [0 ... NR_CPUS-1] = BAD_APICID };

static void map_cpu_to_logical_apicid(void)
{
	int cpu = smp_processor_id();
	int apicid = logical_smp_processor_id();
	int node = apicid_to_node(apicid);

	if (!node_online(node))
		node = first_online_node;

	cpu_2_logical_apicid[cpu] = apicid;
	map_cpu_to_node(cpu, node);
}

static void unmap_cpu_to_logical_apicid(int cpu)
{
	cpu_2_logical_apicid[cpu] = BAD_APICID;
	unmap_cpu_to_node(cpu);
}
#else
#define unmap_cpu_to_logical_apicid(cpu) do {} while (0)
#define map_cpu_to_logical_apicid()  do {} while (0)
#endif

/*
 * Report back to the Boot Processor.
 * Running on AP.
 */
static void __cpuinit smp_callin(void)
{
	int cpuid, phys_id;
	unsigned long timeout;

	/*
	 * If waken up by an INIT in an 82489DX configuration
	 * we may get here before an INIT-deassert IPI reaches
	 * our local APIC.  We have to wait for the IPI or we'll
	 * lock up on an APIC access.
	 */
	wait_for_init_deassert(&init_deasserted);

	/*
	 * (This works even if the APIC is not enabled.)
	 */
	phys_id = GET_APIC_ID(read_apic_id());
	cpuid = smp_processor_id();
	if (cpu_isset(cpuid, cpu_callin_map)) {
		panic("%s: phys CPU#%d, CPU#%d already present??\n", __func__,
					phys_id, cpuid);
	}
	Dprintk("CPU#%d (phys ID: %d) waiting for CALLOUT\n", cpuid, phys_id);

	/*
	 * STARTUP IPIs are fragile beasts as they might sometimes
	 * trigger some glue motherboard logic. Complete APIC bus
	 * silence for 1 second, this overestimates the time the
	 * boot CPU is spending to send the up to 2 STARTUP IPIs
	 * by a factor of two. This should be enough.
	 */

	/*
	 * Waiting 2s total for startup (udelay is not yet working)
	 */
	timeout = jiffies + 2*HZ;
	while (time_before(jiffies, timeout)) {
		/*
		 * Has the boot CPU finished it's STARTUP sequence?
		 */
		if (cpu_isset(cpuid, cpu_callout_map))
			break;
		cpu_relax();
	}

	if (!time_before(jiffies, timeout)) {
		panic("%s: CPU%d started up but did not get a callout!\n",
		      __func__, cpuid);
	}

	/*
	 * the boot CPU has finished the init stage and is spinning
	 * on callin_map until we finish. We are free to set up this
	 * CPU, first the APIC. (this is probably redundant on most
	 * boards)
	 */

	Dprintk("CALLIN, before setup_local_APIC().\n");
	smp_callin_clear_local_apic();
	setup_local_APIC();
	end_local_APIC_setup();
	map_cpu_to_logical_apicid();

	/*
	 * Get our bogomips.
	 *
	 * Need to enable IRQs because it can take longer and then
	 * the NMI watchdog might kill us.
	 */
	local_irq_enable();
	calibrate_delay();
	local_irq_disable();
	Dprintk("Stack at about %p\n", &cpuid);

	/*
	 * Save our processor parameters
	 */
	smp_store_cpu_info(cpuid);

	/*
	 * Allow the master to continue.
	 */
	cpu_set(cpuid, cpu_callin_map);
}

/*
 * Activate a secondary processor.
 */
static void __cpuinit start_secondary(void *unused)
{
	/*
	 * Don't put *anything* before cpu_init(), SMP booting is too
	 * fragile that we want to limit the things done here to the
	 * most necessary things.
	 */
#ifdef CONFIG_VMI
	vmi_bringup();
#endif
	cpu_init();
	preempt_disable();
	smp_callin();

	/* otherwise gcc will move up smp_processor_id before the cpu_init */
	barrier();
	/*
	 * Check TSC synchronization with the BP:
	 */
	check_tsc_sync_target();

	if (nmi_watchdog == NMI_IO_APIC) {
		disable_8259A_irq(0);
		enable_NMI_through_LVT0();
		enable_8259A_irq(0);
	}

#ifdef CONFIG_X86_32
	while (low_mappings)
		cpu_relax();
	__flush_tlb_all();
#endif

	/* This must be done before setting cpu_online_map */
	set_cpu_sibling_map(raw_smp_processor_id());
	wmb();

	/*
	 * We need to hold call_lock, so there is no inconsistency
	 * between the time smp_call_function() determines number of
	 * IPI recipients, and the time when the determination is made
	 * for which cpus receive the IPI. Holding this
	 * lock helps us to not include this cpu in a currently in progress
	 * smp_call_function().
	 */
	lock_ipi_call_lock();
#ifdef CONFIG_X86_64
	spin_lock(&vector_lock);

	/* Setup the per cpu irq handling data structures */
	__setup_vector_irq(smp_processor_id());
	/*
	 * Allow the master to continue.
	 */
	spin_unlock(&vector_lock);
#endif
	cpu_set(smp_processor_id(), cpu_online_map);
	unlock_ipi_call_lock();
	per_cpu(cpu_state, smp_processor_id()) = CPU_ONLINE;

	setup_secondary_clock();

	wmb();
	cpu_idle();
}

#ifdef CONFIG_X86_32
/*
 * Everything has been set up for the secondary
 * CPUs - they just need to reload everything
 * from the task structure
 * This function must not return.
 */
void __devinit initialize_secondary(void)
{
	/*
	 * We don't actually need to load the full TSS,
	 * basically just the stack pointer and the ip.
	 */

	asm volatile(
		"movl %0,%%esp\n\t"
		"jmp *%1"
		:
		:"m" (current->thread.sp), "m" (current->thread.ip));
}
#endif

static void __cpuinit smp_apply_quirks(struct cpuinfo_x86 *c)
{
#ifdef CONFIG_X86_32
	/*
	 * Mask B, Pentium, but not Pentium MMX
	 */
	if (c->x86_vendor == X86_VENDOR_INTEL &&
	    c->x86 == 5 &&
	    c->x86_mask >= 1 && c->x86_mask <= 4 &&
	    c->x86_model <= 3)
		/*
		 * Remember we have B step Pentia with bugs
		 */
		smp_b_stepping = 1;

	/*
	 * Certain Athlons might work (for various values of 'work') in SMP
	 * but they are not certified as MP capable.
	 */
	if ((c->x86_vendor == X86_VENDOR_AMD) && (c->x86 == 6)) {

		if (num_possible_cpus() == 1)
			goto valid_k7;

		/* Athlon 660/661 is valid. */
		if ((c->x86_model == 6) && ((c->x86_mask == 0) ||
		    (c->x86_mask == 1)))
			goto valid_k7;

		/* Duron 670 is valid */
		if ((c->x86_model == 7) && (c->x86_mask == 0))
			goto valid_k7;

		/*
		 * Athlon 662, Duron 671, and Athlon >model 7 have capability
		 * bit. It's worth noting that the A5 stepping (662) of some
		 * Athlon XP's have the MP bit set.
		 * See http://www.heise.de/newsticker/data/jow-18.10.01-000 for
		 * more.
		 */
		if (((c->x86_model == 6) && (c->x86_mask >= 2)) ||
		    ((c->x86_model == 7) && (c->x86_mask >= 1)) ||
		     (c->x86_model > 7))
			if (cpu_has_mp)
				goto valid_k7;

		/* If we get here, not a certified SMP capable AMD system. */
		add_taint(TAINT_UNSAFE_SMP);
	}

valid_k7:
	;
#endif
}

static void __cpuinit smp_checks(void)
{
	if (smp_b_stepping)
		printk(KERN_WARNING "WARNING: SMP operation may be unreliable"
				    "with B stepping processors.\n");

	/*
	 * Don't taint if we are running SMP kernel on a single non-MP
	 * approved Athlon
	 */
	if (tainted & TAINT_UNSAFE_SMP) {
		if (num_online_cpus())
			printk(KERN_INFO "WARNING: This combination of AMD"
				"processors is not suitable for SMP.\n");
		else
			tainted &= ~TAINT_UNSAFE_SMP;
	}
}

/*
 * The bootstrap kernel entry code has set these up. Save them for
 * a given CPU
 */

void __cpuinit smp_store_cpu_info(int id)
{
	struct cpuinfo_x86 *c = &cpu_data(id);

	*c = boot_cpu_data;
	c->cpu_index = id;
	if (id != 0)
		identify_secondary_cpu(c);
	smp_apply_quirks(c);
}


void __cpuinit set_cpu_sibling_map(int cpu)
{
	int i;
	struct cpuinfo_x86 *c = &cpu_data(cpu);

	cpu_set(cpu, cpu_sibling_setup_map);

	if (smp_num_siblings > 1) {
		for_each_cpu_mask(i, cpu_sibling_setup_map) {
			if (c->phys_proc_id == cpu_data(i).phys_proc_id &&
			    c->cpu_core_id == cpu_data(i).cpu_core_id) {
				cpu_set(i, per_cpu(cpu_sibling_map, cpu));
				cpu_set(cpu, per_cpu(cpu_sibling_map, i));
				cpu_set(i, per_cpu(cpu_core_map, cpu));
				cpu_set(cpu, per_cpu(cpu_core_map, i));
				cpu_set(i, c->llc_shared_map);
				cpu_set(cpu, cpu_data(i).llc_shared_map);
			}
		}
	} else {
		cpu_set(cpu, per_cpu(cpu_sibling_map, cpu));
	}

	cpu_set(cpu, c->llc_shared_map);

	if (current_cpu_data.x86_max_cores == 1) {
		per_cpu(cpu_core_map, cpu) = per_cpu(cpu_sibling_map, cpu);
		c->booted_cores = 1;
		return;
	}

	for_each_cpu_mask(i, cpu_sibling_setup_map) {
		if (per_cpu(cpu_llc_id, cpu) != BAD_APICID &&
		    per_cpu(cpu_llc_id, cpu) == per_cpu(cpu_llc_id, i)) {
			cpu_set(i, c->llc_shared_map);
			cpu_set(cpu, cpu_data(i).llc_shared_map);
		}
		if (c->phys_proc_id == cpu_data(i).phys_proc_id) {
			cpu_set(i, per_cpu(cpu_core_map, cpu));
			cpu_set(cpu, per_cpu(cpu_core_map, i));
			/*
			 *  Does this new cpu bringup a new core?
			 */
			if (cpus_weight(per_cpu(cpu_sibling_map, cpu)) == 1) {
				/*
				 * for each core in package, increment
				 * the booted_cores for this new cpu
				 */
				if (first_cpu(per_cpu(cpu_sibling_map, i)) == i)
					c->booted_cores++;
				/*
				 * increment the core count for all
				 * the other cpus in this package
				 */
				if (i != cpu)
					cpu_data(i).booted_cores++;
			} else if (i != cpu && !c->booted_cores)
				c->booted_cores = cpu_data(i).booted_cores;
		}
	}
}

/* maps the cpu to the sched domain representing multi-core */
cpumask_t cpu_coregroup_map(int cpu)
{
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	/*
	 * For perf, we return last level cache shared map.
	 * And for power savings, we return cpu_core_map
	 */
	if (sched_mc_power_savings || sched_smt_power_savings)
		return per_cpu(cpu_core_map, cpu);
	else
		return c->llc_shared_map;
}

#ifdef CONFIG_X86_32
/*
 * We are called very early to get the low memory for the
 * SMP bootup trampoline page.
 */
void __init smp_alloc_memory(void)
{
	trampoline_base = alloc_bootmem_low_pages(PAGE_SIZE);
	/*
	 * Has to be in very low memory so we can execute
	 * real-mode AP code.
	 */
	if (__pa(trampoline_base) >= 0x9F000)
		BUG();
}
#endif

static void impress_friends(void)
{
	int cpu;
	unsigned long bogosum = 0;
	/*
	 * Allow the user to impress friends.
	 */
	Dprintk("Before bogomips.\n");
	for_each_possible_cpu(cpu)
		if (cpu_isset(cpu, cpu_callout_map))
			bogosum += cpu_data(cpu).loops_per_jiffy;
	printk(KERN_INFO
		"Total of %d processors activated (%lu.%02lu BogoMIPS).\n",
		num_online_cpus(),
		bogosum/(500000/HZ),
		(bogosum/(5000/HZ))%100);

	Dprintk("Before bogocount - setting activated=1.\n");
}

static inline void __inquire_remote_apic(int apicid)
{
	unsigned i, regs[] = { APIC_ID >> 4, APIC_LVR >> 4, APIC_SPIV >> 4 };
	char *names[] = { "ID", "VERSION", "SPIV" };
	int timeout;
	u32 status;

	printk(KERN_INFO "Inquiring remote APIC #%d...\n", apicid);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		printk(KERN_INFO "... APIC #%d %s: ", apicid, names[i]);

		/*
		 * Wait for idle.
		 */
		status = safe_apic_wait_icr_idle();
		if (status)
			printk(KERN_CONT
			       "a previous APIC delivery may have failed\n");

		apic_write_around(APIC_ICR2, SET_APIC_DEST_FIELD(apicid));
		apic_write_around(APIC_ICR, APIC_DM_REMRD | regs[i]);

		timeout = 0;
		do {
			udelay(100);
			status = apic_read(APIC_ICR) & APIC_ICR_RR_MASK;
		} while (status == APIC_ICR_RR_INPROG && timeout++ < 1000);

		switch (status) {
		case APIC_ICR_RR_VALID:
			status = apic_read(APIC_RRR);
			printk(KERN_CONT "%08x\n", status);
			break;
		default:
			printk(KERN_CONT "failed\n");
		}
	}
}

#ifdef WAKE_SECONDARY_VIA_NMI
/*
 * Poke the other CPU in the eye via NMI to wake it up. Remember that the normal
 * INIT, INIT, STARTUP sequence will reset the chip hard for us, and this
 * won't ... remember to clear down the APIC, etc later.
 */
static int __devinit
wakeup_secondary_cpu(int logical_apicid, unsigned long start_eip)
{
	unsigned long send_status, accept_status = 0;
	int maxlvt;

	/* Target chip */
	apic_write_around(APIC_ICR2, SET_APIC_DEST_FIELD(logical_apicid));

	/* Boot on the stack */
	/* Kick the second */
	apic_write_around(APIC_ICR, APIC_DM_NMI | APIC_DEST_LOGICAL);

	Dprintk("Waiting for send to finish...\n");
	send_status = safe_apic_wait_icr_idle();

	/*
	 * Give the other CPU some time to accept the IPI.
	 */
	udelay(200);
	/*
	 * Due to the Pentium erratum 3AP.
	 */
	maxlvt = lapic_get_maxlvt();
	if (maxlvt > 3) {
		apic_read_around(APIC_SPIV);
		apic_write(APIC_ESR, 0);
	}
	accept_status = (apic_read(APIC_ESR) & 0xEF);
	Dprintk("NMI sent.\n");

	if (send_status)
		printk(KERN_ERR "APIC never delivered???\n");
	if (accept_status)
		printk(KERN_ERR "APIC delivery error (%lx).\n", accept_status);

	return (send_status | accept_status);
}
#endif	/* WAKE_SECONDARY_VIA_NMI */

#ifdef WAKE_SECONDARY_VIA_INIT
static int __devinit
wakeup_secondary_cpu(int phys_apicid, unsigned long start_eip)
{
	unsigned long send_status, accept_status = 0;
	int maxlvt, num_starts, j;

	if (get_uv_system_type() == UV_NON_UNIQUE_APIC) {
		send_status = uv_wakeup_secondary(phys_apicid, start_eip);
		atomic_set(&init_deasserted, 1);
		return send_status;
	}

	/*
	 * Be paranoid about clearing APIC errors.
	 */
	if (APIC_INTEGRATED(apic_version[phys_apicid])) {
		apic_read_around(APIC_SPIV);
		apic_write(APIC_ESR, 0);
		apic_read(APIC_ESR);
	}

	Dprintk("Asserting INIT.\n");

	/*
	 * Turn INIT on target chip
	 */
	apic_write_around(APIC_ICR2, SET_APIC_DEST_FIELD(phys_apicid));

	/*
	 * Send IPI
	 */
	apic_write_around(APIC_ICR, APIC_INT_LEVELTRIG | APIC_INT_ASSERT
				| APIC_DM_INIT);

	Dprintk("Waiting for send to finish...\n");
	send_status = safe_apic_wait_icr_idle();

	mdelay(10);

	Dprintk("Deasserting INIT.\n");

	/* Target chip */
	apic_write_around(APIC_ICR2, SET_APIC_DEST_FIELD(phys_apicid));

	/* Send IPI */
	apic_write_around(APIC_ICR, APIC_INT_LEVELTRIG | APIC_DM_INIT);

	Dprintk("Waiting for send to finish...\n");
	send_status = safe_apic_wait_icr_idle();

	mb();
	atomic_set(&init_deasserted, 1);

	/*
	 * Should we send STARTUP IPIs ?
	 *
	 * Determine this based on the APIC version.
	 * If we don't have an integrated APIC, don't send the STARTUP IPIs.
	 */
	if (APIC_INTEGRATED(apic_version[phys_apicid]))
		num_starts = 2;
	else
		num_starts = 0;

	/*
	 * Paravirt / VMI wants a startup IPI hook here to set up the
	 * target processor state.
	 */
	startup_ipi_hook(phys_apicid, (unsigned long) start_secondary,
#ifdef CONFIG_X86_64
			 (unsigned long)init_rsp);
#else
			 (unsigned long)stack_start.sp);
#endif

	/*
	 * Run STARTUP IPI loop.
	 */
	Dprintk("#startup loops: %d.\n", num_starts);

	maxlvt = lapic_get_maxlvt();

	for (j = 1; j <= num_starts; j++) {
		Dprintk("Sending STARTUP #%d.\n", j);
		apic_read_around(APIC_SPIV);
		apic_write(APIC_ESR, 0);
		apic_read(APIC_ESR);
		Dprintk("After apic_write.\n");

		/*
		 * STARTUP IPI
		 */

		/* Target chip */
		apic_write_around(APIC_ICR2, SET_APIC_DEST_FIELD(phys_apicid));

		/* Boot on the stack */
		/* Kick the second */
		apic_write_around(APIC_ICR, APIC_DM_STARTUP
					| (start_eip >> 12));

		/*
		 * Give the other CPU some time to accept the IPI.
		 */
		udelay(300);

		Dprintk("Startup point 1.\n");

		Dprintk("Waiting for send to finish...\n");
		send_status = safe_apic_wait_icr_idle();

		/*
		 * Give the other CPU some time to accept the IPI.
		 */
		udelay(200);
		/*
		 * Due to the Pentium erratum 3AP.
		 */
		if (maxlvt > 3) {
			apic_read_around(APIC_SPIV);
			apic_write(APIC_ESR, 0);
		}
		accept_status = (apic_read(APIC_ESR) & 0xEF);
		if (send_status || accept_status)
			break;
	}
	Dprintk("After Startup.\n");

	if (send_status)
		printk(KERN_ERR "APIC never delivered???\n");
	if (accept_status)
		printk(KERN_ERR "APIC delivery error (%lx).\n", accept_status);

	return (send_status | accept_status);
}
#endif	/* WAKE_SECONDARY_VIA_INIT */

struct create_idle {
	struct work_struct work;
	struct task_struct *idle;
	struct completion done;
	int cpu;
};

static void __cpuinit do_fork_idle(struct work_struct *work)
{
	struct create_idle *c_idle =
		container_of(work, struct create_idle, work);

	c_idle->idle = fork_idle(c_idle->cpu);
	complete(&c_idle->done);
}

static int __cpuinit do_boot_cpu(int apicid, int cpu)
/*
 * NOTE - on most systems this is a PHYSICAL apic ID, but on multiquad
 * (ie clustered apic addressing mode), this is a LOGICAL apic ID.
 * Returns zero if CPU booted OK, else error code from wakeup_secondary_cpu.
 */
{
	unsigned long boot_error = 0;
	int timeout;
	unsigned long start_ip;
	unsigned short nmi_high = 0, nmi_low = 0;
	struct create_idle c_idle = {
		.cpu = cpu,
		.done = COMPLETION_INITIALIZER_ONSTACK(c_idle.done),
	};
	INIT_WORK(&c_idle.work, do_fork_idle);
#ifdef CONFIG_X86_64
	/* allocate memory for gdts of secondary cpus. Hotplug is considered */
	if (!cpu_gdt_descr[cpu].address &&
		!(cpu_gdt_descr[cpu].address = get_zeroed_page(GFP_KERNEL))) {
		printk(KERN_ERR "Failed to allocate GDT for CPU %d\n", cpu);
		return -1;
	}

	/* Allocate node local memory for AP pdas */
	if (cpu_pda(cpu) == &boot_cpu_pda[cpu]) {
		struct x8664_pda *newpda, *pda;
		int node = cpu_to_node(cpu);
		pda = cpu_pda(cpu);
		newpda = kmalloc_node(sizeof(struct x8664_pda), GFP_ATOMIC,
				      node);
		if (newpda) {
			memcpy(newpda, pda, sizeof(struct x8664_pda));
			cpu_pda(cpu) = newpda;
		} else
			printk(KERN_ERR
		"Could not allocate node local PDA for CPU %d on node %d\n",
				cpu, node);
	}
#endif

	alternatives_smp_switch(1);

	c_idle.idle = get_idle_for_cpu(cpu);

	/*
	 * We can't use kernel_thread since we must avoid to
	 * reschedule the child.
	 */
	if (c_idle.idle) {
		c_idle.idle->thread.sp = (unsigned long) (((struct pt_regs *)
			(THREAD_SIZE +  task_stack_page(c_idle.idle))) - 1);
		init_idle(c_idle.idle, cpu);
		goto do_rest;
	}

	if (!keventd_up() || current_is_keventd())
		c_idle.work.func(&c_idle.work);
	else {
		schedule_work(&c_idle.work);
		wait_for_completion(&c_idle.done);
	}

	if (IS_ERR(c_idle.idle)) {
		printk("failed fork for CPU %d\n", cpu);
		return PTR_ERR(c_idle.idle);
	}

	set_idle_for_cpu(cpu, c_idle.idle);
do_rest:
#ifdef CONFIG_X86_32
	per_cpu(current_task, cpu) = c_idle.idle;
	init_gdt(cpu);
	early_gdt_descr.address = (unsigned long)get_cpu_gdt_table(cpu);
	c_idle.idle->thread.ip = (unsigned long) start_secondary;
	/* Stack for startup_32 can be just as for start_secondary onwards */
	stack_start.sp = (void *) c_idle.idle->thread.sp;
	irq_ctx_init(cpu);
#else
	cpu_pda(cpu)->pcurrent = c_idle.idle;
	init_rsp = c_idle.idle->thread.sp;
	load_sp0(&per_cpu(init_tss, cpu), &c_idle.idle->thread);
	initial_code = (unsigned long)start_secondary;
	clear_tsk_thread_flag(c_idle.idle, TIF_FORK);
#endif

	/* start_ip had better be page-aligned! */
	start_ip = setup_trampoline();

	/* So we see what's up   */
	printk(KERN_INFO "Booting processor %d/%d ip %lx\n",
			  cpu, apicid, start_ip);

	/*
	 * This grunge runs the startup process for
	 * the targeted processor.
	 */

	atomic_set(&init_deasserted, 0);

	if (get_uv_system_type() != UV_NON_UNIQUE_APIC) {

		Dprintk("Setting warm reset code and vector.\n");

		store_NMI_vector(&nmi_high, &nmi_low);

		smpboot_setup_warm_reset_vector(start_ip);
		/*
		 * Be paranoid about clearing APIC errors.
	 	*/
		apic_write(APIC_ESR, 0);
		apic_read(APIC_ESR);
	}

	/*
	 * Starting actual IPI sequence...
	 */
	boot_error = wakeup_secondary_cpu(apicid, start_ip);

	if (!boot_error) {
		/*
		 * allow APs to start initializing.
		 */
		Dprintk("Before Callout %d.\n", cpu);
		cpu_set(cpu, cpu_callout_map);
		Dprintk("After Callout %d.\n", cpu);

		/*
		 * Wait 5s total for a response
		 */
		for (timeout = 0; timeout < 50000; timeout++) {
			if (cpu_isset(cpu, cpu_callin_map))
				break;	/* It has booted */
			udelay(100);
		}

		if (cpu_isset(cpu, cpu_callin_map)) {
			/* number CPUs logically, starting from 1 (BSP is 0) */
			Dprintk("OK.\n");
			printk(KERN_INFO "CPU%d: ", cpu);
			print_cpu_info(&cpu_data(cpu));
			Dprintk("CPU has booted.\n");
		} else {
			boot_error = 1;
			if (*((volatile unsigned char *)trampoline_base)
					== 0xA5)
				/* trampoline started but...? */
				printk(KERN_ERR "Stuck ??\n");
			else
				/* trampoline code not run */
				printk(KERN_ERR "Not responding.\n");
			if (get_uv_system_type() != UV_NON_UNIQUE_APIC)
				inquire_remote_apic(apicid);
		}
	}

	if (boot_error) {
		/* Try to put things back the way they were before ... */
		unmap_cpu_to_logical_apicid(cpu);
#ifdef CONFIG_X86_64
		clear_node_cpumask(cpu); /* was set by numa_add_cpu */
#endif
		cpu_clear(cpu, cpu_callout_map); /* was set by do_boot_cpu() */
		cpu_clear(cpu, cpu_initialized); /* was set by cpu_init() */
		cpu_clear(cpu, cpu_present_map);
		per_cpu(x86_cpu_to_apicid, cpu) = BAD_APICID;
	}

	/* mark "stuck" area as not stuck */
	*((volatile unsigned long *)trampoline_base) = 0;

	/*
	 * Cleanup possible dangling ends...
	 */
	smpboot_restore_warm_reset_vector();

	return boot_error;
}

int __cpuinit native_cpu_up(unsigned int cpu)
{
	int apicid = cpu_present_to_apicid(cpu);
	unsigned long flags;
	int err;

	WARN_ON(irqs_disabled());

	Dprintk("++++++++++++++++++++=_---CPU UP  %u\n", cpu);

	if (apicid == BAD_APICID || apicid == boot_cpu_physical_apicid ||
	    !physid_isset(apicid, phys_cpu_present_map)) {
		printk(KERN_ERR "%s: bad cpu %d\n", __func__, cpu);
		return -EINVAL;
	}

	/*
	 * Already booted CPU?
	 */
	if (cpu_isset(cpu, cpu_callin_map)) {
		Dprintk("do_boot_cpu %d Already started\n", cpu);
		return -ENOSYS;
	}

	/*
	 * Save current MTRR state in case it was changed since early boot
	 * (e.g. by the ACPI SMI) to initialize new CPUs with MTRRs in sync:
	 */
	mtrr_save_state();

	per_cpu(cpu_state, cpu) = CPU_UP_PREPARE;

#ifdef CONFIG_X86_32
	/* init low mem mapping */
	clone_pgd_range(swapper_pg_dir, swapper_pg_dir + KERNEL_PGD_BOUNDARY,
		min_t(unsigned long, KERNEL_PGD_PTRS, KERNEL_PGD_BOUNDARY));
	flush_tlb_all();
	low_mappings = 1;

	err = do_boot_cpu(apicid, cpu);

	zap_low_mappings();
	low_mappings = 0;
#else
	err = do_boot_cpu(apicid, cpu);
#endif
	if (err) {
		Dprintk("do_boot_cpu failed %d\n", err);
		return -EIO;
	}

	/*
	 * Check TSC synchronization with the AP (keep irqs disabled
	 * while doing so):
	 */
	local_irq_save(flags);
	check_tsc_sync_source(cpu);
	local_irq_restore(flags);

	while (!cpu_online(cpu)) {
		cpu_relax();
		touch_nmi_watchdog();
	}

	return 0;
}

/*
 * Fall back to non SMP mode after errors.
 *
 * RED-PEN audit/test this more. I bet there is more state messed up here.
 */
static __init void disable_smp(void)
{
	cpu_present_map = cpumask_of_cpu(0);
	cpu_possible_map = cpumask_of_cpu(0);
#ifdef CONFIG_X86_32
	smpboot_clear_io_apic_irqs();
#endif
	if (smp_found_config)
		phys_cpu_present_map =
				physid_mask_of_physid(boot_cpu_physical_apicid);
	else
		phys_cpu_present_map = physid_mask_of_physid(0);
	map_cpu_to_logical_apicid();
	cpu_set(0, per_cpu(cpu_sibling_map, 0));
	cpu_set(0, per_cpu(cpu_core_map, 0));
}

/*
 * Various sanity checks.
 */
static int __init smp_sanity_check(unsigned max_cpus)
{
	preempt_disable();
	if (!physid_isset(hard_smp_processor_id(), phys_cpu_present_map)) {
		printk(KERN_WARNING "weird, boot CPU (#%d) not listed"
				    "by the BIOS.\n", hard_smp_processor_id());
		physid_set(hard_smp_processor_id(), phys_cpu_present_map);
	}

	/*
	 * If we couldn't find an SMP configuration at boot time,
	 * get out of here now!
	 */
	if (!smp_found_config && !acpi_lapic) {
		preempt_enable();
		printk(KERN_NOTICE "SMP motherboard not detected.\n");
		disable_smp();
		if (APIC_init_uniprocessor())
			printk(KERN_NOTICE "Local APIC not detected."
					   " Using dummy APIC emulation.\n");
		return -1;
	}

	/*
	 * Should not be necessary because the MP table should list the boot
	 * CPU too, but we do it for the sake of robustness anyway.
	 */
	if (!check_phys_apicid_present(boot_cpu_physical_apicid)) {
		printk(KERN_NOTICE
			"weird, boot CPU (#%d) not listed by the BIOS.\n",
			boot_cpu_physical_apicid);
		physid_set(hard_smp_processor_id(), phys_cpu_present_map);
	}
	preempt_enable();

	/*
	 * If we couldn't find a local APIC, then get out of here now!
	 */
	if (APIC_INTEGRATED(apic_version[boot_cpu_physical_apicid]) &&
	    !cpu_has_apic) {
		printk(KERN_ERR "BIOS bug, local APIC #%d not detected!...\n",
			boot_cpu_physical_apicid);
		printk(KERN_ERR "... forcing use of dummy APIC emulation."
				"(tell your hw vendor)\n");
		smpboot_clear_io_apic();
		return -1;
	}

	verify_local_APIC();

	/*
	 * If SMP should be disabled, then really disable it!
	 */
	if (!max_cpus) {
		printk(KERN_INFO "SMP mode deactivated,"
				 "forcing use of dummy APIC emulation.\n");
		smpboot_clear_io_apic();
#ifdef CONFIG_X86_32
		connect_bsp_APIC();
#endif
		setup_local_APIC();
		end_local_APIC_setup();
		return -1;
	}

	return 0;
}

static void __init smp_cpu_index_default(void)
{
	int i;
	struct cpuinfo_x86 *c;

	for_each_possible_cpu(i) {
		c = &cpu_data(i);
		/* mark all to hotplug */
		c->cpu_index = NR_CPUS;
	}
}

/*
 * Prepare for SMP bootup.  The MP table or ACPI has been read
 * earlier.  Just do some sanity checking here and enable APIC mode.
 */
void __init native_smp_prepare_cpus(unsigned int max_cpus)
{
	preempt_disable();
	smp_cpu_index_default();
	current_cpu_data = boot_cpu_data;
	cpu_callin_map = cpumask_of_cpu(0);
	mb();
	/*
	 * Setup boot CPU information
	 */
	smp_store_cpu_info(0); /* Final full version of the data */
	boot_cpu_logical_apicid = logical_smp_processor_id();
	current_thread_info()->cpu = 0;  /* needed? */
	set_cpu_sibling_map(0);

	if (smp_sanity_check(max_cpus) < 0) {
		printk(KERN_INFO "SMP disabled\n");
		disable_smp();
		goto out;
	}

	preempt_disable();
	if (GET_APIC_ID(read_apic_id()) != boot_cpu_physical_apicid) {
		panic("Boot APIC ID in local APIC unexpected (%d vs %d)",
		     GET_APIC_ID(read_apic_id()), boot_cpu_physical_apicid);
		/* Or can we switch back to PIC here? */
	}
	preempt_enable();

#ifdef CONFIG_X86_32
	connect_bsp_APIC();
#endif
	/*
	 * Switch from PIC to APIC mode.
	 */
	setup_local_APIC();

#ifdef CONFIG_X86_64
	/*
	 * Enable IO APIC before setting up error vector
	 */
	if (!skip_ioapic_setup && nr_ioapics)
		enable_IO_APIC();
#endif
	end_local_APIC_setup();

	map_cpu_to_logical_apicid();

	setup_portio_remap();

	smpboot_setup_io_apic();
	/*
	 * Set up local APIC timer on boot CPU.
	 */

	printk(KERN_INFO "CPU%d: ", 0);
	print_cpu_info(&cpu_data(0));
	setup_boot_clock();
out:
	preempt_enable();
}
/*
 * Early setup to make printk work.
 */
void __init native_smp_prepare_boot_cpu(void)
{
	int me = smp_processor_id();
#ifdef CONFIG_X86_32
	init_gdt(me);
	switch_to_new_gdt();
#endif
	/* already set me in cpu_online_map in boot_cpu_init() */
	cpu_set(me, cpu_callout_map);
	per_cpu(cpu_state, me) = CPU_ONLINE;
}

void __init native_smp_cpus_done(unsigned int max_cpus)
{
	Dprintk("Boot done.\n");

	impress_friends();
	smp_checks();
#ifdef CONFIG_X86_IO_APIC
	setup_ioapic_dest();
#endif
	check_nmi_watchdog();
}

#ifdef CONFIG_HOTPLUG_CPU

#  ifdef CONFIG_X86_32
void cpu_exit_clear(void)
{
	int cpu = raw_smp_processor_id();

	idle_task_exit();

	cpu_uninit();
	irq_ctx_exit(cpu);

	cpu_clear(cpu, cpu_callout_map);
	cpu_clear(cpu, cpu_callin_map);

	unmap_cpu_to_logical_apicid(cpu);
}
#  endif /* CONFIG_X86_32 */

static void remove_siblinginfo(int cpu)
{
	int sibling;
	struct cpuinfo_x86 *c = &cpu_data(cpu);

	for_each_cpu_mask(sibling, per_cpu(cpu_core_map, cpu)) {
		cpu_clear(cpu, per_cpu(cpu_core_map, sibling));
		/*/
		 * last thread sibling in this cpu core going down
		 */
		if (cpus_weight(per_cpu(cpu_sibling_map, cpu)) == 1)
			cpu_data(sibling).booted_cores--;
	}

	for_each_cpu_mask(sibling, per_cpu(cpu_sibling_map, cpu))
		cpu_clear(cpu, per_cpu(cpu_sibling_map, sibling));
	cpus_clear(per_cpu(cpu_sibling_map, cpu));
	cpus_clear(per_cpu(cpu_core_map, cpu));
	c->phys_proc_id = 0;
	c->cpu_core_id = 0;
	cpu_clear(cpu, cpu_sibling_setup_map);
}

static int additional_cpus __initdata = -1;

static __init int setup_additional_cpus(char *s)
{
	return s && get_option(&s, &additional_cpus) ? 0 : -EINVAL;
}
early_param("additional_cpus", setup_additional_cpus);

/*
 * cpu_possible_map should be static, it cannot change as cpu's
 * are onlined, or offlined. The reason is per-cpu data-structures
 * are allocated by some modules at init time, and dont expect to
 * do this dynamically on cpu arrival/departure.
 * cpu_present_map on the other hand can change dynamically.
 * In case when cpu_hotplug is not compiled, then we resort to current
 * behaviour, which is cpu_possible == cpu_present.
 * - Ashok Raj
 *
 * Three ways to find out the number of additional hotplug CPUs:
 * - If the BIOS specified disabled CPUs in ACPI/mptables use that.
 * - The user can overwrite it with additional_cpus=NUM
 * - Otherwise don't reserve additional CPUs.
 * We do this because additional CPUs waste a lot of memory.
 * -AK
 */
__init void prefill_possible_map(void)
{
	int i;
	int possible;

	if (additional_cpus == -1) {
		if (disabled_cpus > 0)
			additional_cpus = disabled_cpus;
		else
			additional_cpus = 0;
	}
	possible = num_processors + additional_cpus;
	if (possible > NR_CPUS)
		possible = NR_CPUS;

	printk(KERN_INFO "SMP: Allowing %d CPUs, %d hotplug CPUs\n",
		possible, max_t(int, possible - num_processors, 0));

	for (i = 0; i < possible; i++)
		cpu_set(i, cpu_possible_map);
}

static void __ref remove_cpu_from_maps(int cpu)
{
	cpu_clear(cpu, cpu_online_map);
#ifdef CONFIG_X86_64
	cpu_clear(cpu, cpu_callout_map);
	cpu_clear(cpu, cpu_callin_map);
	/* was set by cpu_init() */
	clear_bit(cpu, (unsigned long *)&cpu_initialized);
	clear_node_cpumask(cpu);
#endif
}

int __cpu_disable(void)
{
	int cpu = smp_processor_id();

	/*
	 * Perhaps use cpufreq to drop frequency, but that could go
	 * into generic code.
	 *
	 * We won't take down the boot processor on i386 due to some
	 * interrupts only being able to be serviced by the BSP.
	 * Especially so if we're not using an IOAPIC	-zwane
	 */
	if (cpu == 0)
		return -EBUSY;

	if (nmi_watchdog == NMI_LOCAL_APIC)
		stop_apic_nmi_watchdog(NULL);
	clear_local_APIC();

	/*
	 * HACK:
	 * Allow any queued timer interrupts to get serviced
	 * This is only a temporary solution until we cleanup
	 * fixup_irqs as we do for IA64.
	 */
	local_irq_enable();
	mdelay(1);

	local_irq_disable();
	remove_siblinginfo(cpu);

	/* It's now safe to remove this processor from the online map */
	remove_cpu_from_maps(cpu);
	fixup_irqs(cpu_online_map);
	return 0;
}

void __cpu_die(unsigned int cpu)
{
	/* We don't do anything here: idle task is faking death itself. */
	unsigned int i;

	for (i = 0; i < 10; i++) {
		/* They ack this in play_dead by setting CPU_DEAD */
		if (per_cpu(cpu_state, cpu) == CPU_DEAD) {
			printk(KERN_INFO "CPU %d is now offline\n", cpu);
			if (1 == num_online_cpus())
				alternatives_smp_switch(0);
			return;
		}
		msleep(100);
	}
	printk(KERN_ERR "CPU %u didn't die...\n", cpu);
}
#else /* ... !CONFIG_HOTPLUG_CPU */
int __cpu_disable(void)
{
	return -ENOSYS;
}

void __cpu_die(unsigned int cpu)
{
	/* We said "no" in __cpu_disable */
	BUG();
}
#endif

/*
 * If the BIOS enumerates physical processors before logical,
 * maxcpus=N at enumeration-time can be used to disable HT.
 */
static int __init parse_maxcpus(char *arg)
{
	extern unsigned int maxcpus;

	maxcpus = simple_strtoul(arg, NULL, 0);
	return 0;
}
early_param("maxcpus", parse_maxcpus);
