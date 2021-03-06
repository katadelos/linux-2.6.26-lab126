#ifndef _LINUX_FTRACE_H
#define _LINUX_FTRACE_H

#include <linux/tracepoint.h>
#include <linux/ktime.h>

#ifdef CONFIG_FTRACE

#include <linux/linkage.h>
#include <linux/sched.h>
#include <linux/fs.h>

extern int ftrace_enabled;
extern int
ftrace_enable_sysctl(struct ctl_table *table, int write,
		     struct file *filp, void __user *buffer, size_t *lenp,
		     loff_t *ppos);

typedef void (*ftrace_func_t)(unsigned long ip, unsigned long parent_ip);

struct ftrace_ops {
	ftrace_func_t	  func;
	struct ftrace_ops *next;
};

extern int function_trace_stop;

/**
 * ftrace_stop - stop function tracer.
 *
 * A quick way to stop the function tracer. Note this an on off switch,
 * it is not something that is recursive like preempt_disable.
 * This does not disable the calling of mcount, it only stops the
 * calling of functions from mcount.
 */
static inline void ftrace_stop(void)
{
	function_trace_stop = 1;
}

/**
 * ftrace_start - start the function tracer.
 *
 * This function is the inverse of ftrace_stop. This does not enable
 * the function tracing if the function tracer is disabled. This only
 * sets the function tracer flag to contiune calling the functions
 * from mcount.
 */
static inline void ftrace_start(void)
{
	function_trace_stop = 0;
}

/*
 * The ftrace_ops must be a static and should also
 * be read_mostly.  These functions do modify read_mostly variables
 * so use them sparely. Never free an ftrace_op or modify the
 * next pointer after it has been registered. Even after unregistering
 * it, the next pointer may still be used internally.
 */
int register_ftrace_function(struct ftrace_ops *ops);
int unregister_ftrace_function(struct ftrace_ops *ops);
void clear_ftrace_function(void);

extern void ftrace_stub(unsigned long a0, unsigned long a1);

void ftrace_enable(void);
void ftrace_disable(void);

/* totally disable ftrace - can not re-enable after this */
void ftrace_kill(void);
void __ftrace_kill(void);
void ftrace_kill_atomic(void);

/**
 * ftrace_preempt_disable - preempt disable used by function tracers
 *
 * The function tracer needs to be careful in disabling preemption.
 * If it calls preempt_enable() inside the scheduler, we may cause
 * a recursive inifinite loop.
 *
 * Returns flag to be used by ftrace_preempt_enable()
 */
static inline int ftrace_preempt_disable(void)
{
	int resched;

	resched = need_resched();
	barrier();
	preempt_disable_notrace();

	return resched;
}

/**
 * ftrace_preempt_enable - preempt enable used by function tracers
 * @resched: variable returned by corresponding ftrace_preempt_disable
 *
 * If NEED_RESCHED was set before we disabed preemption, since we
 * have not preempted yet, we either have interrupts off, preemption
 * off, or we are inside the scheduler.  The first two are OK,
 * but if resched is set and we are in the scheduler, calling
 * preempt_enable that reschedules will cause a recursion back into
 * the scheduler that will crash the box.
 */
static inline void ftrace_preempt_enable(int resched)
{
	if (resched)
		preempt_enable_no_resched_notrace();
	else
		preempt_enable_notrace();
}

#else /* !CONFIG_FTRACE */
# define register_ftrace_function(ops) do { } while (0)
# define unregister_ftrace_function(ops) do { } while (0)
# define clear_ftrace_function(ops) do { } while (0)
static inline void ftrace_kill_atomic(void) { }
# define ftrace_enable()			do { } while (0)
# define ftrace_disable()			do { } while (0)
# define ftrace_kill()				do { } while (0)
# define __ftrace_kill()			do { } while (0)
# define ftrace_kill_atomic()			do { } while (0)
# define function_trace_stop			0
# define ftrace_stop()				do { } while (0)
# define ftrace_start()				do { } while (0)
#endif /* CONFIG_FTRACE */

#ifdef CONFIG_DYNAMIC_FTRACE
# define FTRACE_HASHBITS	10
# define FTRACE_HASHSIZE	(1<<FTRACE_HASHBITS)

enum {
	FTRACE_FL_FREE		= (1 << 0),
	FTRACE_FL_FAILED	= (1 << 1),
	FTRACE_FL_FILTER	= (1 << 2),
	FTRACE_FL_ENABLED	= (1 << 3),
	FTRACE_FL_NOTRACE	= (1 << 4),
	FTRACE_FL_CONVERTED	= (1 << 5),
	FTRACE_FL_FROZEN	= (1 << 6),
};

struct dyn_ftrace {
	struct hlist_node node;
	unsigned long	  ip; /* address of mcount call-site */
	unsigned long	  flags;
};

int ftrace_force_update(void);
void ftrace_set_filter(unsigned char *buf, int len, int reset);

/* defined in arch */
extern int ftrace_ip_converted(unsigned long ip);
extern unsigned char *ftrace_nop_replace(void);
extern unsigned char *ftrace_call_replace(unsigned long ip, unsigned long addr);
extern int ftrace_dyn_arch_init(void *data);
extern int ftrace_mcount_set(unsigned long *data);
extern int ftrace_modify_code(unsigned long ip, unsigned char *old_code,
			      unsigned char *new_code);
extern int ftrace_update_ftrace_func(ftrace_func_t func);
extern void ftrace_caller(void);
extern void ftrace_call(void);
extern void mcount_call(void);

extern int skip_trace(unsigned long ip);

extern void ftrace_release(void *start, unsigned long size);

extern void ftrace_disable_daemon(void);
extern void ftrace_enable_daemon(void);

#else
# define skip_trace(ip)				({ 0; })
# define ftrace_force_update()			({ 0; })
# define ftrace_set_filter(buf, len, reset)	do { } while (0)
# define ftrace_disable_daemon()		do { } while (0)
# define ftrace_enable_daemon()			do { } while (0)
static inline void ftrace_release(void *start, unsigned long size) { }
#endif /* CONFIG_DYNAMIC_FTRACE */

static inline void tracer_disable(void)
{
#ifdef CONFIG_FTRACE
	ftrace_enabled = 0;
#endif
}

/*
 * Ftrace disable/restore without lock. Some synchronization mechanism
 * must be used to prevent ftrace_enabled to be changed between
 * disable/restore.
 */
static inline int __ftrace_enabled_save(void)
{
#ifdef CONFIG_FTRACE
	int saved_ftrace_enabled = ftrace_enabled;
	ftrace_enabled = 0;
	return saved_ftrace_enabled;
#else
	return 0;
#endif
}

static inline void __ftrace_enabled_restore(int enabled)
{
#ifdef CONFIG_FTRACE
	ftrace_enabled = enabled;
#endif
}

#ifdef CONFIG_FRAME_POINTER
/* TODO: need to fix this for ARM */
# define CALLER_ADDR0 ((unsigned long)__builtin_return_address(0))
# define CALLER_ADDR1 ((unsigned long)__builtin_return_address(1))
# define CALLER_ADDR2 ((unsigned long)__builtin_return_address(2))
# define CALLER_ADDR3 ((unsigned long)__builtin_return_address(3))
# define CALLER_ADDR4 ((unsigned long)__builtin_return_address(4))
# define CALLER_ADDR5 ((unsigned long)__builtin_return_address(5))
# define CALLER_ADDR6 ((unsigned long)__builtin_return_address(6))
#else
# define CALLER_ADDR0 ((unsigned long)__builtin_return_address(0))
# define CALLER_ADDR1 0UL
# define CALLER_ADDR2 0UL
# define CALLER_ADDR3 0UL
# define CALLER_ADDR4 0UL
# define CALLER_ADDR5 0UL
# define CALLER_ADDR6 0UL
#endif

#ifdef CONFIG_IRQSOFF_TRACER
  extern void time_hardirqs_on(unsigned long a0, unsigned long a1);
  extern void time_hardirqs_off(unsigned long a0, unsigned long a1);
#else
# define time_hardirqs_on(a0, a1)		do { } while (0)
# define time_hardirqs_off(a0, a1)		do { } while (0)
#endif

#ifdef CONFIG_PREEMPT_TRACER
  extern void trace_preempt_on(unsigned long a0, unsigned long a1);
  extern void trace_preempt_off(unsigned long a0, unsigned long a1);
#else
# define trace_preempt_on(a0, a1)		do { } while (0)
# define trace_preempt_off(a0, a1)		do { } while (0)
#endif

#ifdef CONFIG_TRACING
extern void
ftrace_special(unsigned long arg1, unsigned long arg2, unsigned long arg3);
void ftrace_halt(void);

/**
 * ftrace_printk - printf formatting in the ftrace buffer
 * @fmt: the printf format for printing
 *
 * Note: __ftrace_printk is an internal function for ftrace_printk and
 *       the @ip is passed in via the ftrace_printk macro.
 *
 * This function allows a kernel developer to debug fast path sections
 * that printk is not appropriate for. By scattering in various
 * printk like tracing in the code, a developer can quickly see
 * where problems are occurring.
 *
 * This is intended as a debugging tool for the developer only.
 * Please refrain from leaving ftrace_printks scattered around in
 * your code.
 */
# define ftrace_printk(fmt...) __ftrace_printk(_THIS_IP_, fmt)
extern int
__ftrace_printk(unsigned long ip, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));
extern void ftrace_dump(void);
#else
static inline void
ftrace_special(unsigned long arg1, unsigned long arg2, unsigned long arg3) { }
static inline void ftrace_halt(void) { }
static inline int
ftrace_printk(const char *fmt, ...) __attribute__ ((format (printf, 1, 0)));

static inline int
ftrace_printk(const char *fmt, ...)
{
	return 0;
}
static inline void ftrace_dump(void) { }
#endif

#ifdef CONFIG_FTRACE_MCOUNT_RECORD
extern void ftrace_init(void);
extern void ftrace_init_module(unsigned long *start, unsigned long *end);
#else
static inline void ftrace_init(void) { }
static inline void
ftrace_init_module(unsigned long *start, unsigned long *end) { }
#endif

struct hrtimer;

DEFINE_TRACE(event_irq,
	TPPROTO(int irq, int user, unsigned long ip),
		TPARGS(irq, user, ip));

DEFINE_TRACE(event_fault,
	TPPROTO(unsigned long ip, unsigned long error, unsigned long addr),
		TPARGS(ip, error, addr));

DEFINE_TRACE(event_timer_set,
	TPPROTO(ktime_t *expires, struct hrtimer *timer),
	     TPARGS(expires, timer));

DEFINE_TRACE(event_timer_triggered,
	TPPROTO(ktime_t *expires, struct hrtimer *timer),
	     TPARGS(expires, timer));

DEFINE_TRACE(event_timestamp,
	TPPROTO(ktime_t *time),
	     TPARGS(time));

DEFINE_TRACE(event_task_activate,
	TPPROTO(struct task_struct *p, int cpu),
	     TPARGS(p, cpu));

DEFINE_TRACE(event_task_deactivate,
	TPPROTO(struct task_struct *p, int cpu),
	     TPARGS(p, cpu));

DEFINE_TRACE(event_program_event,
	TPPROTO(ktime_t *expires, int64_t *delta),
	     TPARGS(expires, delta));

#endif /* _LINUX_FTRACE_H */
