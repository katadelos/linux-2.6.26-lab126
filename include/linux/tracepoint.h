#ifndef _LINUX_TRACEPOINT_H
#define _LINUX_TRACEPOINT_H

/*
 * Kernel Tracepoint API.
 *
 * See Documentation/tracepoint.txt.
 *
 * (C) Copyright 2008 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * Heavily inspired from the Linux Kernel Markers.
 *
 * This file is released under the GPLv2.
 * See the file COPYING for more details.
 */

#include <linux/types.h>
#include <linux/rcupdate.h>

struct module;
struct tracepoint;

struct tracepoint {
	const char *name;		/* Tracepoint name */
	int state;			/* State. */
	void **funcs;
} __attribute__((aligned(8)));


#define TPPROTO(args...)	args
#define TPARGS(args...)		args

#ifdef CONFIG_TRACEPOINTS

/*
 * it_func[0] is never NULL because there is at least one element in the array
 * when the array itself is non NULL.
 */
#define __DO_TRACE(tp, proto, args)					\
	do {								\
		void **it_func;						\
									\
		preempt_disable();					\
		it_func = rcu_dereference((tp)->funcs);			\
		if (it_func) {						\
			do {						\
				((void(*)(proto))(*it_func))(args);	\
			} while (*(++it_func));				\
		}							\
		preempt_enable();					\
	} while (0)

/*
 * Make sure the alignment of the structure in the __tracepoints section will
 * not add unwanted padding between the beginning of the section and the
 * structure. Force alignment to the same alignment as the section start.
 */
#define DEFINE_TRACE(name, proto, args)					\
	static inline void trace_##name(proto)				\
	{								\
		static const char __tpstrtab_##name[]			\
		__attribute__((section("__tracepoints_strings")))	\
		= #name ":" #proto;					\
		static struct tracepoint __tracepoint_##name		\
		__attribute__((section("__tracepoints"), aligned(8))) =	\
		{ __tpstrtab_##name, 0, NULL };				\
		if (unlikely(__tracepoint_##name.state))		\
			__DO_TRACE(&__tracepoint_##name,		\
				TPPROTO(proto), TPARGS(args));		\
	}								\
	static inline int register_trace_##name(void (*probe)(proto))	\
	{								\
		return tracepoint_probe_register(#name ":" #proto,	\
			(void *)probe);					\
	}								\
	static inline void unregister_trace_##name(void (*probe)(proto))\
	{								\
		tracepoint_probe_unregister(#name ":" #proto,		\
			(void *)probe);					\
	}

extern void tracepoint_update_probe_range(struct tracepoint *begin,
	struct tracepoint *end);

#else /* !CONFIG_TRACEPOINTS */
#define DEFINE_TRACE(name, proto, args)			\
	static inline void _do_trace_##name(struct tracepoint *tp, proto) \
	{ }								\
	static inline void trace_##name(proto)				\
	{ }								\
	static inline int register_trace_##name(void (*probe)(proto))	\
	{								\
		return -ENOSYS;						\
	}								\
	static inline void unregister_trace_##name(void (*probe)(proto))\
	{ }

static inline void tracepoint_update_probe_range(struct tracepoint *begin,
	struct tracepoint *end)
{ }
#endif /* CONFIG_TRACEPOINTS */

/*
 * Connect a probe to a tracepoint.
 * Internal API, should not be used directly.
 */
extern int tracepoint_probe_register(const char *name, void *probe);

/*
 * Disconnect a probe from a tracepoint.
 * Internal API, should not be used directly.
 */
extern int tracepoint_probe_unregister(const char *name, void *probe);

struct tracepoint_iter {
	struct module *module;
	struct tracepoint *tracepoint;
};

extern void tracepoint_iter_start(struct tracepoint_iter *iter);
extern void tracepoint_iter_next(struct tracepoint_iter *iter);
extern void tracepoint_iter_stop(struct tracepoint_iter *iter);
extern void tracepoint_iter_reset(struct tracepoint_iter *iter);
extern int tracepoint_get_iter_range(struct tracepoint **tracepoint,
	struct tracepoint *begin, struct tracepoint *end);

#endif
