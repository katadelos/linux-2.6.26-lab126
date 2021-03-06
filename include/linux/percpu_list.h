#ifndef _LINUX_PERCPU_LIST_H
#define _LINUX_PERCPU_LIST_H

#include <linux/lock_list.h>
#include <linux/percpu.h>

#ifdef CONFIG_SMP

struct percpu_list_element {
	spinlock_t lock;
	unsigned long nr;
	struct lock_list_head list;
};

struct percpu_list {
	struct lock_list_head list;
	struct percpu_list_element *percpu_list;
};

static inline
void percpu_list_init(struct percpu_list *pcl)
{
	int cpu;

	INIT_LOCK_LIST_HEAD(&pcl->list);
	pcl->percpu_list = alloc_percpu(struct percpu_list_element);

	for_each_possible_cpu(cpu) {
		struct percpu_list_element *pcle;

		pcle = per_cpu_ptr(pcl->percpu_list, cpu);
		spin_lock_init(&pcle->lock);
		pcle->nr = 0;
		INIT_LOCK_LIST_HEAD(&pcle->list);
	}
}

static inline
void percpu_list_destroy(struct percpu_list *pcl)
{
	free_percpu(pcl->percpu_list);
}

static inline
void percpu_list_fold_cpu(struct percpu_list *pcl, int cpu)
{
	struct percpu_list_element *pcle = per_cpu_ptr(pcl->percpu_list, cpu);

	spin_lock(&pcle->lock);
	if (pcle->nr) {
		pcle->nr = 0;
		lock_list_splice_init(&pcle->list, &pcl->list);
	}
	spin_unlock(&pcle->lock);
}

static inline
void percpu_list_add(struct percpu_list *pcl, struct lock_list_head *elm)
{
	struct percpu_list_element *pcle;
	int cpu = raw_smp_processor_id();
	unsigned long nr;

	pcle = per_cpu_ptr(pcl->percpu_list, cpu);
	spin_lock(&pcle->lock);
	nr = ++pcle->nr;
	lock_list_add(elm, &pcle->list);
	spin_unlock(&pcle->lock);

	if (nr >= 16)
		percpu_list_fold_cpu(pcl, cpu);
}

static inline
void percpu_list_fold(struct percpu_list *pcl)
{
	int cpu;

	for_each_possible_cpu(cpu)
		percpu_list_fold_cpu(pcl, cpu);
}

#else /* CONFIG_SMP */

struct percpu_list {
	struct lock_list_head list;
};

static inline
void percpu_list_init(struct percpu_list *pcl)
{
	INIT_LOCK_LIST_HEAD(&pcl->list);
}

static inline
void percpu_list_destroy(struct percpu_list *pcl)
{
}

static inline
void percpu_list_add(struct percpu_list *pcl, struct lock_list_head *elm)
{
	lock_list_add(elm, &pcl->list);
}

static inline
void percpu_list_fold(struct percpu_list *pcl)
{
}

#endif

static inline
struct lock_list_head *percpu_list_head(struct percpu_list *pcl)
{
	return &pcl->list;
}

#endif /* _LINUX_PERCPU_LIST_H */
