/*
 * Quicklist support.
 *
 * Quicklists are light weight lists of pages that have a defined state
 * on alloc and free. Pages must be in the quicklist specific defined state
 * (zero by default) when the page is freed. It seems that the initial idea
 * for such lists first came from Dave Miller and then various other people
 * improved on it.
 *
 * Copyright (C) 2007 SGI,
 * 	Christoph Lameter <clameter@sgi.com>
 * 		Generalized, added support for multiple lists and
 * 		constructors / destructors.
 */
#include <linux/kernel.h>

#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/module.h>
#include <linux/quicklist.h>

DEFINE_PER_CPU_LOCKED(struct quicklist, quicklist)[CONFIG_NR_QUICK];

#define FRACTION_OF_NODE_MEM	16

static unsigned long max_pages(unsigned long min_pages)
{
	unsigned long node_free_pages, max;
	struct zone *zones = NODE_DATA(numa_node_id())->node_zones;

	node_free_pages =
#ifdef CONFIG_ZONE_DMA
		zone_page_state(&zones[ZONE_DMA], NR_FREE_PAGES) +
#endif
#ifdef CONFIG_ZONE_DMA32
		zone_page_state(&zones[ZONE_DMA32], NR_FREE_PAGES) +
#endif
		zone_page_state(&zones[ZONE_NORMAL], NR_FREE_PAGES);

	max = node_free_pages / FRACTION_OF_NODE_MEM;
	return max(max, min_pages);
}

static long min_pages_to_free(struct quicklist *q,
	unsigned long min_pages, long max_free)
{
	long pages_to_free;

	pages_to_free = q->nr_pages - max_pages(min_pages);

	return min(pages_to_free, max_free);
}

/*
 * Trim down the number of pages in the quicklist
 */
void quicklist_trim(int nr, void (*dtor)(void *),
	unsigned long min_pages, unsigned long max_free)
{
	long pages_to_free;
	struct quicklist *q;
	int cpu;

	q = &get_cpu_var_locked(quicklist, &cpu)[nr];
	if (q->nr_pages > min_pages) {
		pages_to_free = min_pages_to_free(q, min_pages, max_free);

		while (pages_to_free > 0) {
			void *p = __quicklist_alloc(q);

			if (dtor)
				dtor(p);
			free_page((unsigned long)p);
			pages_to_free--;
		}
	}
	put_cpu_var_locked(quicklist, cpu);
}

unsigned long quicklist_total_size(void)
{
	unsigned long count = 0;
	int cpu;
	struct quicklist *ql, *q;

	for_each_online_cpu(cpu) {
		ql = per_cpu_var_locked(quicklist, cpu);
		for (q = ql; q < ql + CONFIG_NR_QUICK; q++)
			count += q->nr_pages;
	}
	return count;
}
