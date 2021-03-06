/* Include in trace.c */

#include <linux/kthread.h>
#include <linux/delay.h>

static inline int trace_valid_entry(struct trace_entry *entry)
{
	switch (entry->type) {
	case TRACE_FN:
	case TRACE_CTX:
	case TRACE_WAKE:
	case TRACE_CONT:
	case TRACE_STACK:
	case TRACE_PRINT:
	case TRACE_SPECIAL:
	case TRACE_IRQ:
	case TRACE_FAULT:
	case TRACE_TIMER_SET:
	case TRACE_TIMER_TRIG:
	case TRACE_TIMESTAMP:
	case TRACE_TASK_ACT:
	case TRACE_TASK_DEACT:
	case TRACE_SYSCALL:
	case TRACE_SYSRET:
		return 1;
	}
	return 0;
}

static int
trace_test_buffer_cpu(struct trace_array *tr, struct trace_array_cpu *data)
{
	struct trace_entry *entries;
	struct page *page;
	int idx = 0;
	int i;

	BUG_ON(list_empty(&data->trace_pages));
	page = list_entry(data->trace_pages.next, struct page, lru);
	entries = page_address(page);

	check_pages(data);
	if (head_page(data) != entries)
		goto failed;

	/*
	 * The starting trace buffer always has valid elements,
	 * if any element exists.
	 */
	entries = head_page(data);

	for (i = 0; i < tr->entries; i++) {

		if (i < data->trace_idx && !trace_valid_entry(&entries[idx])) {
			printk(KERN_CONT ".. invalid entry %d ",
				entries[idx].type);
			goto failed;
		}

		idx++;
		if (idx >= ENTRIES_PER_PAGE) {
			page = virt_to_page(entries);
			if (page->lru.next == &data->trace_pages) {
				if (i != tr->entries - 1) {
					printk(KERN_CONT ".. entries buffer mismatch");
					goto failed;
				}
			} else {
				page = list_entry(page->lru.next, struct page, lru);
				entries = page_address(page);
			}
			idx = 0;
		}
	}

	page = virt_to_page(entries);
	if (page->lru.next != &data->trace_pages) {
		printk(KERN_CONT ".. too many entries");
		goto failed;
	}

	return 0;

 failed:
	/* disable tracing */
	tracing_disabled = 1;
	printk(KERN_CONT ".. corrupted trace buffer .. ");
	return -1;
}

/*
 * Test the trace buffer to see if all the elements
 * are still sane.
 */
static int trace_test_buffer(struct trace_array *tr, unsigned long *count)
{
	unsigned long flags, cnt = 0;
	int cpu, ret = 0;

	/* Don't allow flipping of max traces now */
	raw_local_irq_save(flags);
	__raw_spin_lock(&ftrace_max_lock);
	for_each_possible_cpu(cpu) {
		if (!head_page(tr->data[cpu]))
			continue;

		cnt += tr->data[cpu]->trace_idx;

		ret = trace_test_buffer_cpu(tr, tr->data[cpu]);
		if (ret)
			break;
	}
	__raw_spin_unlock(&ftrace_max_lock);
	raw_local_irq_restore(flags);

	if (count)
		*count = cnt;

	return ret;
}

#ifdef CONFIG_FTRACE

#ifdef CONFIG_DYNAMIC_FTRACE

#define __STR(x) #x
#define STR(x) __STR(x)

/* Test dynamic code modification and ftrace filters */
int trace_selftest_startup_dynamic_tracing(struct tracer *trace,
					   struct trace_array *tr,
					   int (*func)(void))
{
	int save_ftrace_enabled = ftrace_enabled;
	int save_tracer_enabled = tracer_enabled;
	unsigned long count;
	char *func_name;
	int ret;

	/* The ftrace test PASSED */
	printk(KERN_CONT "PASSED\n");
	pr_info("Testing dynamic ftrace: ");

	/* enable tracing, and record the filter function */
	ftrace_enabled = 1;
	tracer_enabled = 1;

	/* passed in by parameter to fool gcc from optimizing */
	func();

	/* update the records */
	ret = ftrace_force_update();
	if (ret) {
		printk(KERN_CONT ".. ftraced failed .. ");
		return ret;
	}

	/*
	 * Some archs *cough*PowerPC*cough* add charachters to the
	 * start of the function names. We simply put a '*' to
	 * accomodate them.
	 */
	func_name = "*" STR(DYN_FTRACE_TEST_NAME);

	/* filter only on our function */
	ftrace_set_filter(func_name, strlen(func_name), 1);

	/* enable tracing */
	tr->ctrl = 1;
	trace->init(tr);

	/* Sleep for a 1/10 of a second */
	msleep(100);

	/* we should have nothing in the buffer */
	ret = trace_test_buffer(tr, &count);
	if (ret)
		goto out;

	if (count) {
		ret = -1;
		printk(KERN_CONT ".. filter did not filter .. ");
		goto out;
	}

	/* call our function again */
	func();

	/* sleep again */
	msleep(100);

	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	ftrace_enabled = 0;

	/* check the trace buffer */
	ret = trace_test_buffer(tr, &count);
	trace->reset(tr);

	/* we should only have one item */
	if (!ret && count != 1) {
		printk(KERN_CONT ".. filter failed count=%ld ..", count);
		ret = -1;
		goto out;
	}
 out:
	ftrace_enabled = save_ftrace_enabled;
	tracer_enabled = save_tracer_enabled;

	/* Enable tracing on all functions again */
	ftrace_set_filter(NULL, 0, 1);

	return ret;
}
#else
# define trace_selftest_startup_dynamic_tracing(trace, tr, func) ({ 0; })
#endif /* CONFIG_DYNAMIC_FTRACE */
/*
 * Simple verification test of ftrace function tracer.
 * Enable ftrace, sleep 1/10 second, and then read the trace
 * buffer to see if all is in order.
 */
int
trace_selftest_startup_function(struct tracer *trace, struct trace_array *tr)
{
	int save_ftrace_enabled = ftrace_enabled;
	int save_tracer_enabled = tracer_enabled;
	unsigned long count;
	int ret;

	/* make sure msleep has been recorded */
	msleep(1);

	/* force the recorded functions to be traced */
	ret = ftrace_force_update();
	if (ret) {
		printk(KERN_CONT ".. ftraced failed .. ");
		return ret;
	}

	/* start the tracing */
	ftrace_enabled = 1;
	tracer_enabled = 1;

	tr->ctrl = 1;
	trace->init(tr);
	/* Sleep for a 1/10 of a second */
	msleep(100);
	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	ftrace_enabled = 0;

	/* check the trace buffer */
	ret = trace_test_buffer(tr, &count);
	trace->reset(tr);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
		goto out;
	}

	ret = trace_selftest_startup_dynamic_tracing(trace, tr,
						     DYN_FTRACE_TEST_NAME);

 out:
	ftrace_enabled = save_ftrace_enabled;
	tracer_enabled = save_tracer_enabled;

	/* kill ftrace totally if we failed */
	if (ret)
		ftrace_kill();

	return ret;
}
#endif /* CONFIG_FTRACE */

#ifdef CONFIG_IRQSOFF_TRACER
int
trace_selftest_startup_irqsoff(struct tracer *trace, struct trace_array *tr)
{
	unsigned long save_max = tracing_max_latency;
	unsigned long count;
	int ret;

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);
	/* reset the max latency */
	tracing_max_latency = 0;
	/* disable interrupts for a bit */
	local_irq_disable();
	udelay(100);
	local_irq_enable();
	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check both trace buffers */
	ret = trace_test_buffer(tr, NULL);
	if (!ret)
		ret = trace_test_buffer(&max_tr, &count);
	trace->reset(tr);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
	}

	tracing_max_latency = save_max;

	return ret;
}
#endif /* CONFIG_IRQSOFF_TRACER */

#ifdef CONFIG_PREEMPT_TRACER
int
trace_selftest_startup_preemptoff(struct tracer *trace, struct trace_array *tr)
{
	unsigned long save_max = tracing_max_latency;
	unsigned long count;
	int ret;

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);
	/* reset the max latency */
	tracing_max_latency = 0;
	/* disable preemption for a bit */
	preempt_disable();
	udelay(100);
	preempt_enable();
	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check both trace buffers */
	ret = trace_test_buffer(tr, NULL);
	if (!ret)
		ret = trace_test_buffer(&max_tr, &count);
	trace->reset(tr);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
	}

	tracing_max_latency = save_max;

	return ret;
}
#endif /* CONFIG_PREEMPT_TRACER */

#if defined(CONFIG_IRQSOFF_TRACER) && defined(CONFIG_PREEMPT_TRACER)
int
trace_selftest_startup_preemptirqsoff(struct tracer *trace, struct trace_array *tr)
{
	unsigned long save_max = tracing_max_latency;
	unsigned long count;
	int ret;

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);

	/* reset the max latency */
	tracing_max_latency = 0;

	/* disable preemption and interrupts for a bit */
	preempt_disable();
	local_irq_disable();
	udelay(100);
	preempt_enable();
	/* reverse the order of preempt vs irqs */
	local_irq_enable();

	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check both trace buffers */
	ret = trace_test_buffer(tr, NULL);
	if (ret)
		goto out;

	ret = trace_test_buffer(&max_tr, &count);
	if (ret)
		goto out;

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
		goto out;
	}

	/* do the test by disabling interrupts first this time */
	tracing_max_latency = 0;
	tr->ctrl = 1;
	trace->ctrl_update(tr);
	preempt_disable();
	local_irq_disable();
	udelay(100);
	preempt_enable();
	/* reverse the order of preempt vs irqs */
	local_irq_enable();

	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check both trace buffers */
	ret = trace_test_buffer(tr, NULL);
	if (ret)
		goto out;

	ret = trace_test_buffer(&max_tr, &count);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
		goto out;
	}

 out:
	trace->reset(tr);
	tracing_max_latency = save_max;

	return ret;
}
#endif /* CONFIG_IRQSOFF_TRACER && CONFIG_PREEMPT_TRACER */

#ifdef CONFIG_SCHED_TRACER
static int trace_wakeup_test_thread(void *data)
{
	/* Make this a RT thread, doesn't need to be too high */
	struct sched_param param = { .sched_priority = 5 };
	struct completion *x = data;

	sched_setscheduler(current, SCHED_FIFO, &param);

	/* Make it know we have a new prio */
	complete(x);

	/* now go to sleep and let the test wake us up */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule();

	/* we are awake, now wait to disappear */
	while (!kthread_should_stop()) {
		/*
		 * This is an RT task, do short sleeps to let
		 * others run.
		 */
		msleep(100);
	}

	return 0;
}

int
trace_selftest_startup_wakeup(struct tracer *trace, struct trace_array *tr)
{
	unsigned long save_max = tracing_max_latency;
	struct task_struct *p;
	struct completion isrt;
	unsigned long count;
	int ret;

	init_completion(&isrt);

	/* create a high prio thread */
	p = kthread_run(trace_wakeup_test_thread, &isrt, "ftrace-test");
	if (IS_ERR(p)) {
		printk(KERN_CONT "Failed to create ftrace wakeup test thread ");
		return -1;
	}

	/* make sure the thread is running at an RT prio */
	wait_for_completion(&isrt);

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);
	/* reset the max latency */
	tracing_max_latency = 0;

	/* sleep to let the RT thread sleep too */
	msleep(100);

	/*
	 * Yes this is slightly racy. It is possible that for some
	 * strange reason that the RT thread we created, did not
	 * call schedule for 100ms after doing the completion,
	 * and we do a wakeup on a task that already is awake.
	 * But that is extremely unlikely, and the worst thing that
	 * happens in such a case, is that we disable tracing.
	 * Honestly, if this race does happen something is horrible
	 * wrong with the system.
	 */

	wake_up_process(p);

	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check both trace buffers */
	ret = trace_test_buffer(tr, NULL);
	if (!ret)
		ret = trace_test_buffer(&max_tr, &count);


	trace->reset(tr);

	tracing_max_latency = save_max;

	/* kill the thread */
	kthread_stop(p);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
	}

	return ret;
}
#endif /* CONFIG_SCHED_TRACER */

#ifdef CONFIG_CONTEXT_SWITCH_TRACER
int
trace_selftest_startup_sched_switch(struct tracer *trace, struct trace_array *tr)
{
	unsigned long count;
	int ret;

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);
	/* Sleep for a 1/10 of a second */
	msleep(100);
	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check the trace buffer */
	ret = trace_test_buffer(tr, &count);
	trace->reset(tr);

	if (!ret && !count) {
		printk(KERN_CONT ".. no entries found ..");
		ret = -1;
	}

	return ret;
}
#endif /* CONFIG_CONTEXT_SWITCH_TRACER */

#ifdef CONFIG_SYSPROF_TRACER
int
trace_selftest_startup_sysprof(struct tracer *trace, struct trace_array *tr)
{
	unsigned long count;
	int ret;

	/* start the tracing */
	tr->ctrl = 1;
	trace->init(tr);
	/* Sleep for a 1/10 of a second */
	msleep(100);
	/* stop the tracing. */
	tr->ctrl = 0;
	trace->ctrl_update(tr);
	/* check the trace buffer */
	ret = trace_test_buffer(tr, &count);
	trace->reset(tr);

	return ret;
}
#endif /* CONFIG_SYSPROF_TRACER */
