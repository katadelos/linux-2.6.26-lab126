#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/smp_lock.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/kallsyms.h>

#include "rtmutex_common.h"

#ifdef CONFIG_LOGDEV
#include <linux/logdev.h>
#define LD_WARN_ON_ONCE(cond)				\
	do {						\
		static int once;			\
		if (unlikely(cond) && !once++) {	\
			lfcnprint("FAILED " #cond);	\
			logdev_print_off();		\
			oops_in_progress++; \
			logdev_dump();			\
			WARN_ON(1);			\
			oops_in_progress--; \
		}					\
	} while (0)
#else
#define LD_WARN_ON_ONCE(cond) WARN_ON_ONCE(cond)
#define lfcnprint(x...) do { } while (0)
#define lmark() do { } while(0)
#define logdev_dump() do { } while (0)
#endif

static DEFINE_RWLOCK(lock1);
static DEFINE_RWLOCK(lock2);
static DEFINE_RWLOCK(lock3);

static DECLARE_RWSEM(sem1);
static DECLARE_RWSEM(sem2);
static DECLARE_RWSEM(sem3);

static DEFINE_MUTEX(mutex1);
static DEFINE_MUTEX(mutex2);
static DEFINE_MUTEX(mutex3);

static DEFINE_SPINLOCK(reverse_lock);

struct locks {
	union {
		struct rw_semaphore *sem;
		rwlock_t *lock;
		struct mutex *mutex;
	};

	int type;
	char *name;

	/* to test unnested locks */
	struct locks *reverse_lock;
	int reverse_read;
	struct task_struct *who;

	/* stats */
	int read_cnt;
	int write_cnt;
	int downgrade;
	int taken;
	int retaken;
	int reversed;
};

enum { LOCK_TYPE_LOCK = 0,
       LOCK_TYPE_SEM = 1,
       LOCK_TYPE_MUTEX = 2
};

static struct locks test_lock1 = {
	{
		.lock = &lock1,
	},
	.type = LOCK_TYPE_LOCK,
	.name = "lock1",
};

static struct locks test_lock2 = {
	{
		.lock = &lock2,
	},
	.type = LOCK_TYPE_LOCK,
	.name = "lock2",
};

static struct locks test_lock3 = {
	{
		.lock = &lock3,
	},
	.type = LOCK_TYPE_LOCK,
	.name = "lock3",
};

static struct locks test_sem1 = {
	{
		.sem = &sem1,
	},
	.type = LOCK_TYPE_SEM,
	.name = "sem1",
};

static struct locks test_sem2 = {
	{
		.sem = &sem2,
	},
	.type = LOCK_TYPE_SEM,
	.name = "sem2",
};

static struct locks test_sem3 = {
	{
		.sem = &sem3,
	},
	.type = LOCK_TYPE_SEM,
	.name = "sem3",
};

static struct locks test_mutex1 = {
	{
		.mutex = &mutex1,
	},
	.type = LOCK_TYPE_MUTEX,
	.name = "mutex1",
};

static struct locks test_mutex2 = {
	{
		.mutex = &mutex2,
	},
	.type = LOCK_TYPE_MUTEX,
	.name = "mutex2",
};

static struct locks test_mutex3 = {
	{
		.mutex = &mutex3,
	},
	.type = LOCK_TYPE_MUTEX,
	.name = "mutex3",
};

static int test_done;

#define TIME_MAX 20000

#define DEFAULT_NR_THREADS 300
#define DEFAULT_NR_RT_THREADS 10

/* times in usecs */
#define DEFAULT_SCHED_OTHER_TIME_US	1000
#define DEFAULT_SCHED_FIFO_TIME_US	200

/* this is in millisecs */
#define DEFAULT_SCHED_FIFO_SLEEP_TIME	2
#define DEFAULT_SCHED_OTHER_SLEEP_TIME	1

#define DEFAULT_RT_THREAD_PRIO 40

#define NR_TESTS 3
static unsigned long sched_other_time_usecs = DEFAULT_SCHED_OTHER_TIME_US;
static unsigned long sched_fifo_time_usecs = DEFAULT_SCHED_FIFO_TIME_US;
static unsigned int sched_other_sleep_ms = DEFAULT_SCHED_OTHER_SLEEP_TIME;
static unsigned int sched_fifo_sleep_ms = DEFAULT_SCHED_FIFO_SLEEP_TIME;

static unsigned long rt_thread_prio = DEFAULT_RT_THREAD_PRIO;
static unsigned int thread_count = DEFAULT_NR_THREADS;
static unsigned int rt_thread_count = DEFAULT_NR_RT_THREADS;
static int test_time = 30;
static struct task_struct **tsks;

static int perform_downgrade_write = 0;

enum {
	LOCK_READ = 0,
	LOCK_WRITE = 1,
	SEM_READ = 2,
	SEM_WRITE = 3,
	MUTEX = 5	/* must be odd */
};

#ifdef CONFIG_PREEMPT_RT
static void show_rtm_owner(char *str, struct rt_mutex *rtm)
{
	struct task_struct *owner;
	unsigned long val;
	char *name;

	rcu_read_lock();
	val = (unsigned long)rtm->owner;
	owner = (struct task_struct *)(val & ~3UL);
	name = "NULL";
	if (owner) {
		if (owner == (struct task_struct *)0x100)
			name = "READER";
		else
			name = owner->comm;
	}
	printk("%s val: %lx  owner: %s\n", str, val, name);

	rcu_read_unlock();
}

static void show_mutex_owner(char *str, struct mutex *mutex)
{
	show_rtm_owner(str, &mutex->lock);
}

static void show_rwm_owner(char *str, struct rw_mutex *rwm)
{
	struct reader_lock_struct *rls;
	struct task_struct *owner;
	unsigned long val;
	char *name;

	rcu_read_lock();
	val = (unsigned long)rwm->owner;
	owner = (struct task_struct *)(val & ~3UL);
	name = "NULL";
	if (owner) {
		switch ((unsigned long)owner) {
		case 0x100:
			name = "READER";
			break;
		case 0x200:
			name = "PENDING READER";
			break;
		case 0x400:
			name = "PENDING WRITER";
			break;
		default:
			name = owner->comm;
		}
	}
	printk("%s val: %lx  owner: %s count %d owners %d ", str, val, name,
	       atomic_read(&rwm->count),
	       atomic_read(&rwm->owners));
	show_rtm_owner("  mutex: ", &rwm->mutex);
	list_for_each_entry(rls, &rwm->readers, list) {
		if (!rls->task)
			printk("NULL TASK!!!\n");
		else
			printk("   owned by: %s:%d\n",
			       rls->task->comm, rls->task->pid);
	}
	rcu_read_unlock();
}

static void show_rwlock_owner(char *str, rwlock_t *lock)
{
	show_rwm_owner(str, &lock->owners);
}

static void show_sem_owner(char *str, struct rw_semaphore *sem)
{
	show_rwm_owner(str, &sem->owners);
}

void print_owned_read_locks(struct task_struct *tsk)
{
	int i;

	if (!tsk->reader_lock_count)
		return;

	oops_in_progress++;
	printk(" %s:%d owns:\n", tsk->comm, tsk->pid);
	for (i = 0; i < tsk->reader_lock_count; i++) {
		printk("    %p\n", tsk->owned_read_locks[i].lock);
	}
	oops_in_progress--;
}

#else
# define show_sem_owner(x...)		do { } while (0)
# define show_rwlock_owner(x...)	do { } while (0)
# define show_mutex_owner(x...)		do { } while (0)
#endif

static int do_read(int read)
{
	unsigned long x;
	int ret;

	x = random32();

	/* rwlock can not schedule */
	if (!(read & ~1)) {
		ret = LOCK_READ;
		goto out;
	}

	/* every other time pick a mutex */
	if (x & 0x1000)
		return MUTEX; /* do mutex */

	/* alternate between locks and semaphores */
	if (x & 0x10)
		ret = LOCK_READ;
	else
		ret = SEM_READ;

 out:
	/* Do write 1 in 16 times */
	return ret | !(x & 0xf);
}

static struct locks *
pick_lock(struct locks *lock, struct locks *sem, struct locks *mutex, int read)
{
	switch (read) {
	case LOCK_READ:
	case LOCK_WRITE:
		return lock;
	case SEM_READ:
	case SEM_WRITE:
		return sem;
	case MUTEX:
		return mutex;
	}
	return NULL;
}

static void do_lock(struct locks *lock, int read)
{
	lfcnprint("reader_lock_count=%d", current->reader_lock_count);
	switch (read) {
	case LOCK_READ:
		if (unlikely(lock->type != LOCK_TYPE_LOCK)) {
			printk("FAILED expected lock but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("read lock %s %p count=%d owners=%d",
			  lock->name, lock, atomic_read(&lock->lock->owners.count),
			  atomic_read(&lock->lock->owners.owners));
		read_lock(lock->lock);
		break;
	case LOCK_WRITE:
		if (unlikely(lock->type != LOCK_TYPE_LOCK)) {
			printk("FAILED expected lock but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("write lock %s %p", lock->name, lock);
		write_lock(lock->lock);
		break;
	case SEM_READ:
		if (unlikely(lock->type != LOCK_TYPE_SEM)) {
			printk("FAILED expected sem but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("read sem %s %p count=%d owners=%d",
			  lock->name, lock,
			  atomic_read(&lock->sem->owners.count),
			  atomic_read(&lock->sem->owners.owners));
		down_read(lock->sem);
		break;
	case SEM_WRITE:
		if (unlikely(lock->type != LOCK_TYPE_SEM)) {
			printk("FAILED expected sem but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("write sem %s %p", lock->name, lock);
		down_write(lock->sem);
		break;
	case MUTEX:
		if (unlikely(lock->type != LOCK_TYPE_MUTEX)) {
			printk("FAILED expected mutex but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("mutex %s %p", lock->name, lock);
		mutex_lock(lock->mutex);
		break;
	default:
		printk("bad lock value %d!!!\n", read);
	}
	lfcnprint("taken %s %p", lock->name, lock);
}

static void do_unlock(struct locks *lock, int read, struct locks *prev_lock)
{
	if (prev_lock) {
		spin_lock(&reverse_lock);
		if (!prev_lock->reverse_lock) {
			int x;
			/* test reverse order unlocking */
			x = random32();
			if (x & 1) {
				lfcnprint("reverse lock %s %p and %s %p",
					  lock->name, lock,
					  prev_lock->name, prev_lock);
				prev_lock->reverse_lock = lock;
				prev_lock->reverse_read = read;
				prev_lock->who = current;
				lock->reversed++;
				spin_unlock(&reverse_lock);
				return;
			}
		}
		spin_unlock(&reverse_lock);
	}

	switch (read) {
	case LOCK_READ:
		if (unlikely(lock->type != LOCK_TYPE_LOCK)) {
			printk("FAILED expected lock but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("read lock %s %p count=%d owners=%d",
			  lock->name, lock, atomic_read(&lock->lock->owners.count),
			  atomic_read(&lock->lock->owners.owners));
		read_unlock(lock->lock);
		break;
	case LOCK_WRITE:
		if (unlikely(lock->type != LOCK_TYPE_LOCK)) {
			printk("FAILED expected lock but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("write lock %s %p", lock->name, lock);
		write_unlock(lock->lock);
		break;
	case SEM_READ:
		if (unlikely(lock->type != LOCK_TYPE_SEM)) {
			printk("FAILED expected sem but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("read sem %s, %p count=%d owners=%d",
			  lock->name, lock, atomic_read(&lock->sem->owners.count),
			  atomic_read(&lock->sem->owners.owners));
		up_read(lock->sem);
		break;
	case SEM_WRITE:
		if (unlikely(lock->type != LOCK_TYPE_SEM)) {
			printk("FAILED expected sem but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("write sem %s %p", lock->name, lock);
		up_write(lock->sem);
		break;
	case MUTEX:
		if (unlikely(lock->type != LOCK_TYPE_MUTEX)) {
			printk("FAILED expected mutex but got %d\n",
			       lock->type);
			return;
		}
		lfcnprint("mutex %s %p", lock->name, lock);
		mutex_unlock(lock->mutex);
		break;
	default:
		printk("bad lock value %d!!!\n", read);
	}
	lfcnprint("%s unlocked", lock->name);

	if (lock->reverse_lock && lock->who == current) {
		lock->who = NULL;
		lfcnprint("unlock reverse lock %s %p",
			  lock->reverse_lock->name, lock->reverse_lock);
		do_unlock(lock->reverse_lock, lock->reverse_read, NULL);
		lock->reverse_lock = NULL;
	}
}

static void do_something(unsigned long time, int ignore, struct locks *prev_lock)
{
	lmark();
	if (test_done)
		return;
	if (time > TIME_MAX)
		time = TIME_MAX;
	udelay(time);
}

static void do_downgrade(unsigned long time, struct locks *lock, int *read)
{
	struct rw_semaphore *sem = lock->sem;
	unsigned long x;

	if (!perform_downgrade_write)
		return;

	if (test_done)
		return;

	if (*read == SEM_WRITE) {
		x = random32();

		/* Do downgrade write 1 in 16 times of a write */
		if (!(x & 0xf)) {
			lfcnprint("downgrade %p", sem);
			lock->downgrade++;
			downgrade_write(sem);
			do_something(time, 0, NULL);
			/* need to do unlock read */
			*read = SEM_READ;
		}
	}
}

static void update_stats(int read, struct locks *lock)
{
	switch (read) {
	case LOCK_READ:
	case SEM_READ:
		lock->read_cnt++;
		break;
	case LOCK_WRITE:
	case SEM_WRITE:
		lock->write_cnt++;
		break;
	}
	lock->taken++;
}

#define MAX_DEPTH 10

static void run_lock(void (*func)(unsigned long time, int read, struct locks *prev_lock),
		     struct locks *lock, unsigned long time, int read, int depth,
		     struct locks *prev_lock);

static void do_again(void (*func)(unsigned long time, int read, struct locks *prev_lock),
		     struct locks *lock, unsigned long time, int read, int depth)
{
	unsigned long x;

	if (test_done)
		return;

	/*
	 * If this was grabbed for read via rwlock, do again
	 * (but not if we did a reverse)
	 */
	if (likely(read != LOCK_READ) || depth >= MAX_DEPTH || lock->reverse_lock)
		return;

	x = random32();
	if (x & 1) {
		lfcnprint("read lock again");
		run_lock(func, lock, time, read, depth+1, NULL);
	}
}

static void run_lock(void (*func)(unsigned long time, int read, struct locks *prev_lock),
		     struct locks *lock, unsigned long time, int read, int depth,
		     struct locks *prev_lock)
{
	if (test_done)
		return;

	update_stats(read, lock);
	if (depth)
		lock->retaken++;
	do_lock(lock, read);
	if (!test_done) {
		func(time, do_read(read), lock);
		do_again(func, lock, time, read, depth);
	}
	do_downgrade(time, lock, &read);
	do_unlock(lock, read, prev_lock);

}

static void run_one_lock(unsigned long time, int read, struct locks *prev_lock)
{
	struct locks *lock;

	lmark();
	lock = pick_lock(&test_lock1, &test_sem1, &test_mutex1, read);
	run_lock(do_something, lock, time, read, 0, prev_lock);
}

static void run_two_locks(unsigned long time, int read, struct locks *prev_lock)
{
	struct locks *lock;

	lmark();
	lock = pick_lock(&test_lock2, &test_sem2, &test_mutex2, read);
	run_lock(run_one_lock, lock, time, read, 0, prev_lock);
}

static void run_three_locks(unsigned long time, int read, struct locks *prev_lock)
{
	struct locks *lock;

	lmark();
	lock = pick_lock(&test_lock3, &test_sem3, &test_mutex3, read);
	run_lock(run_two_locks, lock, time, read, 0, prev_lock);
}

static int run_test(unsigned long time)
{
	unsigned long long start;
	int read;
	int ret;

	if (test_done)
		return 0;

	start = random32();

	read = do_read(MUTEX);

	switch (ret = (start & 3)) {
	case 0:
		run_one_lock(time, read, NULL);
		break;
	case 1:
		run_two_locks(time, read, NULL);
		break;
	case 2:
		run_three_locks(time, read, NULL);
		break;
	default:
		ret = 1;
		run_two_locks(time, read, NULL);
	}

	LD_WARN_ON_ONCE(current->reader_lock_count);

	return ret;
}

static int rwlock_thread(void *arg)
{
	long prio = (long)arg;
	unsigned long time;
	unsigned long run;
	struct sched_param param;

	time = sched_fifo_time_usecs;
	if (prio) {
		param.sched_priority = prio;
		sched_setscheduler(current, SCHED_FIFO, &param);
		time = sched_fifo_time_usecs;
	}

	while (!kthread_should_stop()) {
		run = run_test(time);

		if (prio)
			msleep(sched_fifo_sleep_ms);
		else
			msleep(sched_other_sleep_ms);
	}

	return 0;
}

static void print_lock_stat(struct locks *lock)
{
	switch (lock->type) {
	case LOCK_TYPE_LOCK:
	case LOCK_TYPE_SEM:
		printk("%8s taken for read: %9d\n", lock->name, lock->read_cnt);
		printk("%8s taken for write: %8d\n", lock->name, lock->write_cnt);
		if (lock->type == LOCK_TYPE_LOCK) {
			printk("%8s retaken:        %9d\n",
			       lock->name, lock->retaken);
		} else if (perform_downgrade_write) {
			printk("%8s downgraded:     %9d\n",
			       lock->name, lock->downgrade);
		}
	}
	printk("%8s taken:           %8d\n", lock->name, lock->taken);
	printk("%8s reversed:        %8d\n\n", lock->name, lock->reversed);
}

static int __init mutex_stress_init(void)
{
	long i;

	tsks = kmalloc(sizeof(*tsks) * (thread_count + rt_thread_count), GFP_KERNEL);
	if (!tsks) {
		printk("failed to allocate tasks\n");
		return -1;
	}

	printk("create threads and run for %d seconds\n", test_time);

	for (i=0; i < thread_count; i++)
		tsks[i] = kthread_run(rwlock_thread, NULL, "mtest%d", i);
	for (i=0; i < rt_thread_count; i++) {
		long prio = rt_thread_prio + i;
		tsks[thread_count + i] =
			kthread_run(rwlock_thread, (void*)prio,
				    "mtest%d", thread_count + i);
	}


	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(test_time * HZ);

	printk("kill threads\n");
	test_done = 1;

	set_current_state(TASK_INTERRUPTIBLE);
	/* sleep some to allow all tasks to finish */
	schedule_timeout(3 * HZ);

	lfcnprint("Done");

	show_rwlock_owner("lock1: ", &lock1);
	show_rwlock_owner("lock2: ", &lock2);
	show_rwlock_owner("lock3: ", &lock3);

	show_sem_owner("sem1: ", &sem1);
	show_sem_owner("sem2: ", &sem2);
	show_sem_owner("sem3: ", &sem3);

	show_mutex_owner("mutex1: ", &mutex1);
	show_mutex_owner("mutex2: ", &mutex2);
	show_mutex_owner("mutex3: ", &mutex3);

	oops_in_progress++;
//	logdev_dump();
	oops_in_progress--;

#ifdef CONFIG_PREEMPT_RT
	for (i=0; i < (thread_count + rt_thread_count); i++) {
		if (tsks[i]) {
			struct rt_mutex *mtx;
			unsigned long own;
			struct rt_my_waiter {
				struct plist_node list_entry;
				struct plist_node pi_list_entry;
				struct task_struct *task;
				struct rt_mutex *lock;
			} *w;

			spin_lock_irq(&tsks[i]->pi_lock);

			print_owned_read_locks(tsks[i]);

			if (tsks[i]->pi_blocked_on) {
				w = (void *)tsks[i]->pi_blocked_on;
				mtx = w->lock;
				spin_unlock_irq(&tsks[i]->pi_lock);
				spin_lock_irq(&mtx->wait_lock);
				spin_lock(&tsks[i]->pi_lock);
				own = (unsigned long)mtx->owner & ~3UL;
				oops_in_progress++;
				printk("%s:%d is blocked on %p ",
				       tsks[i]->comm, tsks[i]->pid, mtx);
				if (own == 0x100)
					printk(" owner is READER\n");
				else if (!(own & ~300))
					printk(" owner is ILLEGAL!!\n");
				else if (!own)
					printk(" has no owner!\n");
				else {
					struct task_struct *owner = (void*)own;

					printk(" owner is %s:%d\n",
					       owner->comm, owner->pid);
				}
				oops_in_progress--;

				spin_unlock(&tsks[i]->pi_lock);
				spin_unlock_irq(&mtx->wait_lock);
			} else {
				print_owned_read_locks(tsks[i]);
				spin_unlock_irq(&tsks[i]->pi_lock);
			}
		}
	}
#endif
	for (i=0; i < (thread_count + rt_thread_count); i++) {
		if (tsks[i])
			kthread_stop(tsks[i]);
	}

	print_lock_stat(&test_lock1);
	print_lock_stat(&test_lock2);
	print_lock_stat(&test_lock3);
	print_lock_stat(&test_sem1);
	print_lock_stat(&test_sem2);
	print_lock_stat(&test_sem3);
	print_lock_stat(&test_mutex1);
	print_lock_stat(&test_mutex2);
	print_lock_stat(&test_mutex3);

	if (!perform_downgrade_write) {
		printk("No downgrade writes performed.\n"
		       "   To enable it, pass in perform_downgrade_write=1 to the module\n");
	}

	return 0;
}

static void mutex_stress_exit(void)
{
}

module_init(mutex_stress_init);
module_exit(mutex_stress_exit);

module_param(perform_downgrade_write, int, 0644);
MODULE_PARM_DESC(perform_downgrade_write,
		 "Perform downgrade_write in the test");

module_param(sched_other_time_usecs, ulong, 0644);
MODULE_PARM_DESC(sched_other_time_usecs,
		 "Number of usecs to \"do something\"");

module_param(sched_fifo_time_usecs, ulong, 0644);
MODULE_PARM_DESC(sched_fifo_time_usecs,
		 "Number of usecs for rt tasks to \"do something\"");

module_param(sched_other_sleep_ms, uint, 0644);
MODULE_PARM_DESC(sched_other_sleep_ms,
		 "Number of usecs for tasks to sleep");

module_param(sched_fifo_sleep_ms, uint, 0644);
MODULE_PARM_DESC(sched_fifo_sleep_ms,
		 "Number of usecs for rt tasks to sleep");

module_param(rt_thread_prio, long, 0644);
MODULE_PARM_DESC(rt_thread_prio, "priority if FIFO tasks");

module_param(thread_count, uint, 0644);
MODULE_PARM_DESC(thread_count, "Number of threads to run");

module_param(rt_thread_count, uint, 0644);
MODULE_PARM_DESC(rt_thread_count, "Number of RT threads to run");

module_param(test_time, uint, 0644);
MODULE_PARM_DESC(test_time, "Number of seconds to run the test");

MODULE_AUTHOR("Steven Rostedt");
MODULE_DESCRIPTION("Mutex Stress");
MODULE_LICENSE("GPL");
