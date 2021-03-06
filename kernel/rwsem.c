/* kernel/rwsem.c: R/W semaphores, public implementation
 *
 * Written by David Howells (dhowells@redhat.com).
 * Derived from asm-i386/semaphore.h
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/rwsem.h>

#include <asm/system.h>
#include <asm/atomic.h>

/*
 * lock for reading
 */
void __sched compat_down_read(struct compat_rw_semaphore *sem)
{
	might_sleep();
	rwsem_acquire_read(&sem->dep_map, 0, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_read_trylock, __down_read);
}

EXPORT_SYMBOL(compat_down_read);

/*
 * trylock for reading -- returns 1 if successful, 0 if contention
 */
int compat_down_read_trylock(struct compat_rw_semaphore *sem)
{
	int ret = __down_read_trylock(sem);

	if (ret == 1)
		rwsem_acquire_read(&sem->dep_map, 0, 1, _RET_IP_);
	return ret;
}

EXPORT_SYMBOL(compat_down_read_trylock);

/*
 * lock for writing
 */
void __sched compat_down_write(struct compat_rw_semaphore *sem)
{
	might_sleep();
	rwsem_acquire(&sem->dep_map, 0, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_write_trylock, __down_write);
}

EXPORT_SYMBOL(compat_down_write);

/*
 * trylock for writing -- returns 1 if successful, 0 if contention
 */
int compat_down_write_trylock(struct compat_rw_semaphore *sem)
{
	int ret = __down_write_trylock(sem);

	if (ret == 1)
		rwsem_acquire(&sem->dep_map, 0, 1, _RET_IP_);
	return ret;
}

EXPORT_SYMBOL(compat_down_write_trylock);

/*
 * release a read lock
 */
void compat_up_read(struct compat_rw_semaphore *sem)
{
	rwsem_release(&sem->dep_map, 1, _RET_IP_);

	__up_read(sem);
}

EXPORT_SYMBOL(compat_up_read);

/*
 * release a write lock
 */
void compat_up_write(struct compat_rw_semaphore *sem)
{
	rwsem_release(&sem->dep_map, 1, _RET_IP_);

	__up_write(sem);
}

EXPORT_SYMBOL(compat_up_write);

/*
 * downgrade write lock to read lock
 */
void compat_downgrade_write(struct compat_rw_semaphore *sem)
{
	/*
	 * lockdep: a downgraded write will live on as a write
	 * dependency.
	 */
	__downgrade_write(sem);
}

EXPORT_SYMBOL(compat_downgrade_write);

#ifdef CONFIG_DEBUG_LOCK_ALLOC

void compat_down_read_nested(struct compat_rw_semaphore *sem, int subclass)
{
	might_sleep();
	rwsem_acquire_read(&sem->dep_map, subclass, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_read_trylock, __down_read);
}

EXPORT_SYMBOL(compat_down_read_nested);

void compat_down_read_non_owner(struct compat_rw_semaphore *sem)
{
	might_sleep();

	__down_read(sem);
}

EXPORT_SYMBOL(compat_down_read_non_owner);

void compat_down_write_nested(struct compat_rw_semaphore *sem, int subclass)
{
	might_sleep();
	rwsem_acquire(&sem->dep_map, subclass, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_write_trylock, __down_write);
}

EXPORT_SYMBOL(compat_down_write_nested);

void compat_up_read_non_owner(struct compat_rw_semaphore *sem)
{
	__up_read(sem);
}

EXPORT_SYMBOL(compat_up_read_non_owner);

#endif


