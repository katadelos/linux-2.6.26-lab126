/*
 * Copyright (c) 2000-2002,2005 Silicon Graphics, Inc.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write the Free Software Foundation,
 * Inc.,  51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_types.h"
#include "xfs_log.h"
#include "xfs_inum.h"
#include "xfs_trans.h"
#include "xfs_sb.h"
#include "xfs_ag.h"
#include "xfs_dmapi.h"
#include "xfs_mount.h"
#include "xfs_trans_priv.h"
#include "xfs_error.h"

STATIC void xfs_ail_insert(xfs_ail_t *, xfs_log_item_t *);
STATIC xfs_log_item_t * xfs_ail_delete(xfs_ail_t *, xfs_log_item_t *);
STATIC xfs_log_item_t * xfs_ail_min(xfs_ail_t *);
STATIC xfs_log_item_t * xfs_ail_next(xfs_ail_t *, xfs_log_item_t *);

#ifdef DEBUG
STATIC void xfs_ail_check(xfs_ail_t *, xfs_log_item_t *);
#else
#define	xfs_ail_check(a,l)
#endif /* DEBUG */


/*
 * This is called by the log manager code to determine the LSN
 * of the tail of the log.  This is exactly the LSN of the first
 * item in the AIL.  If the AIL is empty, then this function
 * returns 0.
 *
 * We need the AIL lock in order to get a coherent read of the
 * lsn of the last item in the AIL.
 */
xfs_lsn_t
xfs_trans_tail_ail(
	xfs_mount_t	*mp)
{
	xfs_lsn_t	lsn;
	xfs_log_item_t	*lip;

	spin_lock(&mp->m_ail_lock);
	lip = xfs_ail_min(&mp->m_ail);
	if (lip == NULL) {
		lsn = (xfs_lsn_t)0;
	} else {
		lsn = lip->li_lsn;
	}
	spin_unlock(&mp->m_ail_lock);

	return lsn;
}

/*
 * xfs_trans_push_ail
 *
 * This routine is called to move the tail of the AIL forward.  It does this by
 * trying to flush items in the AIL whose lsns are below the given
 * threshold_lsn.
 *
 * the push is run asynchronously in a separate thread, so we return the tail
 * of the log right now instead of the tail after the push. This means we will
 * either continue right away, or we will sleep waiting on the async thread to
 * do it's work.
 *
 * We do this unlocked - we only need to know whether there is anything in the
 * AIL at the time we are called. We don't need to access the contents of
 * any of the objects, so the lock is not needed.
 */
void
xfs_trans_push_ail(
	xfs_mount_t		*mp,
	xfs_lsn_t		threshold_lsn)
{
	xfs_log_item_t		*lip;

	lip = xfs_ail_min(&mp->m_ail);
	if (lip && !XFS_FORCED_SHUTDOWN(mp)) {
		if (XFS_LSN_CMP(threshold_lsn, mp->m_ail.xa_target) > 0)
			xfsaild_wakeup(mp, threshold_lsn);
	}
}

/*
 * Return the item in the AIL with the current lsn.
 * Return the current tree generation number for use
 * in calls to xfs_trans_next_ail().
 */
STATIC xfs_log_item_t *
xfs_trans_first_push_ail(
	xfs_mount_t	*mp,
	int		*gen,
	xfs_lsn_t	lsn)
{
	xfs_log_item_t	*lip;

	lip = xfs_ail_min(&mp->m_ail);
	*gen = (int)mp->m_ail.xa_gen;
	if (lsn == 0)
		return lip;

	list_for_each_entry(lip, &mp->m_ail.xa_ail, li_ail) {
		if (XFS_LSN_CMP(lip->li_lsn, lsn) >= 0)
			return lip;
	}

	return NULL;
}

/*
 * Function that does the work of pushing on the AIL
 */
long
xfsaild_push(
	xfs_mount_t	*mp,
	xfs_lsn_t	*last_lsn)
{
	long		tout = 1000; /* milliseconds */
	xfs_lsn_t	last_pushed_lsn = *last_lsn;
	xfs_lsn_t	target =  mp->m_ail.xa_target;
	xfs_lsn_t	lsn;
	xfs_log_item_t	*lip;
	int		gen;
	int		restarts;
	int		flush_log, count, stuck;

#define	XFS_TRANS_PUSH_AIL_RESTARTS	10

	spin_lock(&mp->m_ail_lock);
	lip = xfs_trans_first_push_ail(mp, &gen, *last_lsn);
	if (!lip || XFS_FORCED_SHUTDOWN(mp)) {
		/*
		 * AIL is empty or our push has reached the end.
		 */
		spin_unlock(&mp->m_ail_lock);
		last_pushed_lsn = 0;
		goto out;
	}

	XFS_STATS_INC(xs_push_ail);

	/*
	 * While the item we are looking at is below the given threshold
	 * try to flush it out. We'd like not to stop until we've at least
	 * tried to push on everything in the AIL with an LSN less than
	 * the given threshold.
	 *
	 * However, we will stop after a certain number of pushes and wait
	 * for a reduced timeout to fire before pushing further. This
	 * prevents use from spinning when we can't do anything or there is
	 * lots of contention on the AIL lists.
	 */
	tout = 10;
	lsn = lip->li_lsn;
	flush_log = stuck = count = restarts = 0;
	while ((XFS_LSN_CMP(lip->li_lsn, target) < 0)) {
		int	lock_result;
		/*
		 * If we can lock the item without sleeping, unlock the AIL
		 * lock and flush the item.  Then re-grab the AIL lock so we
		 * can look for the next item on the AIL. List changes are
		 * handled by the AIL lookup functions internally
		 *
		 * If we can't lock the item, either its holder will flush it
		 * or it is already being flushed or it is being relogged.  In
		 * any of these case it is being taken care of and we can just
		 * skip to the next item in the list.
		 */
		lock_result = IOP_TRYLOCK(lip);
		spin_unlock(&mp->m_ail_lock);
		switch (lock_result) {
		case XFS_ITEM_SUCCESS:
			XFS_STATS_INC(xs_push_ail_success);
			IOP_PUSH(lip);
			last_pushed_lsn = lsn;
			break;

		case XFS_ITEM_PUSHBUF:
			XFS_STATS_INC(xs_push_ail_pushbuf);
			IOP_PUSHBUF(lip);
			last_pushed_lsn = lsn;
			break;

		case XFS_ITEM_PINNED:
			XFS_STATS_INC(xs_push_ail_pinned);
			stuck++;
			flush_log = 1;
			break;

		case XFS_ITEM_LOCKED:
			XFS_STATS_INC(xs_push_ail_locked);
			last_pushed_lsn = lsn;
			stuck++;
			break;

		case XFS_ITEM_FLUSHING:
			XFS_STATS_INC(xs_push_ail_flushing);
			last_pushed_lsn = lsn;
			stuck++;
			break;

		default:
			ASSERT(0);
			break;
		}

		spin_lock(&mp->m_ail_lock);
		/* should we bother continuing? */
		if (XFS_FORCED_SHUTDOWN(mp))
			break;
		ASSERT(mp->m_log);

		count++;

		/*
		 * Are there too many items we can't do anything with?
		 * If we we are skipping too many items because we can't flush
		 * them or they are already being flushed, we back off and
		 * given them time to complete whatever operation is being
		 * done. i.e. remove pressure from the AIL while we can't make
		 * progress so traversals don't slow down further inserts and
		 * removals to/from the AIL.
		 *
		 * The value of 100 is an arbitrary magic number based on
		 * observation.
		 */
		if (stuck > 100)
			break;

		lip = xfs_trans_next_ail(mp, lip, &gen, &restarts);
		if (lip == NULL)
			break;
		if (restarts > XFS_TRANS_PUSH_AIL_RESTARTS)
			break;
		lsn = lip->li_lsn;
	}
	spin_unlock(&mp->m_ail_lock);

	if (flush_log) {
		/*
		 * If something we need to push out was pinned, then
		 * push out the log so it will become unpinned and
		 * move forward in the AIL.
		 */
		XFS_STATS_INC(xs_push_ail_flush);
		xfs_log_force(mp, (xfs_lsn_t)0, XFS_LOG_FORCE);
	}

	if (!count) {
		/* We're past our target or empty, so idle */
		tout = 1000;
	} else if (XFS_LSN_CMP(lsn, target) >= 0) {
		/*
		 * We reached the target so wait a bit longer for I/O to
		 * complete and remove pushed items from the AIL before we
		 * start the next scan from the start of the AIL.
		 */
		tout += 20;
		last_pushed_lsn = 0;
	} else if ((restarts > XFS_TRANS_PUSH_AIL_RESTARTS) ||
		   ((stuck * 100) / count > 90)) {
		/*
		 * Either there is a lot of contention on the AIL or we
		 * are stuck due to operations in progress. "Stuck" in this
		 * case is defined as >90% of the items we tried to push
		 * were stuck.
		 *
		 * Backoff a bit more to allow some I/O to complete before
		 * continuing from where we were.
		 */
		tout += 10;
	}
out:
	*last_lsn = last_pushed_lsn;
	return tout;
}	/* xfsaild_push */


/*
 * This is to be called when an item is unlocked that may have
 * been in the AIL.  It will wake up the first member of the AIL
 * wait list if this item's unlocking might allow it to progress.
 * If the item is in the AIL, then we need to get the AIL lock
 * while doing our checking so we don't race with someone going
 * to sleep waiting for this event in xfs_trans_push_ail().
 */
void
xfs_trans_unlocked_item(
	xfs_mount_t	*mp,
	xfs_log_item_t	*lip)
{
	xfs_log_item_t	*min_lip;

	/*
	 * If we're forcibly shutting down, we may have
	 * unlocked log items arbitrarily. The last thing
	 * we want to do is to move the tail of the log
	 * over some potentially valid data.
	 */
	if (!(lip->li_flags & XFS_LI_IN_AIL) ||
	    XFS_FORCED_SHUTDOWN(mp)) {
		return;
	}

	/*
	 * This is the one case where we can call into xfs_ail_min()
	 * without holding the AIL lock because we only care about the
	 * case where we are at the tail of the AIL.  If the object isn't
	 * at the tail, it doesn't matter what result we get back.  This
	 * is slightly racy because since we were just unlocked, we could
	 * go to sleep between the call to xfs_ail_min and the call to
	 * xfs_log_move_tail, have someone else lock us, commit to us disk,
	 * move us out of the tail of the AIL, and then we wake up.  However,
	 * the call to xfs_log_move_tail() doesn't do anything if there's
	 * not enough free space to wake people up so we're safe calling it.
	 */
	min_lip = xfs_ail_min(&mp->m_ail);

	if (min_lip == lip)
		xfs_log_move_tail(mp, 1);
}	/* xfs_trans_unlocked_item */


/*
 * Update the position of the item in the AIL with the new
 * lsn.  If it is not yet in the AIL, add it.  Otherwise, move
 * it to its new position by removing it and re-adding it.
 *
 * Wakeup anyone with an lsn less than the item's lsn.  If the item
 * we move in the AIL is the minimum one, update the tail lsn in the
 * log manager.
 *
 * Increment the AIL's generation count to indicate that the tree
 * has changed.
 *
 * This function must be called with the AIL lock held.  The lock
 * is dropped before returning.
 */
void
xfs_trans_update_ail(
	xfs_mount_t	*mp,
	xfs_log_item_t	*lip,
	xfs_lsn_t	lsn) __releases(mp->m_ail_lock)
{
	xfs_log_item_t		*dlip=NULL;
	xfs_log_item_t		*mlip;	/* ptr to minimum lip */

	mlip = xfs_ail_min(&mp->m_ail);

	if (lip->li_flags & XFS_LI_IN_AIL) {
		dlip = xfs_ail_delete(&mp->m_ail, lip);
		ASSERT(dlip == lip);
	} else {
		lip->li_flags |= XFS_LI_IN_AIL;
	}

	lip->li_lsn = lsn;

	xfs_ail_insert(&mp->m_ail, lip);
	mp->m_ail.xa_gen++;

	if (mlip == dlip) {
		mlip = xfs_ail_min(&mp->m_ail);
		spin_unlock(&mp->m_ail_lock);
		xfs_log_move_tail(mp, mlip->li_lsn);
	} else {
		spin_unlock(&mp->m_ail_lock);
	}


}	/* xfs_trans_update_ail */

/*
 * Delete the given item from the AIL.  It must already be in
 * the AIL.
 *
 * Wakeup anyone with an lsn less than item's lsn.    If the item
 * we delete in the AIL is the minimum one, update the tail lsn in the
 * log manager.
 *
 * Clear the IN_AIL flag from the item, reset its lsn to 0, and
 * bump the AIL's generation count to indicate that the tree
 * has changed.
 *
 * This function must be called with the AIL lock held.  The lock
 * is dropped before returning.
 */
void
xfs_trans_delete_ail(
	xfs_mount_t	*mp,
	xfs_log_item_t	*lip) __releases(mp->m_ail_lock)
{
	xfs_log_item_t		*dlip;
	xfs_log_item_t		*mlip;

	if (lip->li_flags & XFS_LI_IN_AIL) {
		mlip = xfs_ail_min(&mp->m_ail);
		dlip = xfs_ail_delete(&mp->m_ail, lip);
		ASSERT(dlip == lip);


		lip->li_flags &= ~XFS_LI_IN_AIL;
		lip->li_lsn = 0;
		mp->m_ail.xa_gen++;

		if (mlip == dlip) {
			mlip = xfs_ail_min(&mp->m_ail);
			spin_unlock(&mp->m_ail_lock);
			xfs_log_move_tail(mp, (mlip ? mlip->li_lsn : 0));
		} else {
			spin_unlock(&mp->m_ail_lock);
		}
	}
	else {
		/*
		 * If the file system is not being shutdown, we are in
		 * serious trouble if we get to this stage.
		 */
		if (XFS_FORCED_SHUTDOWN(mp))
			spin_unlock(&mp->m_ail_lock);
		else {
			xfs_cmn_err(XFS_PTAG_AILDELETE, CE_ALERT, mp,
		"%s: attempting to delete a log item that is not in the AIL",
					__func__);
			spin_unlock(&mp->m_ail_lock);
			xfs_force_shutdown(mp, SHUTDOWN_CORRUPT_INCORE);
		}
	}
}



/*
 * Return the item in the AIL with the smallest lsn.
 * Return the current tree generation number for use
 * in calls to xfs_trans_next_ail().
 */
xfs_log_item_t *
xfs_trans_first_ail(
	xfs_mount_t	*mp,
	int		*gen)
{
	xfs_log_item_t	*lip;

	lip = xfs_ail_min(&mp->m_ail);
	*gen = (int)mp->m_ail.xa_gen;

	return lip;
}

/*
 * If the generation count of the tree has not changed since the
 * caller last took something from the AIL, then return the elmt
 * in the tree which follows the one given.  If the count has changed,
 * then return the minimum elmt of the AIL and bump the restarts counter
 * if one is given.
 */
xfs_log_item_t *
xfs_trans_next_ail(
	xfs_mount_t	*mp,
	xfs_log_item_t	*lip,
	int		*gen,
	int		*restarts)
{
	xfs_log_item_t	*nlip;

	ASSERT(mp && lip && gen);
	if (mp->m_ail.xa_gen == *gen) {
		nlip = xfs_ail_next(&mp->m_ail, lip);
	} else {
		nlip = xfs_ail_min(&mp->m_ail);
		*gen = (int)mp->m_ail.xa_gen;
		if (restarts != NULL) {
			XFS_STATS_INC(xs_push_ail_restarts);
			(*restarts)++;
		}
	}

	return (nlip);
}


/*
 * The active item list (AIL) is a doubly linked list of log
 * items sorted by ascending lsn.  The base of the list is
 * a forw/back pointer pair embedded in the xfs mount structure.
 * The base is initialized with both pointers pointing to the
 * base.  This case always needs to be distinguished, because
 * the base has no lsn to look at.  We almost always insert
 * at the end of the list, so on inserts we search from the
 * end of the list to find where the new item belongs.
 */

/*
 * Initialize the doubly linked list to point only to itself.
 */
int
xfs_trans_ail_init(
	xfs_mount_t	*mp)
{
	INIT_LIST_HEAD(&mp->m_ail.xa_ail);
	return xfsaild_start(mp);
}

void
xfs_trans_ail_destroy(
	xfs_mount_t	*mp)
{
	xfsaild_stop(mp);
}

/*
 * Insert the given log item into the AIL.
 * We almost always insert at the end of the list, so on inserts
 * we search from the end of the list to find where the
 * new item belongs.
 */
STATIC void
xfs_ail_insert(
	xfs_ail_t	*ailp,
	xfs_log_item_t	*lip)
/* ARGSUSED */
{
	xfs_log_item_t	*next_lip;

	/*
	 * If the list is empty, just insert the item.
	 */
	if (list_empty(&ailp->xa_ail)) {
		list_add(&lip->li_ail, &ailp->xa_ail);
		return;
	}

	list_for_each_entry_reverse(next_lip, &ailp->xa_ail, li_ail) {
		if (XFS_LSN_CMP(next_lip->li_lsn, lip->li_lsn) <= 0)
			break;
	}

	ASSERT((&next_lip->li_ail == &ailp->xa_ail) ||
	       (XFS_LSN_CMP(next_lip->li_lsn, lip->li_lsn) <= 0));

	list_add(&lip->li_ail, &next_lip->li_ail);

	xfs_ail_check(ailp, lip);
	return;
}

/*
 * Delete the given item from the AIL.  Return a pointer to the item.
 */
/*ARGSUSED*/
STATIC xfs_log_item_t *
xfs_ail_delete(
	xfs_ail_t	*ailp,
	xfs_log_item_t	*lip)
/* ARGSUSED */
{
	xfs_ail_check(ailp, lip);

	list_del(&lip->li_ail);

	return lip;
}

/*
 * Return a pointer to the first item in the AIL.
 * If the AIL is empty, then return NULL.
 */
STATIC xfs_log_item_t *
xfs_ail_min(
	xfs_ail_t	*ailp)
/* ARGSUSED */
{
	if (list_empty(&ailp->xa_ail))
		return NULL;

	return list_first_entry(&ailp->xa_ail, xfs_log_item_t, li_ail);
}

/*
 * Return a pointer to the item which follows
 * the given item in the AIL.  If the given item
 * is the last item in the list, then return NULL.
 */
STATIC xfs_log_item_t *
xfs_ail_next(
	xfs_ail_t	*ailp,
	xfs_log_item_t	*lip)
/* ARGSUSED */
{
	if (lip->li_ail.next == &ailp->xa_ail)
		return NULL;

	return list_first_entry(&lip->li_ail, xfs_log_item_t, li_ail);
}

#ifdef DEBUG
/*
 * Check that the list is sorted as it should be.
 */
STATIC void
xfs_ail_check(
	xfs_ail_t 	*ailp,
	xfs_log_item_t	*lip)
{
	xfs_log_item_t	*prev_lip;

	if (list_empty(&ailp->xa_ail))
		return;

	/*
	 * Check the next and previous entries are valid.
	 */
	ASSERT((lip->li_flags & XFS_LI_IN_AIL) != 0);
	prev_lip = list_entry(lip->li_ail.prev, xfs_log_item_t, li_ail);
	if (&prev_lip->li_ail != &ailp->xa_ail)
		ASSERT(XFS_LSN_CMP(prev_lip->li_lsn, lip->li_lsn) <= 0);

	prev_lip = list_entry(lip->li_ail.next, xfs_log_item_t, li_ail);
	if (&prev_lip->li_ail != &ailp->xa_ail)
		ASSERT(XFS_LSN_CMP(prev_lip->li_lsn, lip->li_lsn) >= 0);


#ifdef XFS_TRANS_DEBUG
	/*
	 * Walk the list checking lsn ordering, and that every entry has the
	 * XFS_LI_IN_AIL flag set. This is really expensive, so only do it
	 * when specifically debugging the transaction subsystem.
	 */
	prev_lip = list_entry(&ailp->xa_ail, xfs_log_item_t, li_ail);
	list_for_each_entry(lip, &ailp->xa_ail, li_ail) {
		if (&prev_lip->li_ail != &ailp->xa_ail)
			ASSERT(XFS_LSN_CMP(prev_lip->li_lsn, lip->li_lsn) <= 0);
		ASSERT((lip->li_flags & XFS_LI_IN_AIL) != 0);
		prev_lip = lip;
	}
#endif /* XFS_TRANS_DEBUG */
}
#endif /* DEBUG */
