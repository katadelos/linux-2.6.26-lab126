/*
 * Copyright (c) 2000-2004 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dmapool.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/usb.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/clk.h>

#include "../core/hcd.h"

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>

#include <asm/arch/clock.h>
#include <asm/arch/arc_otg.h>

#include <net/mwan.h>

/*-------------------------------------------------------------------------*/

/*
 * EHCI hc_driver implementation ... experimental, incomplete.
 * Based on the final 1.0 register interface specification.
 *
 * USB 2.0 shows up in upcoming www.pcmcia.org technology.
 * First was PCMCIA, like ISA; then CardBus, which is PCI.
 * Next comes "CardBay", using USB 2.0 signals.
 *
 * Contains additional contributions by Brad Hards, Rory Bolt, and others.
 * Special thanks to Intel and VIA for providing host controllers to
 * test this driver on, and Cypress (including In-System Design) for
 * providing early devices for those host controllers to talk to!
 */

#define DRIVER_VERSION "10 Dec 2004"
#define DRIVER_AUTHOR "David Brownell"
#define DRIVER_DESC "USB 2.0 'Enhanced' Host Controller (EHCI) Driver"

static const char	hcd_name [] = "ehci_hcd";


#undef VERBOSE_DEBUG
#undef EHCI_URB_TRACE

#ifdef DEBUG
#define EHCI_STATS
#endif

/* magic numbers that can affect system performance */
#define	EHCI_TUNE_CERR		3	/* 0-3 qtd retries; 0 == don't stop */
#define	EHCI_TUNE_RL_HS		4	/* nak throttle; see 4.9 */
#define	EHCI_TUNE_RL_TT		0
#define	EHCI_TUNE_MULT_HS	1	/* 1-3 transactions/uframe; 4.10.3 */
#define	EHCI_TUNE_MULT_TT	1
#define	EHCI_TUNE_FLS		2	/* (small) 256 frame schedule */

#define EHCI_IAA_MSECS		10		/* arbitrary */
#define EHCI_IO_JIFFIES		(HZ/10)		/* io watchdog > irq_thresh */
#define EHCI_ASYNC_JIFFIES	(HZ/20)		/* async idle timeout */
#define EHCI_SHRINK_JIFFIES	(HZ/200)	/* async qh unlink delay */

/* Initial IRQ latency:  faster than hw default */
static int log2_irq_thresh = 0;		// 0 to 6
module_param (log2_irq_thresh, int, S_IRUGO);
MODULE_PARM_DESC (log2_irq_thresh, "log2 IRQ latency, 1-64 microframes");

/* initial park setting:  slower than hw default */
static unsigned park = 0;
module_param (park, uint, S_IRUGO);
MODULE_PARM_DESC (park, "park setting; 1-3 back-to-back async packets");

/* for flakey hardware, ignore overcurrent indicators */
static int ignore_oc = 0;
module_param (ignore_oc, bool, S_IRUGO);
MODULE_PARM_DESC (ignore_oc, "ignore bogus hardware overcurrent indications");

#define	INTR_MASK (STS_IAA | STS_FATAL | STS_PCD | STS_ERR | STS_INT)

/*-------------------------------------------------------------------------*/

#include "ehci.h"
#include "ehci-dbg.c"

/*-------------------------------------------------------------------------*/


/*
 * handshake - spin reading hc until handshake completes or fails
 * @ptr: address of hc register to be read
 * @mask: bits to look at in result of read
 * @done: value of those bits when handshake succeeds
 * @usec: timeout in microseconds
 *
 * Returns negative errno, or zero on success
 *
 * Success happens when the "mask" bits have the specified value (hardware
 * handshake done).  There are two failure modes:  "usec" have passed (major
 * hardware flakeout), or the register reads as all-ones (hardware removed).
 *
 * That last failure should_only happen in cases like physical cardbus eject
 * before driver shutdown. But it also seems to be caused by bugs in cardbus
 * bridge shutdown:  shutting down the bridge before the devices using it.
 */
static int handshake (struct ehci_hcd *ehci, void __iomem *ptr,
		      u32 mask, u32 done, int usec)
{
	u32	result;

	do {
		result = ehci_readl(ehci, ptr);
		if (result == ~(u32)0)		/* card removed */
			return -ENODEV;
		result &= mask;
		if (result == done)
			return 0;
		udelay (1);
		usec--;
	} while (usec > 0);
	return -ETIMEDOUT;
}

/* force HC to halt state from unknown (EHCI spec section 2.3) */
static int ehci_halt (struct ehci_hcd *ehci)
{
	u32	temp = ehci_readl(ehci, &ehci->regs->status);

	/* disable any irqs left enabled by previous code */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);

	if ((temp & STS_HALT) != 0)
		return 0;

	temp = ehci_readl(ehci, &ehci->regs->command);
	temp &= ~CMD_RUN;
	ehci_writel(ehci, temp, &ehci->regs->command);
	return handshake (ehci, &ehci->regs->status,
			  STS_HALT, STS_HALT, 16 * 125);
}

static int handshake_on_error_set_halt(struct ehci_hcd *ehci, void __iomem *ptr,
					u32 mask, u32 done, int usec)
{
	int error;
	
	error = handshake(ehci, ptr, mask, done, usec);
	if (error) {
		ehci_halt(ehci);
		ehci_to_hcd(ehci)->state = HC_STATE_HALT;
		ehci_err(ehci, "force halt; handhake %p %08x %08x -> %d\n",
			ptr, mask, done, error);
	}

	return error;
}

/* put TDI/ARC silicon into EHCI mode */
static void tdi_reset (struct ehci_hcd *ehci)
{
	u32 __iomem	*reg_ptr;
	u32		tmp;

	reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
	tmp = ehci_readl(ehci, reg_ptr);
	tmp |= USBMODE_CM_HC;
	/* The default byte access to MMR space is LE after
	 * controller reset. Set the required endian mode
	 * for transfer buffers to match the host microprocessor
	 */
	if (ehci_big_endian_mmio(ehci))
		tmp |= USBMODE_BE;
	ehci_writel(ehci, tmp, reg_ptr);
}

/* reset a non-running (STS_HALT == 1) controller */
static int ehci_reset (struct ehci_hcd *ehci)
{
	int	retval;
	u32	command = ehci_readl(ehci, &ehci->regs->command);

	command |= CMD_RESET;
	dbg_cmd (ehci, "reset", command);
	ehci_writel(ehci, command, &ehci->regs->command);
	ehci_to_hcd(ehci)->state = HC_STATE_HALT;
	ehci->next_statechange = jiffies;
	retval = handshake (ehci, &ehci->regs->command,
			    CMD_RESET, 0, 250 * 1000);

	if (retval)
		return retval;

	if (ehci_is_TDI(ehci))
		tdi_reset (ehci);

	return retval;
}

/* idle the controller (from running) */
static void ehci_quiesce (struct ehci_hcd *ehci)
{
	u32	temp;

#ifdef DEBUG
	if (!HC_IS_RUNNING (ehci_to_hcd(ehci)->state))
		BUG ();
#endif

	/* wait for any schedule enables/disables to take effect */
	temp = ehci_readl(ehci, &ehci->regs->command) << 10;
	temp &= STS_ASS | STS_PSS;
	if (handshake_on_error_set_halt(ehci, &ehci->regs->status,
					STS_ASS | STS_PSS, temp, 16 * 125))
		return;

	/* then disable anything that's still active */
	temp = ehci_readl(ehci, &ehci->regs->command);
	temp &= ~(CMD_ASE | CMD_IAAD | CMD_PSE);
	ehci_writel(ehci, temp, &ehci->regs->command);

	/* hardware can take 16 microframes to turn off ... */
	handshake_on_error_set_halt(ehci, &ehci->regs->status,
				    STS_ASS | STS_PSS, 0, 16 * 125);
}

/*
 * USB EHCI low power idle mode.
 */
extern int wakeup_value;			/* Enter IDLE or not */
#define EHCI_IDLE_SAMPLING_RATE 	1000	/* time (ms) between idle loop iterations */

atomic_t ehci_irq_last_count = ATOMIC_INIT(0);
atomic_t ehci_irq_current_count = ATOMIC_INIT(0);
static int kick_off_delayed_work = 0;
struct ehci_hcd *ehci_low_power;

static void ehci_idle_count(struct work_struct *unused);
atomic_t suspended = ATOMIC_INIT(0);
static struct clk *usb_ahb_clk;

int suspend_count = 0;
EXPORT_SYMBOL(suspend_count);

/*-------------------------------------------------------------------------*/

static void end_unlink_async(struct ehci_hcd *ehci);
static void ehci_work(struct ehci_hcd *ehci);

#include "ehci-hub.c"
#ifdef CONFIG_USB_STATIC_IRAM
#include "ehci-mem-iram.c"
#include "ehci-q-iram.c"
#else
#include "ehci-mem.c"
#include "ehci-q.c"
#endif
#include "ehci-sched.c"

/*-------------------------------------------------------------------------*/

/*
 * Suspend the USB bus that is now idle
 */
static int ehci_idle_bus_suspend (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	int			port;
	int 			mask;

	ehci_dbg(ehci, "suspend root hub\n");

	if (time_before (jiffies, ehci->next_statechange))
		mdelay(15);

	del_timer_sync(&ehci->watchdog);
	del_timer_sync(&ehci->iaa_watchdog);

	port = HCS_N_PORTS (ehci->hcs_params);

	/* stop schedules, clean any completed work */
	if (HC_IS_RUNNING(hcd->state)) {
		ehci_quiesce (ehci);
		hcd->state = HC_STATE_QUIESCING;
	}

	ehci->command = ehci_readl(ehci, &ehci->regs->command);

	ehci_work(ehci);

	ehci->bus_suspended = 0;
	ehci->owned_ports = 0;
	while (port--) {
		u32 __iomem	*reg = &ehci->regs->port_status [port];
		u32		t1 = ehci_readl(ehci, reg) & ~PORT_RWC_BITS;
		u32		t2 = t1;

		/* keep track of which ports we suspend */
		if (t1 & PORT_OWNER)
			set_bit(port, &ehci->owned_ports);
		else if ((t1 & PORT_PE) && !(t1 & PORT_SUSPEND)) {
			t2 |= PORT_SUSPEND;
			set_bit(port, &ehci->bus_suspended);
		}

		/* enable remote wakeup on all ports */
		t2 &= ~PORT_WAKE_BITS;

		if (t1 != t2) {
			ehci_vdbg (ehci, "port %d, %08x -> %08x\n",
				port + 1, t1, t2);
			ehci_writel(ehci, t2, reg);
		}
	}

	if (ehci->bus_suspended)
		udelay(150);

	ehci_halt (ehci);
	hcd->state = HC_STATE_SUSPENDED;

	if (ehci->reclaim)
		end_unlink_async(ehci);

	mask = INTR_MASK;
	mask &= ~STS_PCD;

	ehci_writel(ehci, mask, &ehci->regs->intr_enable);
	ehci_readl(ehci, &ehci->regs->intr_enable);

	ehci->next_statechange = jiffies + msecs_to_jiffies(10);
	del_timer_sync(&ehci->watchdog);

	mdelay(100);
	return 0;
}
		
/* caller has locked the root hub, and should reset/reinit on error */
static int ehci_idle_bus_resume (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	u32			temp;
	u32			power_okay;
	int			i;

	ehci_dbg (ehci, "ehci_idle_bus_resume\n");

	if (time_before (jiffies, ehci->next_statechange))
		mdelay(15);

	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		return -ESHUTDOWN;
	}

	power_okay = ehci_readl(ehci, &ehci->regs->intr_enable);
	ehci_dbg(ehci, "resume root hub%s\n",
			power_okay ? "" : " after power loss");

	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	ehci_writel(ehci, 0, &ehci->regs->segment);
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32) ehci->async->qh_dma, &ehci->regs->async_next);
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	mdelay(100);

	i = HCS_N_PORTS (ehci->hcs_params);
	while (i--) {
		temp = ehci_readl(ehci, &ehci->regs->port_status [i]);
		temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
		if (test_bit(i, &ehci->bus_suspended) &&
				(temp & PORT_SUSPEND)) {
			ehci->reset_done [i] = jiffies + msecs_to_jiffies (20);
			temp |= PORT_RESUME;
		}
		ehci_writel(ehci, temp, &ehci->regs->port_status [i]);
	}

	mdelay(100);

	i = HCS_N_PORTS (ehci->hcs_params);
	while (i--) {
		temp = ehci_readl(ehci, &ehci->regs->port_status [i]);
		if (test_bit(i, &ehci->bus_suspended) &&
				(temp & PORT_SUSPEND)) {
			temp &= ~(PORT_RWC_BITS | PORT_RESUME);
			ehci_writel(ehci, temp, &ehci->regs->port_status [i]);
			ehci_vdbg (ehci, "resumed port %d\n", i + 1);
		}
	}

	(void) ehci_readl(ehci, &ehci->regs->command);

	temp = 0;
	if (ehci->async->qh_next.qh)
		temp |= CMD_ASE;
	if (ehci->periodic_sched)
		temp |= CMD_PSE;
	if (temp) {
		ehci->command |= temp;
		ehci_writel(ehci, ehci->command, &ehci->regs->command);
	}

	ehci->next_statechange = jiffies + msecs_to_jiffies(5);
	hcd->state = HC_STATE_RUNNING;
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
	mdelay(100);
	return 0;
}

static int ehci_idle_bus_suspend_nonatomic (struct usb_hcd *hcd)
{
        struct ehci_hcd         *ehci = hcd_to_ehci (hcd);
        int                     port;
        int                     mask;

        ehci_dbg(ehci, "suspend root hub\n");

        if (time_before (jiffies, ehci->next_statechange))
                msleep(5);

        del_timer_sync(&ehci->watchdog);
        del_timer_sync(&ehci->iaa_watchdog);

        port = HCS_N_PORTS (ehci->hcs_params);
        spin_lock_irq (&ehci->lock);

        /* stop schedules, clean any completed work */
        if (HC_IS_RUNNING(hcd->state)) {
                ehci_quiesce (ehci);
                hcd->state = HC_STATE_QUIESCING;
        }

        ehci->command = ehci_readl(ehci, &ehci->regs->command);

        ehci_work(ehci);

        ehci->bus_suspended = 0;
        ehci->owned_ports = 0;
        while (port--) {
                u32 __iomem     *reg = &ehci->regs->port_status [port];
                u32             t1 = ehci_readl(ehci, reg) & ~PORT_RWC_BITS;
                u32             t2 = t1;

                /* keep track of which ports we suspend */
                if (t1 & PORT_OWNER)
                        set_bit(port, &ehci->owned_ports);
                else if ((t1 & PORT_PE) && !(t1 & PORT_SUSPEND)) {
                        t2 |= PORT_SUSPEND;
                        set_bit(port, &ehci->bus_suspended);
                }

                /* enable remote wakeup on all ports */
                if (hcd->self.root_hub->do_remote_wakeup)
			t2 |= PORT_WAKE_BITS;
                else
                        t2 &= ~PORT_WAKE_BITS;

                if (t1 != t2) {
                        ehci_vdbg (ehci, "port %d, %08x -> %08x\n",
                                port + 1, t1, t2);
                        ehci_writel(ehci, t2, reg);
                }
        }

        if (ehci->bus_suspended)
                udelay(150);

        ehci_halt (ehci);
        hcd->state = HC_STATE_SUSPENDED;

        if (ehci->reclaim)
                end_unlink_async(ehci);

        mask = INTR_MASK;
        if (!hcd->self.root_hub->do_remote_wakeup)
                mask &= ~STS_PCD;

        ehci_writel(ehci, mask, &ehci->regs->intr_enable);
        ehci_readl(ehci, &ehci->regs->intr_enable);

        ehci->next_statechange = jiffies + msecs_to_jiffies(10);
        spin_unlock_irq (&ehci->lock);
        del_timer_sync(&ehci->watchdog);
        return 0;
}

/* caller has locked the root hub, and should reset/reinit on error */
static int ehci_idle_bus_resume_nonatomic (struct usb_hcd *hcd)
{
        struct ehci_hcd         *ehci = hcd_to_ehci (hcd);
        u32                     temp;
        u32                     power_okay;
        int                     i;
        u8                      resume_needed = 0;

        if (time_before (jiffies, ehci->next_statechange))
                msleep(5);

        spin_lock_irq (&ehci->lock);
        if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
                spin_unlock_irq(&ehci->lock);
                return -ESHUTDOWN;
        }

        power_okay = ehci_readl(ehci, &ehci->regs->intr_enable);
        ehci_dbg(ehci, "resume root hub%s\n",
                        power_okay ? "" : " after power loss");

        ehci_writel(ehci, 0, &ehci->regs->intr_enable);

        ehci_writel(ehci, 0, &ehci->regs->segment);
        ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
        ehci_writel(ehci, (u32) ehci->async->qh_dma, &ehci->regs->async_next);

        ehci_writel(ehci, ehci->command, &ehci->regs->command);
        spin_unlock_irq(&ehci->lock);
        msleep(8);
        spin_lock_irq(&ehci->lock);

        i = HCS_N_PORTS (ehci->hcs_params);
        while (i--) {
                temp = ehci_readl(ehci, &ehci->regs->port_status [i]);
                temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
                if (test_bit(i, &ehci->bus_suspended) &&
                                (temp & PORT_SUSPEND)) {
                        ehci->reset_done [i] = jiffies + msecs_to_jiffies (20);
                        temp |= PORT_RESUME;
                        resume_needed = 1;
                }
                ehci_writel(ehci, temp, &ehci->regs->port_status [i]);
        }

        if (resume_needed) {
                spin_unlock_irq(&ehci->lock);
                msleep(20);
                spin_lock_irq(&ehci->lock);
        }

	i = HCS_N_PORTS (ehci->hcs_params);
        while (i--) {
                temp = ehci_readl(ehci, &ehci->regs->port_status [i]);
                if (test_bit(i, &ehci->bus_suspended) &&
                                (temp & PORT_SUSPEND)) {
                        temp &= ~(PORT_RWC_BITS | PORT_RESUME);
                        ehci_writel(ehci, temp, &ehci->regs->port_status [i]);
                        ehci_vdbg (ehci, "resumed port %d\n", i + 1);
                }
        }

        (void) ehci_readl(ehci, &ehci->regs->command);

        temp = 0;
        if (ehci->async->qh_next.qh)
                temp |= CMD_ASE;
        if (ehci->periodic_sched)
                temp |= CMD_PSE;
        if (temp) {
                ehci->command |= temp;
                ehci_writel(ehci, ehci->command, &ehci->regs->command);
        }

        ehci->next_statechange = jiffies + msecs_to_jiffies(5);
        hcd->state = HC_STATE_RUNNING;
        ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
        spin_unlock_irq (&ehci->lock);
        ehci_handover_companion_ports(ehci);
        return 0;
}


/* Flag that keeps track of USB HC1 in low power idle mode */
extern int usb_hc1_gated;

static int in_deep_idle = 0;
static int deep_idle_enable_value = 0;

/* Ehci is ready to suspend */
int ehci_suspending = 0;

/* Audio playing flag */
extern int mx35_luigi_audio_playing_flag;

static void ehci_low_power_enter(struct ehci_hcd *ehci)
{
	if (atomic_read(&suspended) == 1)
		return;

	/* Ehci is suspending, return */
	if (ehci_suspending)
		return;

	if (mx35_luigi_audio_playing_flag)
		return;

	if (wakeup_value == 1) {
		ehci_dbg (ehci, "entering idle\n");
		spin_lock_irq(&ehci->lock);

		atomic_set(&suspended, 1);

		USBCTRL |= UCTRL_H2WIE; 

		ehci_idle_bus_suspend(ehci_to_hcd(ehci));
		ehci_low_power = ehci;

		usb_hc1_gated = 1;
		suspend_count++;
		if (deep_idle_enable_value) {
			ehci_dbg(ehci, "entering deep idle\n");
			clk_disable(usb_ahb_clk);
			in_deep_idle = 1;
		}

		spin_unlock_irq(&ehci->lock);
		printk(KERN_DEBUG "ehci entering lpm\n");
	}
}

static void ehci_low_power_exit(void)
{
	if (wakeup_value == 0)
		return;

	if (atomic_read(&suspended) == 0) {
		return;
	}

	if (mx35_luigi_audio_playing_flag && !atomic_read(&suspended))
		return;

	spin_lock_irq(&ehci_low_power->lock);

	atomic_set(&suspended, 0);

	if (in_deep_idle) {
		ehci_dbg(ehci_low_power, "leaving deep idle\n");
		clk_enable(usb_ahb_clk);
		in_deep_idle = 0;
	}

	usb_hc1_gated = 0;

	ehci_idle_bus_resume(ehci_to_hcd(ehci_low_power));

	USBCTRL &= ~(UCTRL_H2WIE);
	ehci_dbg (ehci_low_power, "left idle\n");
	spin_unlock_irq (&ehci_low_power->lock);
	printk(KERN_DEBUG "ehci exiting lpm\n");
}

static void ehci_low_power_enter_nonatomic (struct ehci_hcd *ehci)
{
        if (atomic_read(&suspended) == 1)
                return;

        /* Ehci is suspending, return */
        if (ehci_suspending)
                return;

	if (mx35_luigi_audio_playing_flag)
                return;

        if (wakeup_value == 1) {
                ehci_dbg (ehci, "entering idle\n");
                spin_lock_irq(&ehci->lock);
                ehci_low_power = ehci;
                USBCTRL |= UCTRL_H2WIE;

                spin_unlock_irq(&ehci->lock);
                ehci_idle_bus_suspend_nonatomic(ehci_to_hcd(ehci));
                spin_lock_irq(&ehci->lock);

                usb_hc1_gated = 1;
                suspend_count++;
                if (deep_idle_enable_value) {
                        ehci_dbg(ehci, "entering deep idle\n");
                        clk_disable(usb_ahb_clk);
                        in_deep_idle = 1;
                }

                atomic_set(&suspended, 1);
                spin_unlock_irq(&ehci->lock);

		printk("Enter LPM\n");
        }

}

static void ehci_low_power_exit_nonatomic (void)
{
	if (wakeup_value == 0)
                return;

        if (atomic_read(&suspended) == 0) {
                return;
        }

	if (mx35_luigi_audio_playing_flag && !atomic_read(&suspended))
                return;

        spin_lock_irq(&ehci_low_power->lock);
        if (in_deep_idle) {
                ehci_dbg(ehci_low_power, "leaving deep idle\n");
                clk_enable(usb_ahb_clk);
                in_deep_idle = 0;
        }

        usb_hc1_gated = 0;

        spin_unlock_irq(&ehci_low_power->lock);
        ehci_idle_bus_resume_nonatomic(ehci_to_hcd(ehci_low_power));
        spin_lock_irq(&ehci_low_power->lock);

        USBCTRL &= ~(UCTRL_H2WIE);
        atomic_set(&suspended, 0);
        spin_unlock_irq(&ehci_low_power->lock);
        ehci_dbg (ehci_low_power, "left idle\n");

	printk("left idle\n");
}

/* time in seconds required for USB to sleep if there is no activity */
#define EHCI_NORMAL_IDLE_THRESHOLD	3	/* normal case (seconds) */
#define EHCI_HOST_WAKE_IDLE_THRESHOLD	30	/* after deep idle host wake */
#define EHCI_IRQ_SAMPLING_RATE		1000	/* 1 seconds after the ehci IRQs begin */

atomic_t ehci_idle_counter = ATOMIC_INIT(0);
atomic_t ehci_idle_threshold = ATOMIC_INIT(EHCI_NORMAL_IDLE_THRESHOLD);

static void ehci_idle_count(struct work_struct *work)
{
	struct ehci_hcd *ehci = container_of(work, struct ehci_hcd, dwork.work);
	unsigned long temp;

	if (atomic_read(&ehci_irq_last_count) !=
		atomic_read(&ehci_irq_current_count)) {
			temp = atomic_read(&ehci_irq_current_count);
			atomic_set(&ehci_irq_last_count, temp);
			atomic_set(&ehci_idle_counter, 0);
			if (!ehci_suspending) {
				schedule_delayed_work(&ehci->dwork,
					msecs_to_jiffies(EHCI_IDLE_SAMPLING_RATE));
			}
	}
	else {
		if (atomic_read(&ehci_idle_counter) >= 
			atomic_read(&ehci_idle_threshold)) {
		    	atomic_set(&ehci_idle_threshold, EHCI_NORMAL_IDLE_THRESHOLD);
			atomic_set(&ehci_idle_counter, 0);
			ehci_low_power_enter_nonatomic(ehci);
			kick_off_delayed_work = 0;
		}
		else {
			atomic_inc(&ehci_idle_counter);
			ehci_dbg(ehci, "no activity\n");
			if (!ehci_suspending) {
				schedule_delayed_work(&ehci->dwork,
					msecs_to_jiffies(EHCI_IDLE_SAMPLING_RATE));
			}
		}
	}
}

void ehci_hcd_recalc_work(void)
{
	if (atomic_read(&suspended) == 0) {
		return;
	}

	atomic_set(&ehci_idle_counter, 0);
	ehci_low_power_exit();
}
EXPORT_SYMBOL(ehci_hcd_recalc_work);

static void ehci_hcd_recalc_work_nonatomic(void)
{
	if (atomic_read(&suspended) == 0) {
                return;
        }

        atomic_set(&ehci_idle_counter, 0);
        ehci_low_power_exit_nonatomic();
}

void ehci_hcd_restart_idle(void)
{
	if ((atomic_read(&suspended) == 0) && suspend_count) {
	    	atomic_set(&ehci_idle_counter, 0);
		schedule_delayed_work(&ehci_low_power->dwork,
			msecs_to_jiffies(EHCI_IDLE_SAMPLING_RATE));
	}
}
EXPORT_SYMBOL(ehci_hcd_restart_idle);

static void ehci_hcd_host_wake(void)
{
	/* if in deep idle, set long idle threshold */
	if (deep_idle_enable_value) {
		atomic_set(&ehci_idle_threshold, EHCI_HOST_WAKE_IDLE_THRESHOLD);
	}
	ehci_hcd_recalc_work_nonatomic();
}

static void ehci_iaa_watchdog(unsigned long param)
{
	struct ehci_hcd		*ehci = (struct ehci_hcd *) param;
	unsigned long		flags;

	/*
	 * If suspended, no need to run the timer and wakeup the bus
	 */
	if (atomic_read(&suspended) == 1)
		return;

	spin_lock_irqsave (&ehci->lock, flags);

	/* Lost IAA irqs wedge things badly; seen first with a vt8235.
	 * So we need this watchdog, but must protect it against both
	 * (a) SMP races against real IAA firing and retriggering, and
	 * (b) clean HC shutdown, when IAA watchdog was pending.
	 */
	if (ehci->reclaim
			&& !timer_pending(&ehci->iaa_watchdog)
			&& HC_IS_RUNNING(ehci_to_hcd(ehci)->state)) {
		u32 cmd, status;

		/* If we get here, IAA is *REALLY* late.  It's barely
		 * conceivable that the system is so busy that CMD_IAAD
		 * is still legitimately set, so let's be sure it's
		 * clear before we read STS_IAA.  (The HC should clear
		 * CMD_IAAD when it sets STS_IAA.)
		 */
		cmd = ehci_readl(ehci, &ehci->regs->command);
		if (cmd & CMD_IAAD)
			ehci_writel(ehci, cmd & ~CMD_IAAD,
					&ehci->regs->command);

		/* If IAA is set here it either legitimately triggered
		 * before we cleared IAAD above (but _way_ late, so we'll
		 * still count it as lost) ... or a silicon erratum:
		 * - VIA seems to set IAA without triggering the IRQ;
		 * - IAAD potentially cleared without setting IAA.
		 */
		status = ehci_readl(ehci, &ehci->regs->status);
		if ((status & STS_IAA) || !(cmd & CMD_IAAD)) {
			COUNT (ehci->stats.lost_iaa);
			ehci_writel(ehci, STS_IAA, &ehci->regs->status);
		}

		ehci_vdbg(ehci, "IAA watchdog: status %x cmd %x\n",
				status, cmd);
		end_unlink_async(ehci);
	}

	spin_unlock_irqrestore(&ehci->lock, flags);
}

static void ehci_watchdog(unsigned long param)
{
	struct ehci_hcd		*ehci = (struct ehci_hcd *) param;
	unsigned long		flags;

	/*
	 * If suspended, no need to run the timer and wakeup the bus
	 */
	if (atomic_read(&suspended) == 1)
		return;

	spin_lock_irqsave(&ehci->lock, flags);

	/* stop async processing after it's idled a bit */
	if (test_bit (TIMER_ASYNC_OFF, &ehci->actions))
		start_unlink_async (ehci, ehci->async);

	/* ehci could run by timer, without IRQs ... */
	ehci_work (ehci);

	spin_unlock_irqrestore (&ehci->lock, flags);
}

/* On some systems, leaving remote wakeup enabled prevents system shutdown.
 * The firmware seems to think that powering off is a wakeup event!
 * This routine turns off remote wakeup and everything else, on all ports.
 */
static void ehci_turn_off_all_ports(struct ehci_hcd *ehci)
{
	int	port = HCS_N_PORTS(ehci->hcs_params);

	while (port--)
		ehci_writel(ehci, PORT_RWC_BITS,
				&ehci->regs->port_status[port]);
}

/*
 * Halt HC, turn off all ports, and let the BIOS use the companion controllers.
 * Should be called with ehci->lock held.
 */
static void ehci_silence_controller(struct ehci_hcd *ehci)
{
	ehci_halt(ehci);
	ehci_turn_off_all_ports(ehci);

	/* make BIOS/etc use companion controller during reboot */
	ehci_writel(ehci, 0, &ehci->regs->configured_flag);

	/* unblock posted writes */
	ehci_readl(ehci, &ehci->regs->configured_flag);
}

/* ehci_shutdown kick in for silicon on any bus (not just pci, etc).
 * This forcibly disables dma and IRQs, helping kexec and other cases
 * where the next system software may expect clean state.
 */
static void ehci_shutdown(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);

	del_timer_sync(&ehci->watchdog);
	del_timer_sync(&ehci->iaa_watchdog);

	spin_lock_irq(&ehci->lock);
	ehci_silence_controller(ehci);
	spin_unlock_irq(&ehci->lock);
}

static void ehci_port_power (struct ehci_hcd *ehci, int is_on)
{
	unsigned port;

	if (!HCS_PPC (ehci->hcs_params))
		return;

	ehci_dbg (ehci, "...power%s ports...\n", is_on ? "up" : "down");
	for (port = HCS_N_PORTS (ehci->hcs_params); port > 0; )
		(void) ehci_hub_control(ehci_to_hcd(ehci),
				is_on ? SetPortFeature : ClearPortFeature,
				USB_PORT_FEAT_POWER,
				port--, NULL, 0);
	/* Flush those writes */
	ehci_readl(ehci, &ehci->regs->command);
	mdelay(20);
}

/*-------------------------------------------------------------------------*/

/*
 * ehci_work is called from some interrupts, timers, and so on.
 * it calls driver completion functions, after dropping ehci->lock.
 */
static void ehci_work (struct ehci_hcd *ehci)
{
	timer_action_done (ehci, TIMER_IO_WATCHDOG);

	/* another CPU may drop ehci->lock during a schedule scan while
	 * it reports urb completions.  this flag guards against bogus
	 * attempts at re-entrant schedule scanning.
	 */
	if (ehci->scanning)
		return;
	ehci->scanning = 1;
	scan_async (ehci);
	if (ehci->next_uframe != -1)
		scan_periodic (ehci);
	ehci->scanning = 0;

	/* the IO watchdog guards against hardware or driver bugs that
	 * misplace IRQs, and should let us run completely without IRQs.
	 * such lossage has been observed on both VT6202 and VT8235.
	 */
	if (HC_IS_RUNNING (ehci_to_hcd(ehci)->state) &&
			(ehci->async->qh_next.ptr != NULL ||
			 ehci->periodic_sched != 0))
		timer_action (ehci, TIMER_IO_WATCHDOG);
}

/*
 * Called when the ehci_hcd module is removed.
 */
static void ehci_stop (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);

	ehci_dbg (ehci, "stop\n");

	/* prevent mwan from waking up USB because module */
	/* is no longer available to respond to callback  */
	wan_set_usb_wake_callback(NULL);

	/* no more interrupts ... */
	del_timer_sync (&ehci->watchdog);
	del_timer_sync(&ehci->iaa_watchdog);
	cancel_rearming_delayed_work(&ehci->dwork);

	spin_lock_irq(&ehci->lock);
	if (HC_IS_RUNNING (hcd->state))
		ehci_quiesce (ehci);

	ehci_silence_controller(ehci);
	ehci_reset (ehci);
	spin_unlock_irq(&ehci->lock);

	remove_companion_file(ehci);
	remove_debug_files (ehci);

	/* root hub is shut down separately (first, when possible) */
	spin_lock_irq (&ehci->lock);
	if (ehci->async)
		ehci_work (ehci);
	spin_unlock_irq (&ehci->lock);
	ehci_mem_cleanup (ehci);

#ifdef	EHCI_STATS
	ehci_dbg (ehci, "irq normal %ld err %ld reclaim %ld (lost %ld)\n",
		ehci->stats.normal, ehci->stats.error, ehci->stats.reclaim,
		ehci->stats.lost_iaa);
	ehci_dbg (ehci, "complete %ld unlink %ld\n",
		ehci->stats.complete, ehci->stats.unlink);
#endif

	dbg_status (ehci, "ehci_stop completed",
		    ehci_readl(ehci, &ehci->regs->status));
}

/* one-time init, only for memory state */
static int ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	u32			temp;
	int			retval;
	u32			hcc_params;

	spin_lock_init(&ehci->lock);

	init_timer(&ehci->watchdog);
	ehci->watchdog.function = ehci_watchdog;
	ehci->watchdog.data = (unsigned long) ehci;

	init_timer(&ehci->iaa_watchdog);
	ehci->iaa_watchdog.function = ehci_iaa_watchdog;
	ehci->iaa_watchdog.data = (unsigned long) ehci;

	usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");

	/*
	 * hw default: 1K periodic list heads, one per frame.
	 * periodic_size can shrink by USBCMD update if hcc_params allows.
	 */
	ehci->periodic_size = DEFAULT_I_TDPS;
	if ((retval = ehci_mem_init(ehci, GFP_KERNEL)) < 0)
		return retval;

	/* controllers may cache some of the periodic schedule ... */
	hcc_params = ehci_readl(ehci, &ehci->caps->hcc_params);
	if (HCC_ISOC_CACHE(hcc_params))		// full frame cache
		ehci->i_thresh = 8;
	else					// N microframes cached
		ehci->i_thresh = 2 + HCC_ISOC_THRES(hcc_params);

	ehci->reclaim = NULL;
	ehci->next_uframe = -1;

	/*
	 * dedicate a qh for the async ring head, since we couldn't unlink
	 * a 'real' qh without stopping the async schedule [4.8].  use it
	 * as the 'reclamation list head' too.
	 * its dummy is used in hw_alt_next of many tds, to prevent the qh
	 * from automatically advancing to the next td after short reads.
	 */
	ehci->async->qh_next.qh = NULL;
	ehci->async->hw_next = QH_NEXT(ehci, ehci->async->qh_dma);
	ehci->async->hw_info1 = cpu_to_hc32(ehci, QH_HEAD);
	ehci->async->hw_token = cpu_to_hc32(ehci, QTD_STS_HALT);
	ehci->async->hw_qtd_next = EHCI_LIST_END(ehci);
	ehci->async->qh_state = QH_STATE_LINKED;
	ehci->async->hw_alt_next = QTD_NEXT(ehci, ehci->async->dummy->qtd_dma);

	INIT_DELAYED_WORK(&ehci->dwork, ehci_idle_count);

	/* clear interrupt enables, set irq latency */
	if (log2_irq_thresh < 0 || log2_irq_thresh > 6)
		log2_irq_thresh = 0;
	temp = 1 << (16 + log2_irq_thresh);
	if (HCC_CANPARK(hcc_params)) {
		/* HW default park == 3, on hardware that supports it (like
		 * NVidia and ALI silicon), maximizes throughput on the async
		 * schedule by avoiding QH fetches between transfers.
		 *
		 * With fast usb storage devices and NForce2, "park" seems to
		 * make problems:  throughput reduction (!), data errors...
		 */
		if (park) {
			park = min(park, (unsigned) 3);
			temp |= CMD_PARK;
			temp |= park << 8;
		}
		ehci_dbg(ehci, "park %d\n", park);
	}
	if (HCC_PGM_FRAMELISTLEN(hcc_params)) {
		/* periodic schedule size can be smaller than default */
		temp &= ~(3 << 2);
		temp |= (EHCI_TUNE_FLS << 2);
		switch (EHCI_TUNE_FLS) {
		case 0: ehci->periodic_size = 1024; break;
		case 1: ehci->periodic_size = 512; break;
		case 2: ehci->periodic_size = 256; break;
		default:	BUG();
		}
	}
	ehci->command = temp;
	wakeup_value = 1;

	/* the ehci structure has been initialized */
	schedule_delayed_work (&ehci->dwork,
		msecs_to_jiffies(EHCI_IDLE_SAMPLING_RATE));

	return 0;
}

/* start HC running; it's halted, ehci_init() has been run (once) */
static int ehci_run (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	int			retval;
	u32			temp;
	u32			hcc_params;

	hcd->uses_new_polling = 1;
	hcd->poll_rh = 0;

	/* EHCI spec section 4.1 */
	if ((retval = ehci_reset(ehci)) != 0) {
		ehci_mem_cleanup(ehci);
		return retval;
	}
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);

	/*
	 * hcc_params controls whether ehci->regs->segment must (!!!)
	 * be used; it constrains QH/ITD/SITD and QTD locations.
	 * pci_pool consistent memory always uses segment zero.
	 * streaming mappings for I/O buffers, like pci_map_single(),
	 * can return segments above 4GB, if the device allows.
	 *
	 * NOTE:  the dma mask is visible through dma_supported(), so
	 * drivers can pass this info along ... like NETIF_F_HIGHDMA,
	 * Scsi_Host.highmem_io, and so forth.  It's readonly to all
	 * host side drivers though.
	 */
	hcc_params = ehci_readl(ehci, &ehci->caps->hcc_params);
	if (HCC_64BIT_ADDR(hcc_params)) {
		ehci_writel(ehci, 0, &ehci->regs->segment);
#if 0
// this is deeply broken on almost all architectures
		if (!dma_set_mask(hcd->self.controller, DMA_64BIT_MASK))
			ehci_info(ehci, "enabled 64bit DMA\n");
#endif
	}


	// Philips, Intel, and maybe others need CMD_RUN before the
	// root hub will detect new devices (why?); NEC doesn't
	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	dbg_cmd (ehci, "init", ehci->command);

	/*
	 * Start, enabling full USB 2.0 functionality ... usb 1.1 devices
	 * are explicitly handed to companion controller(s), so no TT is
	 * involved with the root hub.  (Except where one is integrated,
	 * and there's no companion controller unless maybe for USB OTG.)
	 *
	 * Turning on the CF flag will transfer ownership of all ports
	 * from the companions to the EHCI controller.  If any of the
	 * companions are in the middle of a port reset at the time, it
	 * could cause trouble.  Write-locking ehci_cf_port_reset_rwsem
	 * guarantees that no resets are in progress.  After we set CF,
	 * a short delay lets the hardware catch up; new resets shouldn't
	 * be started before the port switching actions could complete.
	 */
	down_write(&ehci_cf_port_reset_rwsem);
	hcd->state = HC_STATE_RUNNING;
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */
	mdelay(5);
	up_write(&ehci_cf_port_reset_rwsem);

	temp = HC_VERSION(ehci_readl(ehci, &ehci->caps->hc_capbase));
	ehci_info (ehci,
		"USB %x.%x started, EHCI %x.%02x, driver %s%s\n",
		((ehci->sbrn & 0xf0)>>4), (ehci->sbrn & 0x0f),
		temp >> 8, temp & 0xff, DRIVER_VERSION,
		ignore_oc ? ", overcurrent ignored" : "");

	ehci_writel(ehci, INTR_MASK,
		    &ehci->regs->intr_enable); /* Turn On Interrupts */

	/* GRR this is run-once init(), being done every time the HC starts.
	 * So long as they're part of class devices, we can't do it init()
	 * since the class device isn't created that early.
	 */
	create_debug_files(ehci);
	create_companion_file(ehci);

	return 0;
}

/*-------------------------------------------------------------------------*/

static irqreturn_t ehci_irq (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	u32			status, pcd_status = 0, cmd;
	int			bh;

	spin_lock (&ehci->lock);

	atomic_inc(&ehci_irq_current_count);
	if (kick_off_delayed_work == 0) {
		kick_off_delayed_work = 1;

		/* Resume the USB if needed */
		spin_unlock (&ehci->lock);
		ehci_hcd_recalc_work();
		spin_lock (&ehci->lock);

		/* Kick off the delayed workqueue */
		schedule_delayed_work(&ehci->dwork,
			msecs_to_jiffies(EHCI_IRQ_SAMPLING_RATE));
	}

	status = ehci_readl(ehci, &ehci->regs->status);

	/* e.g. cardbus physical eject */
	if (status == ~(u32) 0) {
		ehci_dbg (ehci, "device removed\n");
		goto dead;
	}

	status &= INTR_MASK;
	if (!status) {			/* irq sharing? */
		spin_unlock(&ehci->lock);
		goto out;
	}

	/* clear (just) interrupts */
	ehci_writel(ehci, status, &ehci->regs->status);
	cmd = ehci_readl(ehci, &ehci->regs->command);
	bh = 0;

#ifdef	VERBOSE_DEBUG
	/* unrequested/ignored: Frame List Rollover */
	dbg_status (ehci, "irq", status);
#endif

	/* INT, ERR, and IAA interrupt rates can be throttled */

	/* normal [4.15.1.2] or error [4.15.1.1] completion */
	if (likely ((status & (STS_INT|STS_ERR)) != 0)) {
		if (likely ((status & STS_ERR) == 0))
			COUNT (ehci->stats.normal);
		else
			COUNT (ehci->stats.error);
		bh = 1;
	}

	/* complete the unlinking of some qh [4.15.2.3] */
	if (status & STS_IAA) {
		/* guard against (alleged) silicon errata */
		if (cmd & CMD_IAAD) {
			ehci_writel(ehci, cmd & ~CMD_IAAD,
					&ehci->regs->command);
			ehci_dbg(ehci, "IAA with IAAD still set?\n");
		}
		if (ehci->reclaim) {
			COUNT(ehci->stats.reclaim);
			end_unlink_async(ehci);
		} else
			ehci_dbg(ehci, "IAA with nothing to reclaim?\n");
	}

	/* remote wakeup [4.3.1] */
	if (status & STS_PCD) {
		unsigned	i = HCS_N_PORTS (ehci->hcs_params);

		/* kick root hub later */
		pcd_status = status;

		/* resume root hub? */
		if (!(ehci_readl(ehci, &ehci->regs->command) & CMD_RUN))
			usb_hcd_resume_root_hub(hcd);

		while (i--) {
			int pstatus = ehci_readl(ehci,
						 &ehci->regs->port_status [i]);

			if (pstatus & PORT_OWNER)
				continue;
			if (!(pstatus & PORT_RESUME)
					|| ehci->reset_done [i] != 0)
				continue;

			/* start 20 msec resume signaling from this port,
			 * and make khubd collect PORT_STAT_C_SUSPEND to
			 * stop that signaling.
			 */
			ehci->reset_done [i] = jiffies + msecs_to_jiffies (20);
			ehci_dbg (ehci, "port %d remote wakeup\n", i + 1);
			mod_timer(&hcd->rh_timer, ehci->reset_done[i]);
		}
	}

	/* PCI errors [4.15.2.4] */
	if (unlikely ((status & STS_FATAL) != 0)) {
		dbg_cmd (ehci, "fatal", ehci_readl(ehci,
						   &ehci->regs->command));
		dbg_status (ehci, "fatal", status);
		if (status & STS_HALT) {
			ehci_err (ehci, "fatal error\n");
dead:
			ehci_reset (ehci);
			ehci_writel(ehci, 0, &ehci->regs->configured_flag);
			/* generic layer kills/unlinks all urbs, then
			 * uses ehci_stop to clean up the rest
			 */
			bh = 1;
		}
	}

	if (bh)
		ehci_work (ehci);
	spin_unlock (&ehci->lock);
	if (pcd_status)
		usb_hcd_poll_rh_status(hcd);
out:
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

/*
 * non-error returns are a promise to giveback() the urb later
 * we drop ownership so next owner (or urb unlink) can get it
 *
 * urb + dev is in hcd.self.controller.urb_list
 * we're queueing TDs onto software and hardware lists
 *
 * hcd-specific init for hcpriv hasn't been done yet
 *
 * NOTE:  control, bulk, and interrupt share the same code to append TDs
 * to a (possibly active) QH, and the same QH scanning code.
 */
static int ehci_urb_enqueue (
	struct usb_hcd	*hcd,
	struct urb	*urb,
	gfp_t		mem_flags
) {
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	struct list_head	qtd_list;

	INIT_LIST_HEAD (&qtd_list);

	switch (usb_pipetype (urb->pipe)) {
	case PIPE_CONTROL:
		/* qh_completions() code doesn't handle all the fault cases
		 * in multi-TD control transfers.  Even 1KB is rare anyway.
		 */
		if (urb->transfer_buffer_length > (16 * 1024))
			return -EMSGSIZE;
		/* FALLTHROUGH */
	/* case PIPE_BULK: */
	default:
		if (!qh_urb_transaction (ehci, urb, &qtd_list, mem_flags))
			return -ENOMEM;
		return submit_async(ehci, urb, &qtd_list, mem_flags);

	case PIPE_INTERRUPT:
		if (!qh_urb_transaction (ehci, urb, &qtd_list, mem_flags))
			return -ENOMEM;
		return intr_submit(ehci, urb, &qtd_list, mem_flags);

	case PIPE_ISOCHRONOUS:
		if (urb->dev->speed == USB_SPEED_HIGH)
			return itd_submit (ehci, urb, mem_flags);
		else
			return sitd_submit (ehci, urb, mem_flags);
	}
}

static void unlink_async (struct ehci_hcd *ehci, struct ehci_qh *qh)
{
	/* failfast */
	if (!HC_IS_RUNNING(ehci_to_hcd(ehci)->state) && ehci->reclaim)
		end_unlink_async(ehci);

	/* if it's not linked then there's nothing to do */
	if (qh->qh_state != QH_STATE_LINKED)
		;

	/* defer till later if busy */
	else if (ehci->reclaim) {
		struct ehci_qh		*last;

		for (last = ehci->reclaim;
				last->reclaim;
				last = last->reclaim)
			continue;
		qh->qh_state = QH_STATE_UNLINK_WAIT;
		last->reclaim = qh;

	/* start IAA cycle */
	} else
		start_unlink_async (ehci, qh);
}

/* remove from hardware lists
 * completions normally happen asynchronously
 */

static int ehci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	struct ehci_qh		*qh;
	unsigned long		flags;
	int			rc;

	spin_lock_irqsave (&ehci->lock, flags);
	rc = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (rc)
		goto done;

	switch (usb_pipetype (urb->pipe)) {
	// case PIPE_CONTROL:
	// case PIPE_BULK:
	default:
		qh = (struct ehci_qh *) urb->hcpriv;
		if (!qh)
			break;
		switch (qh->qh_state) {
		case QH_STATE_LINKED:
		case QH_STATE_COMPLETING:
			unlink_async(ehci, qh);
			break;
		case QH_STATE_UNLINK:
		case QH_STATE_UNLINK_WAIT:
			/* already started */
			break;
		case QH_STATE_IDLE:
			WARN_ON(1);
			break;
		}
		break;

	case PIPE_INTERRUPT:
		qh = (struct ehci_qh *) urb->hcpriv;
		if (!qh)
			break;
		switch (qh->qh_state) {
		case QH_STATE_LINKED:
			intr_deschedule (ehci, qh);
			/* FALL THROUGH */
		case QH_STATE_IDLE:
			qh_completions (ehci, qh);
			break;
		default:
			ehci_dbg (ehci, "bogus qh %p state %d\n",
					qh, qh->qh_state);
			goto done;
		}

		/* reschedule QH iff another request is queued */
		if (!list_empty (&qh->qtd_list)
				&& HC_IS_RUNNING (hcd->state)) {
			rc = qh_schedule(ehci, qh);

			/* An error here likely indicates handshake failure
			 * or no space left in the schedule.  Neither fault
			 * should happen often ...
			 *
			 * FIXME kill the now-dysfunctional queued urbs
			 */
			if (rc != 0)
				ehci_err(ehci,
					"can't reschedule qh %p, err %d",
					qh, rc);
		}
		break;

	case PIPE_ISOCHRONOUS:
		// itd or sitd ...

		// wait till next completion, do it then.
		// completion irqs can wait up to 1024 msec,
		break;
	}
done:
	spin_unlock_irqrestore (&ehci->lock, flags);
	return rc;
}

/*-------------------------------------------------------------------------*/

// bulk qh holds the data toggle

static void
ehci_endpoint_disable (struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	unsigned long		flags;
	struct ehci_qh		*qh, *tmp;

	/* ASSERT:  any requests/urbs are being unlinked */
	/* ASSERT:  nobody can be submitting urbs for this any more */

rescan:
	spin_lock_irqsave (&ehci->lock, flags);
	qh = ep->hcpriv;
	if (!qh)
		goto done;

	/* endpoints can be iso streams.  for now, we don't
	 * accelerate iso completions ... so spin a while.
	 */
	if (qh->hw_info1 == 0) {
		ehci_vdbg (ehci, "iso delay\n");
		goto idle_timeout;
	}

	if (!HC_IS_RUNNING (hcd->state))
		qh->qh_state = QH_STATE_IDLE;
	switch (qh->qh_state) {
	case QH_STATE_LINKED:
		for (tmp = ehci->async->qh_next.qh;
				tmp && tmp != qh;
				tmp = tmp->qh_next.qh)
			continue;
		/* periodic qh self-unlinks on empty */
		if (!tmp)
			goto nogood;
		unlink_async (ehci, qh);
		/* FALL THROUGH */
	case QH_STATE_UNLINK:		/* wait for hw to finish? */
	case QH_STATE_UNLINK_WAIT:
idle_timeout:
		spin_unlock_irqrestore (&ehci->lock, flags);
		schedule_timeout_uninterruptible(1);
		goto rescan;
	case QH_STATE_IDLE:		/* fully unlinked */
		if (list_empty (&qh->qtd_list)) {
			qh_put (qh);
			break;
		}
		/* else FALL THROUGH */
	default:
nogood:
		/* caller was supposed to have unlinked any requests;
		 * that's not our job.  just leak this memory.
		 */
		ehci_err (ehci, "qh %p (#%02x) state %d%s\n",
			qh, ep->desc.bEndpointAddress, qh->qh_state,
			list_empty (&qh->qtd_list) ? "" : "(has tds)");
		break;
	}
	ep->hcpriv = NULL;
done:
	spin_unlock_irqrestore (&ehci->lock, flags);
	return;
}

static int ehci_get_frame (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	return (ehci_readl(ehci, &ehci->regs->frame_index) >> 3) %
		ehci->periodic_size;
}

/*-------------------------------------------------------------------------*/

#define DRIVER_INFO DRIVER_VERSION " " DRIVER_DESC

MODULE_DESCRIPTION (DRIVER_INFO);
MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_LICENSE ("GPL");

#ifdef CONFIG_PCI
#include "ehci-pci.c"
#define	PCI_DRIVER		ehci_pci_driver
#endif

#ifdef CONFIG_USB_EHCI_FSL
#include "ehci-fsl.c"
#define	PLATFORM_DRIVER		ehci_fsl_driver
#endif

#ifdef CONFIG_SOC_AU1200
#include "ehci-au1xxx.c"
#define	PLATFORM_DRIVER		ehci_hcd_au1xxx_driver
#endif

#ifdef CONFIG_USB_EHCI_ARC
#include "ehci-arc.c"
#define	PLATFORM_DRIVER		ehci_fsl_driver
#endif

#ifdef CONFIG_PPC_PS3
#include "ehci-ps3.c"
#define	PS3_SYSTEM_BUS_DRIVER	ps3_ehci_driver
#endif

#if defined(CONFIG_440EPX) && !defined(CONFIG_PPC_MERGE)
#include "ehci-ppc-soc.c"
#define	PLATFORM_DRIVER		ehci_ppc_soc_driver
#endif

#ifdef CONFIG_USB_EHCI_HCD_PPC_OF
#include "ehci-ppc-of.c"
#define OF_PLATFORM_DRIVER	ehci_hcd_ppc_of_driver
#endif

#ifdef CONFIG_PLAT_ORION
#include "ehci-orion.c"
#define	PLATFORM_DRIVER		ehci_orion_driver
#endif

#ifdef CONFIG_ARCH_IXP4XX
#include "ehci-ixp4xx.c"
#define	PLATFORM_DRIVER		ixp4xx_ehci_driver
#endif

#if !defined(PCI_DRIVER) && !defined(PLATFORM_DRIVER) && \
    !defined(PS3_SYSTEM_BUS_DRIVER) && !defined(OF_PLATFORM_DRIVER)
#error "missing bus glue for ehci-hcd"
#endif

static int __init ehci_hcd_init(void)
{
	int retval = 0;

	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		 hcd_name,
		 sizeof(struct ehci_qh), sizeof(struct ehci_qtd),
		 sizeof(struct ehci_itd), sizeof(struct ehci_sitd));

#ifdef DEBUG
	ehci_debug_root = debugfs_create_dir("ehci", NULL);
	if (!ehci_debug_root)
		return -ENOENT;
#endif

#ifdef PLATFORM_DRIVER
	retval = platform_driver_register(&PLATFORM_DRIVER);
	if (retval < 0)
		goto clean0;
#endif

#ifdef PCI_DRIVER
	retval = pci_register_driver(&PCI_DRIVER);
	if (retval < 0)
		goto clean1;
#endif

#ifdef PS3_SYSTEM_BUS_DRIVER
	retval = ps3_ehci_driver_register(&PS3_SYSTEM_BUS_DRIVER);
	if (retval < 0)
		goto clean2;
#endif

#ifdef OF_PLATFORM_DRIVER
	retval = of_register_platform_driver(&OF_PLATFORM_DRIVER);
	if (retval < 0)
		goto clean3;
#endif
	return retval;

#ifdef OF_PLATFORM_DRIVER
	/* of_unregister_platform_driver(&OF_PLATFORM_DRIVER); */
clean3:
#endif
#ifdef PS3_SYSTEM_BUS_DRIVER
	ps3_ehci_driver_unregister(&PS3_SYSTEM_BUS_DRIVER);
clean2:
#endif
#ifdef PCI_DRIVER
	pci_unregister_driver(&PCI_DRIVER);
clean1:
#endif
#ifdef PLATFORM_DRIVER
	platform_driver_unregister(&PLATFORM_DRIVER);
clean0:
#endif
#ifdef DEBUG
	debugfs_remove(ehci_debug_root);
	ehci_debug_root = NULL;
#endif
	return retval;
}
module_init(ehci_hcd_init);

static void __exit ehci_hcd_cleanup(void)
{
#ifdef OF_PLATFORM_DRIVER
	of_unregister_platform_driver(&OF_PLATFORM_DRIVER);
#endif
#ifdef PLATFORM_DRIVER
	platform_driver_unregister(&PLATFORM_DRIVER);
#endif
#ifdef PCI_DRIVER
	pci_unregister_driver(&PCI_DRIVER);
#endif
#ifdef PS3_SYSTEM_BUS_DRIVER
	ps3_ehci_driver_unregister(&PS3_SYSTEM_BUS_DRIVER);
#endif
#ifdef DEBUG
	debugfs_remove(ehci_debug_root);
#endif
}
module_exit(ehci_hcd_cleanup);

