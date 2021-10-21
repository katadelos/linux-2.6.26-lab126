/*
 * Copyright (c) 2004-2006 Atheros Communications Inc.
 * All rights reserved.
 *
 * 
// The software source and binaries included in this development package are
// licensed, not sold. You, or your company, received the package under one
// or more license agreements. The rights granted to you are specifically
// listed in these license agreement(s). All other rights remain with Atheros
// Communications, Inc., its subsidiaries, or the respective owner including
// those listed on the included copyright notices.  Distribution of any
// portion of this package must be in strict compliance with the license
// agreement(s) terms.
// </copyright>
// 
// <summary>
// 	Wifi driver for AR6002
// </summary>
//
 *
 */

#ifndef _DEBUG_LINUX_H_
#define _DEBUG_LINUX_H_

#define DBG_DEFAULTS	(DBG_ERROR|DBG_WARNING)

extern A_UINT32 g_dbg_flags;

#define DBGFMT  "%s() : "
#define DBGARG  __func__
#define DBGFN	A_PRINTF

/* ------- Debug related stuff ------- */
enum {
    ATH_DEBUG_SEND = 0x0001,
    ATH_DEBUG_RECV = 0x0002,
    ATH_DEBUG_SYNC = 0x0004,
    ATH_DEBUG_DUMP = 0x0008,
    ATH_DEBUG_IRQ  = 0x0010,
    ATH_DEBUG_TRC  = 0x0020,
    ATH_DEBUG_WARN = 0x0040,
    ATH_DEBUG_ERR  = 0x0080,
    ATH_LOG_INF	   = 0x0100,
    ATH_DEBUG_BMI  = 0x0110,
    ATH_DEBUG_WMI  = 0x0120,
    ATH_DEBUG_HIF  = 0x0140,
    ATH_DEBUG_HTC  = 0x0180,
    ATH_DEBUG_WLAN = 0x1000,
    ATH_LOG_ERR	   = 0x1010,
    ATH_DEBUG_ANY  = 0xFFFF,
};

#ifdef DEBUG
#define A_DPRINTF(f, a) \
	if(g_dbg_flags & (f)) \
	{	\
		DBGFN a ; \
	}


// TODO FIX usage of A_PRINTF!
#define AR_DEBUG_LVL_CHECK(lvl) (debughtc & (lvl))
#define AR_DEBUG_PRINTBUF(buffer, length, desc) do {   \
    if (debughtc & ATH_DEBUG_DUMP) {             \
        DebugDumpBytes(buffer, length,desc);               \
    }                                            \
} while(0)
#define PRINTX_ARG(arg...) arg
#define AR_DEBUG_PRINTF(flags, args) do {        \
    if (debughtc & (flags)) {                    \
        A_PRINTF(KERN_ALERT PRINTX_ARG args);    \
    }                                            \
} while (0)
#define AR_DEBUG_ASSERT(test) do {               \
    if (!(test)) {                               \
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Debug Assert Caught, File %s, Line: %d, Test:%s \n",__FILE__, __LINE__,#test));         \
    }                                            \
} while(0)
extern int debughtc;
#else
#define AR_DEBUG_PRINTF(flags, args)
#define AR_DEBUG_PRINTBUF(buffer, length, desc)
#define AR_DEBUG_ASSERT(test)
#define AR_DEBUG_LVL_CHECK(lvl) 0
#define A_DPRINTF(f, a)
#endif

#endif /* _DEBUG_LINUX_H_ */
