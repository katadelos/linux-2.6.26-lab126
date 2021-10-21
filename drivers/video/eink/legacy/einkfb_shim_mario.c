/*
 *  linux/drivers/video/eink/legacy/einkfb_shim_mario.c -- eInk framebuffer device platform compatibility
 *
 *      Copyright (C) 2005-2010 Amazon Technologies
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#include "../hal/einkfb_hal.h"
#include "einkfb_shim.h"

#include "einksp_mario.h"

#if PRAGMAS
	#pragma mark Definitions/Globals
#endif

#define HEAD_CRC	2
#define EXTRA_FIELD	4
#define ORIG_NAME	8
#define COMMENT		0x10
#define RESERVED	0xe0

#define DEFLATED	8

#define HEADER_0	0x1f
#define HEADER_1	0x8b
#define BEST_COMP	2
#define UNIX_OS		3

static void *z_inflate_workspace = NULL;
static void *z_deflate_workspace = NULL;

static einkfb_dma_addr_t kernelbuffer_phys = INIT_EINKFB_DMA_ADDR_T();
static int show_logo = 0;

module_param_named(show_logo, show_logo, int, S_IRUGO);
MODULE_PARM_DESC(show_logo, "non-zero to show logo");

#if PRAGMAS
	#pragma mark -
	#pragma mark Local Utilities
	#pragma mark -
#endif

static int init_z_inflate_workspace(void)
{
	int result = 0;
	
	if (!z_inflate_workspace) {
	    z_inflate_workspace = kzalloc(zlib_inflate_workspacesize(), GFP_ATOMIC);
	}
	
	if (z_inflate_workspace) {
		result = 1;
	}

	return (result);
}

static void done_z_inflate_workspace(void)
{
	if (z_inflate_workspace) {
		kfree(z_inflate_workspace);
		z_inflate_workspace = NULL;
	}
}

static int init_z_deflate_workspace(void)
{
	int result = 0;
	
	if (!z_deflate_workspace) {
	    z_deflate_workspace = kzalloc(zlib_deflate_workspacesize(), GFP_ATOMIC);
	}
	
	if (z_deflate_workspace) {
		result = 1;
	}

	return (result);
}

static void done_z_deflate_workspace(void)
{
	if (z_deflate_workspace) {
		kfree(z_deflate_workspace);
		z_deflate_workspace = NULL;
	}
}

// CRC-32 algorithm from:
//  <http://glacier.lbl.gov/cgi-bin/viewcvs.cgi/dor-test/crc32.c?rev=HEAD>

/* Table of CRCs of all 8-bit messages. */
static unsigned long crc_table[256];

/* Flag: has the table been computed? Initially false. */
static int crc_table_computed = 0;

/* Make the table for a fast CRC. */
static void make_crc_table(void)
{
  unsigned long c;
  int n, k;

  for (n = 0; n < 256; n++) {
    c = (unsigned long) n;
    for (k = 0; k < 8; k++) {
      if (c & 1) {
        c = 0xedb88320L ^ (c >> 1);
      } else {
        c = c >> 1;
      }
    }
    crc_table[n] = c;
  }
  crc_table_computed = 1;
}

/*
 Update a running crc with the bytes buf[0..len-1] and return
 the updated crc. The crc should be initialized to zero. Pre- and
 post-conditioning (one's complement) is performed within this
 function so it shouldn't be done by the caller. Usage example:

   unsigned long crc = 0L;

   while (read_buffer(buffer, length) != EOF) {
     crc = update_crc(crc, buffer, length);
   }
   if (crc != original_crc) error();
*/
static unsigned long update_crc(unsigned long crc,
                unsigned char *buf, int len)
{
  unsigned long c = crc ^ 0xffffffffL;
  int n;

  if (!crc_table_computed)
    make_crc_table();
  for (n = 0; n < len; n++) {
    c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
  }
  return c ^ 0xffffffffL;
}

/* Return the CRC of the bytes buf[0..len-1]. */
static unsigned long crc32(unsigned char *buf, int len)
{
  return update_crc(0L, buf, len);
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Utilities
	#pragma mark -
#endif

int einkfb_shim_gunzip(unsigned char *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	z_stream s;
	int r, i, flags;

	if (!init_z_inflate_workspace()) {
		einkfb_print_error ("Error: gunzip failed to allocate workspace\n");
		return (-1);
	}
	
	/* skip header */
	i = 10;
	flags = src[3];
	if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
		einkfb_print_error ("Error: Bad gzipped data\n");
		return (-1);
	}
	if ((flags & EXTRA_FIELD) != 0)
		i = 12 + src[10] + (src[11] << 8);
	if ((flags & ORIG_NAME) != 0)
		while (src[i++] != 0)
			;
	if ((flags & COMMENT) != 0)
		while (src[i++] != 0)
			;
	if ((flags & HEAD_CRC) != 0)
		i += 2;
	if (i >= *lenp) {
		einkfb_print_error ("Error: gunzip out of data in header\n");
		return (-1);
	}

	/* Initialize ourself. */
	s.workspace = z_inflate_workspace;
	r = zlib_inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		einkfb_print_error ("Error: zlib_inflateInit2() returned %d\n", r);
		return (-1);
	}
	s.next_in = src + i;
	s.avail_in = *lenp - i;
	s.next_out = dst;
	s.avail_out = dstlen;
	r = zlib_inflate(&s, Z_FINISH);
	if (r != Z_OK && r != Z_STREAM_END) {
		einkfb_print_error ("Error: zlib_inflate() returned %d\n", r);
		return (-1);
	}
	*lenp = s.next_out - (unsigned char *) dst;
	zlib_inflateEnd(&s);

	return (0);
}

int einkfb_shim_gzip(unsigned char *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	z_stream s;
	int r, i;
	
	if (!init_z_deflate_workspace()) {
		einkfb_print_error ("Error: gzip failed to allocate workspace\n");
		return (-1);
	}

	/* write header 1 (leave hole for deflate's non-gzip header) */
	for (i = 0; i < 8; i++) {
		dst[i] = 0;
	}
	dst[0] = HEADER_0;
	dst[1] = HEADER_1;
	dst[2] = DEFLATED;
	
	/* Initialize ourself. */
	s.workspace = z_deflate_workspace;
	r = zlib_deflateInit(&s, Z_BEST_COMPRESSION);
	if (r != Z_OK) {
		einkfb_print_error ("Error: zlib_deflateInit() returned %d\n", r);
		return (-1);
	}
	s.next_in = src;
	s.avail_in = *lenp;
	s.next_out = dst + i;
	s.avail_out = dstlen - i;
	r = zlib_deflate(&s, Z_FINISH);
	if (r != Z_OK && r != Z_STREAM_END) {
		einkfb_print_error ("Error: zlib_deflate() returned %d\n", r);
		return (-1);
	}

	/* write header 2 (fill in deflate's header with gzip's)*/
	dst[8] = BEST_COMP;
	dst[9] = UNIX_OS;
	
	/* write trailer (replace adler32 with crc32 & length) */
	i = s.next_out - (unsigned char *) dst - 4;
	s.adler = crc32(src, *lenp);
	
	dst[i++] = (unsigned char)(s.adler & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 8) & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 16) & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 24) & 0xff);
	dst[i++] = (unsigned char)(s.total_in & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 8) & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 16) & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 24) & 0xff);

	*lenp = i;
	zlib_deflateEnd(&s);
	
	return (0);
}

unsigned char *einkfb_shim_alloc_kernelbuffer(unsigned long size, struct einkfb_info *info)
{
	unsigned char *result = NULL;
	
	if ( info->dma )
	{
		result = EINKFB_MALLOC_MOD(size, &kernelbuffer_phys);
		
		if ( result )
		einkfb_memset(result, EINKFB_WHITE, size);
	}
	
	return ( result );
}

void einkfb_shim_free_kernelbuffer(unsigned char *buffer, struct einkfb_info *info)
{
	if ( info->dma )
		EINKFB_FREE_MOD(buffer, &kernelbuffer_phys);
}

einkfb_dma_addr_t *einkfb_shim_get_kernelbuffer_phys(void)
{
	return ( &kernelbuffer_phys );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Power
	#pragma mark -
#endif

static int einkfb_shim_suspend_resume_hook(bool which)
{
	int result = EINKFB_FAILURE;
	
	switch ( which )
	{
		case EINKFB_SUSPEND:
			result = EINKFB_BLANK(FB_BLANK_HSYNC_SUSPEND);
		break;
		
		case EINKFB_RESUME:
			result = EINKFB_BLANK(FB_BLANK_UNBLANK);
		break;
	}
	
	return ( result );
}

static ssize_t store_test_suspend_resume(FB_DSTOR_PARAMS)
{
	int result = -EINVAL, suspend_resume;
	
	if ( sscanf(buf, "%d", &suspend_resume) )
	{
		if ( suspend_resume )
			einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
		else
			einkfb_shim_suspend_resume_hook(EINKFB_RESUME);
		
		result = count;
	}

	return ( result );
}

void einkfb_shim_sleep_screen(unsigned int cmd, splash_screen_type which_screen)
{
	// We may already be sleeping, so, to put up this screen during sleep,
	// we need override the current power level.
	//
	// Because we may need to wake up the screen in this case, whatever 
	// is in the framebuffer before we make this call might show up on
	// on the screen.  To prevent that, we clear whatever is in the
	// in all of the buffers.
	//
	// So, we may momentarily wake to a blank screen before the splash
	// screen itself comes up.
	//
	if ( FBIO_EINK_SPLASH_SCREEN_SLEEP == cmd )
	{
		CLEAR_BUFFERS_FOR_WAKE();
		POWER_OVERRIDE_BEGIN();
	}
	
	splash_screen_dispatch(which_screen);
	
	// Ensure that we're back to sleep when requested.
	//
	if ( FBIO_EINK_SPLASH_SCREEN_SLEEP == cmd )
	{	
		einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
		POWER_OVERRIDE_END();
	}
}

void einkfb_shim_power_op_complete(void)
{
}

void einkfb_shim_power_off_screen(void)
{
	POWER_OVERRIDE_BEGIN();
	CLEAR_SCREEN();
	
	einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
	POWER_OVERRIDE_END();
}

bool einkfb_shim_override_power_lockout(unsigned int cmd, unsigned long flag)
{
	return ( false );
}

bool einkfb_shim_enforce_power_lockout(void)
{
	return ( false );
}

char *einkfb_shim_get_power_string(void)
{
	return ( "not applicable" );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Splash Screen
	#pragma mark -
#endif

bool einkfb_shim_platform_splash_screen_dispatch(splash_screen_type which_screen, int yres)
{
	system_screen_t system_screen = INIT_SYSTEM_SCREEN_T();
	bool handled = true;
	
	switch ( which_screen )
	{
		case splash_screen_usb_recovery_util:
			system_screen.picture_header_len = PICTURE_USB_RECOVERY_UTIL_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_USB_RECOVERY_UTIL_HEADER(yres);
			system_screen.header_offset = USB_RECOVERY_UTIL_OFFSET_HEADER(yres);
			system_screen.header_width = USB_RECOVERY_UTIL_WIDTH_HEADER(yres);
									
			system_screen.picture_footer_len = PICTURE_USB_RECOVERY_UTIL_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_USB_RECOVERY_UTIL_FOOTER(yres);
			system_screen.footer_offset = USB_RECOVERY_UTIL_OFFSET_FOOTER(yres);
			system_screen.footer_width = USB_RECOVERY_UTIL_WIDTH_FOOTER(yres);

			system_screen.picture_body_len = PICTURE_USB_RECOVERY_UTIL_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_USB_RECOVERY_UTIL_BODY(yres);
			system_screen.body_offset = USB_RECOVERY_UTIL_OFFSET_BODY(yres);
			system_screen.body_width = USB_RECOVERY_UTIL_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
				
		case splash_screen_lowbatt:
			system_screen.picture_header_len = PICTURE_LOWBATT_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_LOWBATT_HEADER(yres);
			system_screen.header_offset = LOWBATT_OFFSET_HEADER(yres);
			system_screen.header_width = LOWBATT_WIDTH_HEADER(yres);
			
			system_screen.picture_footer_len = PICTURE_LOWBATT_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_LOWBATT_FOOTER(yres);
			system_screen.footer_offset = LOWBATT_OFFSET_FOOTER(yres);
			system_screen.footer_width = LOWBATT_WIDTH_FOOTER(yres);
			
			system_screen.picture_body_len = PICTURE_LOWBATT_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_LOWBATT_BODY(yres);
			system_screen.body_offset = LOWBATT_OFFSET_BODY(yres);
			system_screen.body_width = LOWBATT_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
		
		case splash_screen_repair_needed:
			system_screen.picture_header_len = PICTURE_REPAIR_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_REPAIR_HEADER(yres);
			system_screen.header_offset = REPAIR_OFFSET_HEADER(yres);
			system_screen.header_width = REPAIR_WIDTH_HEADER(yres);
			
			system_screen.picture_footer_len = PICTURE_REPAIR_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_REPAIR_FOOTER(yres);
			system_screen.footer_offset = REPAIR_OFFSET_FOOTER(yres);
			system_screen.footer_width = REPAIR_WIDTH_FOOTER(yres);
			
			system_screen.picture_body_len = PICTURE_REPAIR_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_REPAIR_BODY(yres);
			system_screen.body_offset = REPAIR_OFFSET_BODY(yres);
			system_screen.body_width = REPAIR_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
		
		default:
			handled = false;
		break;
	}

	return ( handled );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Main
	#pragma mark -
#endif

static DEVICE_ATTR(test_suspend_resume, DEVICE_MODE_W, NULL, store_test_suspend_resume);

void einkfb_shim_platform_init(struct einkfb_info *info)
{
	// Hook into the suspend/resume mechanism.
	//
	FB_DEVICE_CREATE_FILE(&info->dev->dev, &dev_attr_test_suspend_resume);
	einkfb_set_suspend_resume_hook(einkfb_shim_suspend_resume_hook);
 
	// Say that we should put up a splash screen on reboot.
	//
	EINKFB_IOCTL(FBIO_EINK_SET_REBOOT_BEHAVIOR, reboot_screen_splash);
	
	// Bring up the logo if we should.
	//
	splash_screen_up = splash_screen_invalid;
	
	if ( show_logo )
	{
		// But, first, get the screen clear if it hasn't been cleared already.
		//
		if ( !info->init )
			CLEAR_SCREEN();
		
		splash_screen_dispatch(splash_screen_boot);
	}
}

void einkfb_shim_platform_done(struct einkfb_info *info)
{
	// Remove the hook into the suspend/resume mechanism.
	//
	device_remove_file(&info->dev->dev, &dev_attr_test_suspend_resume);
	einkfb_set_suspend_resume_hook(NULL);
	
	// Say that we're done with the zlib workspaces.
	//
	done_z_inflate_workspace();
	done_z_deflate_workspace();
}
