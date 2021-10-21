/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup GPIO_MX35 Board GPIO and Muxing Setup
 * @ingroup MSL_MX35
 */
/*!
 * @file mach-mx35/iomux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX35
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include "iomux.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>
#endif

/*!
 * IOMUX register (base) addresses
 */
enum iomux_reg_addr {
	IOMUXGPR = IO_ADDRESS(IOMUXC_BASE_ADDR),
	/*!< General purpose */
	IOMUXSW_MUX_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR) + 4,
	/*!< MUX control */
	IOMUXSW_MUX_END = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x324,
	/*!< last MUX control register */
	IOMUXSW_PAD_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x328,
	/*!< Pad control */
	IOMUXSW_PAD_END = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x790,
	/*!< last Pad control register */
	IOMUXSW_INPUT_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x7A8,
	/*!< input select register */
	IOMUXSW_INPUT_END = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x9F4,
	/*!< last input select register */
};

#define MUX_PIN_NUM_MAX		\
		(((IOMUXSW_PAD_END - IOMUXSW_PAD_CTL) >> 2) + 1)
#define MUX_INPUT_NUM_MUX	\
		(((IOMUXSW_INPUT_END - IOMUXSW_INPUT_CTL) >> 2) + 1)

#define PIN_TO_IOMUX_INDEX(pin) ((PIN_TO_IOMUX_PAD(pin) - 0x328) >> 2)

static DEFINE_RAW_SPINLOCK(gpio_mux_lock);
static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];

#ifdef CONFIG_DEBUG_FS

struct mx35_pins {
	char *name;
	int value;
};

#define MX35_MAX_PINS	223	/* Total number of pins on MX35 */

#endif

/*!
 * This function is used to configure a pin through the IOMUX module.
 * FIXED ME: for backward compatible. Will be static function!
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  cfg		an output function as defined in \b #iomux_pin_cfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
static int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_cfg_t cfg)
{
	u32 ret = 0;
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u32 mux_reg = PIN_TO_IOMUX_MUX(pin);
	u8 *rp;

	BUG_ON(pin_index > MUX_PIN_NUM_MAX);
	if (mux_reg != NON_MUX_I) {
		mux_reg += IOMUXGPR;
		BUG_ON((mux_reg > IOMUXSW_MUX_END)
		       || (mux_reg < IOMUXSW_MUX_CTL));
		spin_lock(&gpio_mux_lock);
		__raw_writel(cfg, mux_reg);
		/*
		 * Log a warning if a pin changes ownership
		 */
		rp = iomux_pin_res_table + pin_index;
		if ((cfg & *rp) && (*rp != cfg)) {
			/*Console: how to do */
			printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
			       " config changed, index=%d register=%d, "
			       " prev=0x%x new=0x%x\n", pin_index, mux_reg,
			       *rp, cfg);
			ret = -EINVAL;
		}
		*rp = cfg;
		spin_unlock(&gpio_mux_lock);
	}

	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  cfg		an input function as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t cfg)
{
	int ret = iomux_config_mux(pin, cfg);
	if (GPIO_TO_PORT(IOMUX_TO_GPIO(pin)) < GPIO_PORT_NUM) {
		if (((cfg & (~MUX_CONFIG_SION)) == MUX_CONFIG_GPIO) ||
		    (((cfg & (~MUX_CONFIG_SION)) == MUX_CONFIG_FUNC) &&
		     ((pin == MX35_PIN_GPIO1_0) || (pin == MX35_PIN_GPIO1_1) ||
		      (pin == MX35_PIN_GPIO2_0) || (pin == MX35_PIN_GPIO3_0))))
			ret |= mxc_request_gpio(pin);
	}
	return ret;
}

EXPORT_SYMBOL(mxc_request_iomux);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  cfg		an input function as defined in \b #iomux_pin_cfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t cfg)
{
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u8 *rp = iomux_pin_res_table + pin_index;

	BUG_ON((pin_index > MUX_PIN_NUM_MAX));

	*rp = 0;
	if (GPIO_TO_PORT(IOMUX_TO_GPIO(pin)) < GPIO_PORT_NUM) {
		if (((cfg & (~MUX_CONFIG_SION)) == MUX_CONFIG_GPIO) ||
		    (((cfg & (~MUX_CONFIG_SION)) == MUX_CONFIG_FUNC) &&
		     ((pin == MX35_PIN_GPIO1_0) || (pin == MX35_PIN_GPIO1_1) ||
		      (pin == MX35_PIN_GPIO2_0) || (pin == MX35_PIN_GPIO3_0))))
			mxc_free_gpio(pin);
	}
}

EXPORT_SYMBOL(mxc_free_iomux);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin     a pin number as defined in \b #iomux_pin_name_t
 * @param  config  the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	u32 pad_reg = IOMUXGPR + PIN_TO_IOMUX_PAD(pin);

	BUG_ON((pad_reg > IOMUXSW_PAD_END) || (pad_reg < IOMUXSW_PAD_CTL));

	spin_lock(&gpio_mux_lock);
	__raw_writel(config, pad_reg);
	spin_unlock(&gpio_mux_lock);
}

EXPORT_SYMBOL(mxc_iomux_set_pad);

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en)
{
	u32 l;

	spin_lock(&gpio_mux_lock);
	l = __raw_readl(IOMUXGPR);

	if (en)
		l |= gp;
	else
		l &= ~gp;

	__raw_writel(l, IOMUXGPR);
	spin_unlock(&gpio_mux_lock);
}

EXPORT_SYMBOL(mxc_iomux_set_gpr);

/*!
 * This function configures input path.
 *
 * @param input index of input select register as defined in \b
 *  			#iomux_input_select_t
 * @param config the binary value of elements defined in \b
 * 			#iomux_input_config_t
 */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config)
{
	u32 reg = IOMUXSW_INPUT_CTL + (input << 2);

	BUG_ON(input >= MUX_INPUT_NUM_MUX);

	spin_lock(&gpio_mux_lock);
	__raw_writel(config, reg);
	spin_unlock(&gpio_mux_lock);
}

EXPORT_SYMBOL(mxc_iomux_set_input);

#ifdef CONFIG_DEBUG_FS

static struct mx35_pins mx35_luigi_pins[] = {
        { "MX35_PIN_CAPTURE", _MXC_BUILD_GPIO_PIN(0, 4, 0x4, 0x328) },
        { "MX35_PIN_COMPARE", _MXC_BUILD_GPIO_PIN(0, 5, 0x8, 0x32C) },
        { "MX35_PIN_WATCHDOG_RST", _MXC_BUILD_GPIO_PIN(0, 6, 0xC, 0x330) },
        { "MX35_PIN_GPIO1_0", _MXC_BUILD_GPIO_PIN(0, 0, 0x10, 0x334) },
        { "MX35_PIN_GPIO1_1", _MXC_BUILD_GPIO_PIN(0, 1, 0x14, 0x338) },
        { "MX35_PIN_GPIO2_0", _MXC_BUILD_GPIO_PIN(1, 0, 0x18, 0x33C) },
        { "MX35_PIN_GPIO3_0", _MXC_BUILD_GPIO_PIN(2, 1, 0x1C, 0x340) },
        { "MX35_PIN_CLKO", _MXC_BUILD_GPIO_PIN(0, 8, 0x20, 0x34C) },
        { "MX35_PIN_POWER_FAIL", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x360) },
        { "MX35_PIN_VSTBY", _MXC_BUILD_GPIO_PIN(0, 7, 0x24, 0x364) },
        { "MX35_PIN_A0", _MXC_BUILD_NON_GPIO_PIN(0x28, 0x368) },
        { "MX35_PIN_A1", _MXC_BUILD_NON_GPIO_PIN(0x2C, 0x36C) },
        { "MX35_PIN_A2", _MXC_BUILD_NON_GPIO_PIN(0x30, 0x370) },
        { "MX35_PIN_A3", _MXC_BUILD_NON_GPIO_PIN(0x34, 0x374) },
        { "MX35_PIN_A4", _MXC_BUILD_NON_GPIO_PIN(0x38, 0x378) },
        { "MX35_PIN_A5", _MXC_BUILD_NON_GPIO_PIN(0x3C, 0x37C) },
        { "MX35_PIN_A6", _MXC_BUILD_NON_GPIO_PIN(0x40, 0x380) },
        { "MX35_PIN_A7", _MXC_BUILD_NON_GPIO_PIN(0x44, 0x384) },
        { "MX35_PIN_A8", _MXC_BUILD_NON_GPIO_PIN(0x48, 0x388) },
        { "MX35_PIN_A9", _MXC_BUILD_NON_GPIO_PIN(0x4C, 0x38C) },
        { "MX35_PIN_A10", _MXC_BUILD_NON_GPIO_PIN(0x50, 0x390) },
        { "MX35_PIN_MA10", _MXC_BUILD_NON_GPIO_PIN(0x54, 0x394) },
        { "MX35_PIN_A11", _MXC_BUILD_NON_GPIO_PIN(0x58, 0x398) },
        { "MX35_PIN_A12", _MXC_BUILD_NON_GPIO_PIN(0x5C, 0x39C) },
        { "MX35_PIN_A13", _MXC_BUILD_NON_GPIO_PIN(0x60, 0x3A0) },
        { "MX35_PIN_A14", _MXC_BUILD_NON_GPIO_PIN(0x64, 0x3A4) },
        { "MX35_PIN_A15", _MXC_BUILD_NON_GPIO_PIN(0x68, 0x3A8) },
        { "MX35_PIN_A16", _MXC_BUILD_NON_GPIO_PIN(0x6C, 0x3AC) },
        { "MX35_PIN_A17", _MXC_BUILD_NON_GPIO_PIN(0x70, 0x3B0) },
        { "MX35_PIN_A18", _MXC_BUILD_NON_GPIO_PIN(0x74, 0x3B4) },
        { "MX35_PIN_A19", _MXC_BUILD_NON_GPIO_PIN(0x78, 0x3B8) },
        { "MX35_PIN_A20", _MXC_BUILD_NON_GPIO_PIN(0x7C, 0x3BC) },
        { "MX35_PIN_A21", _MXC_BUILD_NON_GPIO_PIN(0x80, 0x3C0) },
        { "MX35_PIN_A22", _MXC_BUILD_NON_GPIO_PIN(0x84, 0x3C4) },
        { "MX35_PIN_A23", _MXC_BUILD_NON_GPIO_PIN(0x88, 0x3C8) },
        { "MX35_PIN_A24", _MXC_BUILD_NON_GPIO_PIN(0x8C, 0x3CC) },
        { "MX35_PIN_A25", _MXC_BUILD_NON_GPIO_PIN(0x90, 0x3D0) },
        { "MX35_PIN_EB0", _MXC_BUILD_NON_GPIO_PIN(0x94, 0x46C) },
        { "MX35_PIN_EB1", _MXC_BUILD_NON_GPIO_PIN(0x98, 0x470) },
        { "MX35_PIN_OE", _MXC_BUILD_NON_GPIO_PIN(0x9C, 0x474) },
        { "MX35_PIN_CS0", _MXC_BUILD_NON_GPIO_PIN(0xA0, 0x478) },
        { "MX35_PIN_CS1", _MXC_BUILD_NON_GPIO_PIN(0xA4, 0x47C) },
        { "MX35_PIN_CS2", _MXC_BUILD_NON_GPIO_PIN(0xA8, 0x480) },
        { "MX35_PIN_CS3", _MXC_BUILD_NON_GPIO_PIN(0xAC, 0x484) },
        { "MX35_PIN_CS4", _MXC_BUILD_GPIO_PIN(0, 20, 0xB0, 0x488) },
        { "MX35_PIN_CS5", _MXC_BUILD_GPIO_PIN(0, 21, 0xB4, 0x48C) },
        { "MX35_PIN_NFCE_B", _MXC_BUILD_GPIO_PIN(0, 22, 0xB8, 0x490) },
        { "MX35_PIN_LBA", _MXC_BUILD_NON_GPIO_PIN(0xBC, 0x498) },
        { "MX35_PIN_BCLK", _MXC_BUILD_NON_GPIO_PIN(0xC0, 0x49C) },
        { "MX35_PIN_RW", _MXC_BUILD_NON_GPIO_PIN(0xC4, 0x4A0) },
        { "MX35_PIN_RAS", _MXC_BUILD_NON_GPIO_PIN(0xC8, 0x4A4) },
        { "MX35_PIN_CAS", _MXC_BUILD_NON_GPIO_PIN(0xCC, 0x4A8) },
        { "MX35_PIN_SDWE", _MXC_BUILD_NON_GPIO_PIN(0xD0, 0x4AC) },
        { "MX35_PIN_SDCKE0", _MXC_BUILD_NON_GPIO_PIN(0xD4, 0x4B0) },
        { "MX35_PIN_SDCKE1", _MXC_BUILD_NON_GPIO_PIN(0xD8, 0x4B4) },
        { "MX35_PIN_NFWE_B", _MXC_BUILD_GPIO_PIN(0, 18, 0xC8, 0x4CC) },
        { "MX35_PIN_NFRE_B", _MXC_BUILD_GPIO_PIN(0, 19, 0xCC, 0x4D0) },
        { "MX35_PIN_NFALE", _MXC_BUILD_GPIO_PIN(0, 20, 0xD0, 0x4D4) },
        { "MX35_PIN_NFCLE", _MXC_BUILD_GPIO_PIN(0, 21, 0xD4, 0x4D8) },
        { "MX35_PIN_NFWP_B", _MXC_BUILD_GPIO_PIN(0, 22, 0xD8, 0x4DC) },
        { "MX35_PIN_NFRB", _MXC_BUILD_GPIO_PIN(0, 23, 0xDC, 0x4E0) },
        { "MX35_PIN_D15", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4E4) },
        { "MX35_PIN_D14", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4E8) },
        { "MX35_PIN_D13", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4EC) },
        { "MX35_PIN_D12", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4F0) },
        { "MX35_PIN_D11", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4F4) },
        { "MX35_PIN_D10", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4F8) },
        { "MX35_PIN_D9", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x4FC) },
        { "MX35_PIN_D8", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x500) },
        { "MX35_PIN_D7", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x504) },
        { "MX35_PIN_D6", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x508) },
        { "MX35_PIN_D5", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x50C) },
        { "MX35_PIN_D4", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x510) },
        { "MX35_PIN_D3", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x514) },
        { "MX35_PIN_D2", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x518) },
        { "MX35_PIN_D1", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x51C) },
        { "MX35_PIN_D0", _MXC_BUILD_NON_GPIO_PIN(NON_MUX_I, 0x520) },
        { "MX35_PIN_CSI_D8", _MXC_BUILD_GPIO_PIN(0, 20, 0xE0, 0x524) },
        { "MX35_PIN_CSI_D9", _MXC_BUILD_GPIO_PIN(0, 21, 0xE4, 0x528) },
        { "MX35_PIN_CSI_D10", _MXC_BUILD_GPIO_PIN(0, 22, 0xE8, 0x52C) },
        { "MX35_PIN_CSI_D11", _MXC_BUILD_GPIO_PIN(0, 23, 0xEC, 0x530) },
        { "MX35_PIN_CSI_D12", _MXC_BUILD_GPIO_PIN(0, 24, 0xF0, 0x534) },
        { "MX35_PIN_CSI_D13", _MXC_BUILD_GPIO_PIN(0, 25, 0xF4, 0x538) },
        { "MX35_PIN_CSI_D14", _MXC_BUILD_GPIO_PIN(0, 26, 0xF8, 0x53C) },
        { "MX35_PIN_CSI_D15", _MXC_BUILD_GPIO_PIN(0, 27, 0xFC, 0x540) },
        { "MX35_PIN_CSI_MCLK", _MXC_BUILD_GPIO_PIN(0, 28, 0x100, 0x544) },
        { "MX35_PIN_CSI_VSYNC", _MXC_BUILD_GPIO_PIN(0, 29, 0x104, 0x548) },
        { "MX35_PIN_CSI_HSYNC", _MXC_BUILD_GPIO_PIN(0, 30, 0x108, 0x54C) },
        { "MX35_PIN_CSI_PIXCLK", _MXC_BUILD_GPIO_PIN(0, 31, 0x10C, 0x550) },
        { "MX35_PIN_I2C1_CLK", _MXC_BUILD_GPIO_PIN(1, 24, 0x110, 0x554) },
        { "MX35_PIN_I2C1_DAT", _MXC_BUILD_GPIO_PIN(1, 25, 0x114, 0x558) },
        { "MX35_PIN_I2C2_CLK", _MXC_BUILD_GPIO_PIN(1, 26, 0x118, 0x55C) },
        { "MX35_PIN_I2C2_DAT", _MXC_BUILD_GPIO_PIN(1, 27, 0x11C, 0x560) },
        { "MX35_PIN_STXD4", _MXC_BUILD_GPIO_PIN(1, 28, 0x120, 0x564) },
        { "MX35_PIN_SRXD4", _MXC_BUILD_GPIO_PIN(1, 29, 0x124, 0x568) },
        { "MX35_PIN_SCK4", _MXC_BUILD_GPIO_PIN(1, 30, 0x128, 0x56C) },
        { "MX35_PIN_STXFS4", _MXC_BUILD_GPIO_PIN(1, 31, 0x12C, 0x570) },
        { "MX35_PIN_STXD5", _MXC_BUILD_GPIO_PIN(0, 0, 0x130, 0x574) },
        { "MX35_PIN_SRXD5", _MXC_BUILD_GPIO_PIN(0, 1, 0x134, 0x578) },
        { "MX35_PIN_SCK5", _MXC_BUILD_GPIO_PIN(0, 2, 0x138, 0x57C) },
        { "MX35_PIN_STXFS5", _MXC_BUILD_GPIO_PIN(0, 3, 0x13C, 0x580) },
        { "MX35_PIN_SCKR", _MXC_BUILD_GPIO_PIN(0, 4, 0x140, 0x584) },
        { "MX35_PIN_FSR", _MXC_BUILD_GPIO_PIN(0, 5, 0x144, 0x588) },
        { "MX35_PIN_HCKR", _MXC_BUILD_GPIO_PIN(0, 6, 0x148, 0x58C) },
        { "MX35_PIN_SCKT", _MXC_BUILD_GPIO_PIN(0, 7, 0x14C, 0x590) },
        { "MX35_PIN_FST", _MXC_BUILD_GPIO_PIN(0, 8, 0x150, 0x594) },
        { "MX35_PIN_HCKT", _MXC_BUILD_GPIO_PIN(0, 9, 0x154, 0x598) },
        { "MX35_PIN_TX5_RX0", _MXC_BUILD_GPIO_PIN(0, 10, 0x158, 0x59C) },
        { "MX35_PIN_TX4_RX1", _MXC_BUILD_GPIO_PIN(0, 11, 0x15C, 0x5A0) },
        { "MX35_PIN_TX3_RX2", _MXC_BUILD_GPIO_PIN(0, 12, 0x160, 0x5A4) },
        { "MX35_PIN_TX2_RX3", _MXC_BUILD_GPIO_PIN(0, 13, 0x164, 0x5A8) },
        { "MX35_PIN_TX1", _MXC_BUILD_GPIO_PIN(0, 14, 0x168, 0x5AC) },
        { "MX35_PIN_TX0", _MXC_BUILD_GPIO_PIN(0, 15, 0x16C, 0x5B0) },
        { "MX35_PIN_CSPI1_MOSI", _MXC_BUILD_GPIO_PIN(0, 16, 0x170, 0x5B4) },
        { "MX35_PIN_CSPI1_MISO", _MXC_BUILD_GPIO_PIN(0, 17, 0x174, 0x5B8) },
        { "MX35_PIN_CSPI1_SS0", _MXC_BUILD_GPIO_PIN(0, 18, 0x178, 0x5BC) },
        { "MX35_PIN_CSPI1_SS1", _MXC_BUILD_GPIO_PIN(0, 19, 0x17C, 0x5C0) },
        { "MX35_PIN_CSPI1_SCLK", _MXC_BUILD_GPIO_PIN(2, 4, 0x180, 0x5C4) },
        { "MX35_PIN_CSPI1_SPI_RDY", _MXC_BUILD_GPIO_PIN(2, 5, 0x184, 0x5C8) },
        { "MX35_PIN_RXD1", _MXC_BUILD_GPIO_PIN(2, 6, 0x188, 0x5CC) },
        { "MX35_PIN_TXD1", _MXC_BUILD_GPIO_PIN(2, 7, 0x18C, 0x5D0) },
        { "MX35_PIN_RTS1", _MXC_BUILD_GPIO_PIN(2, 8, 0x190, 0x5D4) },
        { "MX35_PIN_CTS1", _MXC_BUILD_GPIO_PIN(2, 9, 0x194, 0x5D8) },
        { "MX35_PIN_RXD2", _MXC_BUILD_GPIO_PIN(2, 10, 0x198, 0x5DC) },
        { "MX35_PIN_TXD2", _MXC_BUILD_GPIO_PIN(1, 11, 0x19C, 0x5E0) },
        { "MX35_PIN_RTS2", _MXC_BUILD_GPIO_PIN(1, 12, 0x1A0, 0x5E4) },
        { "MX35_PIN_CTS2", _MXC_BUILD_GPIO_PIN(1, 13, 0x1A4, 0x5E8) },
        { "MX35_PIN_USBOTG_PWR", _MXC_BUILD_GPIO_PIN(2, 14, 0x1A8, 0x60C) },
        { "MX35_PIN_USBOTG_OC", _MXC_BUILD_GPIO_PIN(2, 15, 0x1AC, 0x610) },
        { "MX35_PIN_LD0", _MXC_BUILD_GPIO_PIN(1, 0, 0x1B0, 0x614) },
        { "MX35_PIN_LD1", _MXC_BUILD_GPIO_PIN(1, 1, 0x1B4, 0x618) },
        { "MX35_PIN_LD2", _MXC_BUILD_GPIO_PIN(1, 2, 0x1B8, 0x61C) },
        { "MX35_PIN_LD3", _MXC_BUILD_GPIO_PIN(1, 3, 0x1BC, 0x620) },
        { "MX35_PIN_LD4", _MXC_BUILD_GPIO_PIN(1, 4, 0x1C0, 0x624) },
        { "MX35_PIN_LD5", _MXC_BUILD_GPIO_PIN(1, 5, 0x1C4, 0x628) },
        { "MX35_PIN_LD6", _MXC_BUILD_GPIO_PIN(1, 6, 0x1C8, 0x62C) },
        { "MX35_PIN_LD7", _MXC_BUILD_GPIO_PIN(1, 7, 0x1CC, 0x630) },
        { "MX35_PIN_LD8", _MXC_BUILD_GPIO_PIN(1, 8, 0x1D0, 0x634) },
        { "MX35_PIN_LD9", _MXC_BUILD_GPIO_PIN(1, 9, 0x1D4, 0x638) },
        { "MX35_PIN_LD10", _MXC_BUILD_GPIO_PIN(1, 10, 0x1D8, 0x63C) },
        { "MX35_PIN_LD11", _MXC_BUILD_GPIO_PIN(1, 11, 0x1DC, 0x640) },
        { "MX35_PIN_LD12", _MXC_BUILD_GPIO_PIN(1, 12, 0x1E0, 0x644) },
        { "MX35_PIN_LD13", _MXC_BUILD_GPIO_PIN(1, 13, 0x1E4, 0x648) },
        { "MX35_PIN_LD14", _MXC_BUILD_GPIO_PIN(1, 14, 0x1E8, 0x64C) },
        { "MX35_PIN_LD15", _MXC_BUILD_GPIO_PIN(1, 15, 0x1EC, 0x650) },
        { "MX35_PIN_LD16", _MXC_BUILD_GPIO_PIN(1, 16, 0x1F0, 0x654) },
        { "MX35_PIN_LD17", _MXC_BUILD_GPIO_PIN(1, 17, 0x1F4, 0x658) },
        { "MX35_PIN_LD18", _MXC_BUILD_GPIO_PIN(2, 24, 0x1F8, 0x65C) },
        { "MX35_PIN_LD19", _MXC_BUILD_GPIO_PIN(2, 25, 0x1FC, 0x660) },
        { "MX35_PIN_LD20", _MXC_BUILD_GPIO_PIN(2, 26, 0x200, 0x664) },
        { "MX35_PIN_LD21", _MXC_BUILD_GPIO_PIN(2, 27, 0x204, 0x668) },
        { "MX35_PIN_LD22", _MXC_BUILD_GPIO_PIN(2, 28, 0x208, 0x66C) },
        { "MX35_PIN_LD23", _MXC_BUILD_GPIO_PIN(2, 29, 0x20C, 0x670) },
        { "MX35_PIN_D3_HSYNC", _MXC_BUILD_GPIO_PIN(2, 30, 0x210, 0x674) },
        { "MX35_PIN_D3_FPSHIFT", _MXC_BUILD_GPIO_PIN(2, 31, 0x214, 0x678) },
        { "MX35_PIN_D3_DRDY", _MXC_BUILD_GPIO_PIN(0, 0, 0x218, 0x67C) },
        { "MX35_PIN_CONTRAST", _MXC_BUILD_GPIO_PIN(0, 1, 0x21C, 0x680) },
        { "MX35_PIN_D3_VSYNC", _MXC_BUILD_GPIO_PIN(0, 2, 0x220, 0x684) },
        { "MX35_PIN_D3_REV", _MXC_BUILD_GPIO_PIN(0, 3, 0x224, 0x688) },
        { "MX35_PIN_D3_CLS", _MXC_BUILD_GPIO_PIN(0, 4, 0x228, 0x68C) },
        { "MX35_PIN_D3_SPL", _MXC_BUILD_GPIO_PIN(0, 5, 0x22C, 0x690) },
        { "MX35_PIN_SD1_CMD", _MXC_BUILD_GPIO_PIN(0, 6, 0x230, 0x694) },
        { "MX35_PIN_SD1_CLK", _MXC_BUILD_GPIO_PIN(0, 7, 0x234, 0x698) },
        { "MX35_PIN_SD1_DATA0", _MXC_BUILD_GPIO_PIN(0, 8, 0x238, 0x69C) },
        { "MX35_PIN_SD1_DATA1", _MXC_BUILD_GPIO_PIN(0, 9, 0x23C, 0x6A0) },
        { "MX35_PIN_SD1_DATA2", _MXC_BUILD_GPIO_PIN(0, 10, 0x240, 0x6A4) },
        { "MX35_PIN_SD1_DATA3", _MXC_BUILD_GPIO_PIN(0, 11, 0x244, 0x6A8) },
        { "MX35_PIN_SD2_CMD", _MXC_BUILD_GPIO_PIN(1, 0, 0x248, 0x6AC) },
        { "MX35_PIN_SD2_CLK", _MXC_BUILD_GPIO_PIN(1, 1, 0x24C, 0x6B0) },
        { "MX35_PIN_SD2_DATA0", _MXC_BUILD_GPIO_PIN(1, 2, 0x250, 0x6B4) },
        { "MX35_PIN_SD2_DATA1", _MXC_BUILD_GPIO_PIN(1, 3, 0x254, 0x6B8) },
        { "MX35_PIN_SD2_DATA2", _MXC_BUILD_GPIO_PIN(1, 4, 0x258, 0x6BC) },
        { "MX35_PIN_SD2_DATA3", _MXC_BUILD_GPIO_PIN(1, 5, 0x25C, 0x6C0) },
        { "MX35_PIN_ATA_CS0", _MXC_BUILD_GPIO_PIN(1, 6, 0x260, 0x6C4) },
        { "MX35_PIN_ATA_CS1", _MXC_BUILD_GPIO_PIN(1, 7, 0x264, 0x6C8) },
        { "MX35_PIN_ATA_DIOR", _MXC_BUILD_GPIO_PIN(1, 8, 0x268, 0x6CC) },
        { "MX35_PIN_ATA_DIOW", _MXC_BUILD_GPIO_PIN(1, 9, 0x26C, 0x6D0) },
        { "MX35_PIN_ATA_DMACK", _MXC_BUILD_GPIO_PIN(1, 10, 0x270, 0x6D4) },
        { "MX35_PIN_ATA_RESET_B", _MXC_BUILD_GPIO_PIN(1, 11, 0x274, 0x6D8) },
        { "MX35_PIN_ATA_IORDY", _MXC_BUILD_GPIO_PIN(1, 12, 0x278, 0x6DC) },
        { "MX35_PIN_ATA_DATA0", _MXC_BUILD_GPIO_PIN(1, 13, 0x27C, 0x6E0) },
        { "MX35_PIN_ATA_DATA1", _MXC_BUILD_GPIO_PIN(1, 14, 0x280, 0x6E4) },
        { "MX35_PIN_ATA_DATA2", _MXC_BUILD_GPIO_PIN(1, 15, 0x284, 0x6E8) },
        { "MX35_PIN_ATA_DATA3", _MXC_BUILD_GPIO_PIN(1, 16, 0x288, 0x6EC) },
        { "MX35_PIN_ATA_DATA4", _MXC_BUILD_GPIO_PIN(1, 17, 0x28C, 0x6F0) },
        { "MX35_PIN_ATA_DATA5", _MXC_BUILD_GPIO_PIN(1, 18, 0x290, 0x6F4) },
        { "MX35_PIN_ATA_DATA6", _MXC_BUILD_GPIO_PIN(1, 19, 0x294, 0x6F8) },
        { "MX35_PIN_ATA_DATA7", _MXC_BUILD_GPIO_PIN(1, 20, 0x298, 0x6FC) },
        { "MX35_PIN_ATA_DATA8", _MXC_BUILD_GPIO_PIN(1, 21, 0x29C, 0x700) },
        { "MX35_PIN_ATA_DATA9", _MXC_BUILD_GPIO_PIN(1, 22, 0x2A0, 0x704) },
        { "MX35_PIN_ATA_DATA10", _MXC_BUILD_GPIO_PIN(1, 23, 0x2A4, 0x708) },
        { "MX35_PIN_ATA_DATA11", _MXC_BUILD_GPIO_PIN(1, 24, 0x2A8, 0x70C) },
        { "MX35_PIN_ATA_DATA12", _MXC_BUILD_GPIO_PIN(1, 25, 0x2AC, 0x710) },
        { "MX35_PIN_ATA_DATA13", _MXC_BUILD_GPIO_PIN(1, 26, 0x2B0, 0x714) },
        { "MX35_PIN_ATA_DATA14", _MXC_BUILD_GPIO_PIN(1, 27, 0x2B4, 0x718) },
        { "MX35_PIN_ATA_DATA15", _MXC_BUILD_GPIO_PIN(1, 28, 0x2B8, 0x71C) },
        { "MX35_PIN_ATA_INTRQ", _MXC_BUILD_GPIO_PIN(1, 29, 0x2BC, 0x720) },
        { "MX35_PIN_ATA_BUFF_EN", _MXC_BUILD_GPIO_PIN(1, 30, 0x2C0, 0x724) },
        { "MX35_PIN_ATA_DMARQ", _MXC_BUILD_GPIO_PIN(1, 31, 0x2C4, 0x728) },
        { "MX35_PIN_ATA_DA0", _MXC_BUILD_GPIO_PIN(2, 0, 0x2C8, 0x72C) },
        { "MX35_PIN_ATA_DA1", _MXC_BUILD_GPIO_PIN(2, 1, 0x2CC, 0x730) },
        { "MX35_PIN_ATA_DA2", _MXC_BUILD_GPIO_PIN(2, 2, 0x2D0, 0x734) },
        { "MX35_PIN_MLB_CLK", _MXC_BUILD_GPIO_PIN(2, 3, 0x2D4, 0x738) },
        { "MX35_PIN_MLB_DAT", _MXC_BUILD_GPIO_PIN(2, 4, 0x2D8, 0x73C) },
        { "MX35_PIN_MLB_SIG", _MXC_BUILD_GPIO_PIN(2, 5, 0x2DC, 0x740) },
        { "MX35_PIN_FEC_TX_CLK", _MXC_BUILD_GPIO_PIN(2, 6, 0x2E0, 0x744) },
        { "MX35_PIN_FEC_RX_CLK", _MXC_BUILD_GPIO_PIN(2, 7, 0x2E4, 0x748) },
        { "MX35_PIN_FEC_RX_DV", _MXC_BUILD_GPIO_PIN(2, 8, 0x2E8, 0x74C) },
        { "MX35_PIN_FEC_COL", _MXC_BUILD_GPIO_PIN(2, 9, 0x2EC, 0x750) },
        { "MX35_PIN_FEC_RDATA0", _MXC_BUILD_GPIO_PIN(2, 10, 0x2F0, 0x754) },
        { "MX35_PIN_FEC_TDATA0", _MXC_BUILD_GPIO_PIN(2, 11, 0x2F4, 0x758) },
        { "MX35_PIN_FEC_TX_EN", _MXC_BUILD_GPIO_PIN(2, 12, 0x2F8, 0x75C) },
        { "MX35_PIN_FEC_MDC", _MXC_BUILD_GPIO_PIN(2, 13, 0x2FC, 0x760) },
        { "MX35_PIN_FEC_MDIO", _MXC_BUILD_GPIO_PIN(2, 14, 0x300, 0x764) },
        { "MX35_PIN_FEC_TX_ERR", _MXC_BUILD_GPIO_PIN(2, 15, 0x304, 0x768) },
        { "MX35_PIN_FEC_RX_ERR", _MXC_BUILD_GPIO_PIN(2, 16, 0x308, 0x76C) },
        { "MX35_PIN_FEC_CRS", _MXC_BUILD_GPIO_PIN(2, 17, 0x30C, 0x770) },
        { "MX35_PIN_FEC_RDATA1", _MXC_BUILD_GPIO_PIN(2, 18, 0x310, 0x774) },
        { "MX35_PIN_FEC_TDATA1", _MXC_BUILD_GPIO_PIN(2, 19, 0x314, 0x778) },
        { "MX35_PIN_FEC_RDATA2", _MXC_BUILD_GPIO_PIN(2, 20, 0x318, 0x77C) },
        { "MX35_PIN_FEC_TDATA2", _MXC_BUILD_GPIO_PIN(2, 21, 0x31C, 0x780) },
        { "MX35_PIN_FEC_RDATA3", _MXC_BUILD_GPIO_PIN(2, 22, 0x320, 0x784) },
        { "MX35_PIN_FEC_TDATA3", _MXC_BUILD_GPIO_PIN(2, 23, 0x324, 0x788) },
};

void mx35_show_pins(struct seq_file *s)
{
	int pin = 0, val = 0, t = 0;
	
	for (pin = 0; pin < MX35_MAX_PINS; pin++) {
		val = __raw_readl(IOMUXGPR + PIN_TO_IOMUX_PAD(mx35_luigi_pins[pin].value));
		t += seq_printf(s, "%s  Pad Value = 0x%x\n", mx35_luigi_pins[pin].name, val);
	}
	return;
}

EXPORT_SYMBOL(mx35_show_pins);

#endif /* CONFIG_DEBUG_FS */
