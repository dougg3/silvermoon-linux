/*
 *  linux/arch/arm/mach-mmp/silvermoon.c
 *
 *  Support for the Chumby Silvermoon platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/gpio-pxa.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/smc91x.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/mmc/sdhci.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/irqs.h>
#include <video/pxa168fb.h>
#include <linux/input.h>

#include "common.h"

static unsigned long common_pin_config[] __initdata = {

	/* MMC3 (internal SD card) */
	GPIO0_MMC3_DAT7,
	GPIO1_MMC3_DAT6,
	GPIO2_MMC3_DAT5,
	GPIO3_MMC3_DAT4,
	GPIO4_MMC3_DAT3,
	GPIO5_MMC3_DAT2,
	GPIO6_MMC3_DAT1,
	GPIO7_MMC3_DAT0,
	GPIO8_MMC3_CLK,
	GPIO9_MMC3_CMD,
	GPIO16_SMC_nCS0_DIS,

	/* Static Memory Controller */
	GPIO18_SMC_nCS0,
	GPIO34_SMC_nCS1,
	GPIO23_SMC_nLUA,
	GPIO25_SMC_nLLA,
	GPIO28_SMC_RDY,
	GPIO29_SMC_SCLK,
	GPIO35_SMC_BE1,
	GPIO36_SMC_BE2,
	GPIO27_GPIO,	/* Ethernet IRQ */

	/* LCD */
	MFP_CFG_DRV_PULL(GPIO56, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO57, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO58, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO59, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO60, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO61, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO62, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO63, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO64, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO65, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO66, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO67, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO68, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO69, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO70, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO71, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO72, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO73, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO74, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO75, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO76, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO77, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO78, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO79, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO80, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO81, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO82, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO83, AF1, FAST, LOW),

	/* USB OTG VBUS? undocumented in vendor kernel */
	MFP_CFG(GPIO85, AF0),

	/* I2C */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

	/* Ensure UART1 input works? */
	MFP_CFG(GPIO109, AF0),

	/* SSP0 -- I2S */
	GPIO113_I2S_MCLK,
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_RXD,
	GPIO117_I2S_TXD,

	/* MMC2 (Wifi) */
	GPIO90_MMC2_DAT3,
	GPIO91_MMC2_DAT2,
	GPIO92_MMC2_DAT1,
	GPIO93_MMC2_DAT0,
	GPIO94_MMC2_CMD,
	GPIO95_MMC2_CLK,

	/* LCD power - switches to AF2 when in use! */
	MFP_CFG_DRV(GPIO84, AF0, SLOW),

	/* Touchscreen */
	MFP_CFG(GPIO118, AF1),
	MFP_CFG(GPIO119, AF1),
	MFP_CFG(GPIO120, AF1),
	MFP_CFG(GPIO121, AF1),

	/* Top button */
	MFP_CFG(GPIO89, AF0),

	/* Headphone jack detect */
	MFP_CFG(GPIO97, AF0),

	/* Cryptoprocessor UART */
	MFP_CFG(GPIO98, AF2),
	MFP_CFG(GPIO99, AF2),

	/* Media insert detect */
	MFP_CFG(GPIO100, AF0), /* SD */
	MFP_CFG(GPIO101, AF0), /* XD */
	MFP_CFG(GPIO102, AF0), /* Memory Stick */
	MFP_CFG(GPIO103, AF0), /* Compact Flash */
};

static struct pxa_gpio_platform_data silvermoon_pxa_gpio_data = {
	.irq_base	= MMP_GPIO_TO_IRQ(0),
};

/*static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock	= 25641,
		.refresh	= 60,
		.xres		= 800,
		.yres		= 600,
		.hsync_len	= 128,
		.left_margin	= 215,
		.right_margin	= 40,
		.vsync_len	= 4,
		.upper_margin	= 34,
		.lower_margin	= 14,
		.sync		= FB_SYNC_HOR_HIGH_ACT,
	},
};*/

static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock	= 25132,
		.refresh	= 60,
		.xres		= 800,
		.yres		= 600,
		.hsync_len	= 128,
		.left_margin	= 88,
		.right_margin	= 40,
		.vsync_len	= 3,
		.upper_margin	= 24,
		.lower_margin	= 1,
		/*.sync		= FB_SYNC_HOR_HIGH_ACT,*/
	},
};

struct pxa168fb_mach_info chumby8_lcd_info = {
	.id			= "Base-chumby",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGBA888,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO/* | 0x20U*/, /* No crossing 1K boundary */
	.dumb_mode		= DUMB_MODE_RGB666,

	.gpio_output_data = 0x10, /* LCD_SPU_DUMB_CTRL[27:20] */
	.gpio_output_mask = 0xff, /* LCD_SPU_DUMB_CTRL[19:12] */

	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 1,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
};

static struct i2c_board_info chumby8_i2c1_info[] __initdata = {
};

static struct i2c_board_info chumby8_i2c2_info[] __initdata = {
	{ I2C_BOARD_INFO("wm8961", 0x4a), },
};

static struct sdhci_pxa_platdata chumby8_sdh_platdata = {
	.quirks = SDHCI_QUIRK_NO_HISPD_BIT | SDHCI_QUIRK_NO_BUSY_IRQ | SDHCI_QUIRK_32BIT_DMA_SIZE,
	.flags = PXA_FLAG_CARD_PERMANENT,
	.clk_delay_cycles = 0x1f,
	.delay_in_ms = 5,
	.pxav1_controller = 1,
};

#if defined(CONFIG_USB_EHCI_MV)
static struct mv_usb_platform_data pxa168_sph_pdata = {
	.mode		= MV_USB_MODE_HOST,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
	.set_vbus	= NULL,
};
#endif

/* Chosen to not conflict with SW_HEADPHONE_INSERT = 0x02 */
#define SW_SILVERMOON_SD_CARD	12
#define SW_SILVERMOON_XD_CARD	13
#define SW_SILVERMOON_MS_CARD	14
#define SW_SILVERMOON_CF_CARD	15

static struct gpio_keys_button silvermoon_buttons[] = {
	{
		.type = EV_KEY,
		.code = BTN_TOP,
		.gpio = 89,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 10,
		.desc = "Chumby button",
	},

	{
		.type = EV_SW,
		.code = SW_HEADPHONE_INSERT,
		.gpio = 97,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 200,
		.desc = "Headphone jack",
	},

	{
		.type = EV_SW,
		.code = SW_SILVERMOON_SD_CARD,
		.gpio = 100,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 200,
		.desc = "SD card present",
	},

	{
		.type = EV_SW,
		.code = SW_SILVERMOON_XD_CARD,
		.gpio = 101,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 200,
		.desc = "XD card present",
	},

	{
		.type = EV_SW,
		.code = SW_SILVERMOON_MS_CARD,
		.gpio = 102,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 200,
		.desc = "MS card present",
	},

	{
		.type = EV_SW,
		.code = SW_SILVERMOON_CF_CARD,
		.gpio = 103,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 200,
		.desc = "CF card present",
	},
};

static struct gpio_keys_platform_data silvermoon_key_data = {
	.buttons = silvermoon_buttons,
	.nbuttons = ARRAY_SIZE(silvermoon_buttons),
};

static struct platform_device silvermoon_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &silvermoon_key_data,
	},
};

static struct platform_device *silvermoon_devices[] = {
	&pxa168_device_gpio,
	&silvermoon_gpio_keys_device,
};

static void __init silvermoon_init(void)
{
	mfp_config(ARRAY_AND_SIZE(common_pin_config));

	/* Set up IO stuff */
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_1P8V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	/* on-chip devices */
	pxa168_add_uart(1);
	pxa168_add_uart(3);

	pxa168_add_twsi(0, NULL, ARRAY_AND_SIZE(chumby8_i2c1_info));
	pxa168_add_twsi(1, NULL, ARRAY_AND_SIZE(chumby8_i2c2_info));

	pxa168_add_sdh(0, &chumby8_sdh_platdata);
	pxa168_add_sdh(1, &chumby8_sdh_platdata);
	pxa168_add_sdh(2, &chumby8_sdh_platdata);
	pxa168_add_fb(&chumby8_lcd_info);

	platform_device_add_data(&pxa168_device_gpio,
			&silvermoon_pxa_gpio_data,
			sizeof(silvermoon_pxa_gpio_data));

	platform_add_devices(ARRAY_AND_SIZE(silvermoon_devices));

#if defined(CONFIG_USB_EHCI_MV)
	pxa168_add_usb_host(&pxa168_sph_pdata);
#endif

	/* Ensure UART will work */
	if (gpio_request(109, "GPIO109")) {
		printk(KERN_ERR "GPIO109 request failed\n");
	} else {
		gpio_direction_output(109, 1);
		gpio_set_value(109, 0);
		gpio_free(109);
	}
}

MACHINE_START(SILVERMOON, "Chumby Silvermoon platform")
	.map_io		= mmp_map_io,
	.nr_irqs	= MMP_NR_IRQS,
	.init_irq	= pxa168_init_irq,
	.init_time	= pxa168_timer_init,
	.init_machine	= silvermoon_init,
	.restart	= pxa168_restart,
MACHINE_END

