// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 Marvell International Ltd.
 *		Zhangfei Gao <zhangfei.gao@marvell.com>
 *		Kevin Wang <dwang4@marvell.com>
 *		Jun Nie <njun@marvell.com>
 *		Qiming Wu <wuqm@marvell.com>
 *		Philip Rakity <prakity@marvell.com>
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/mmc.h>
#include <linux/pinctrl/consumer.h>

#include "sdhci.h"
#include "sdhci-pltfm.h"

#define SD_FIFO_PARAM		0xe0
#define DIS_PAD_SD_CLK_GATE	0x0400 /* Turn on/off Dynamic SD Clock Gating */
#define CLK_GATE_ON		0x0200 /* Disable/enable Clock Gate */
#define CLK_GATE_CTL		0x0100 /* Clock Gate Control */
#define CLK_GATE_SETTING_BITS	(DIS_PAD_SD_CLK_GATE | \
		CLK_GATE_ON | CLK_GATE_CTL)

#define SD_CLOCK_BURST_SIZE_SETUP	0xe6
#define SDCLK_SEL_SHIFT		8
#define SDCLK_SEL_MASK		0x3
#define SDCLK_DELAY_SHIFT	10
#define SDCLK_DELAY_MASK	0x3c

#define SD_CE_ATA_2		0xea
#define MMC_CARD		0x1000
#define MMC_WIDTH		0x0100

struct sdhci_pxav2_host {
	struct clk *axi_clk; /* optional extra AXI clock to enable */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sdio_irq_fix; /* for SDIO errata on pxa168 */
};

static void pxav2_reset(struct sdhci_host *host, u8 mask)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	sdhci_reset(host, mask);

	if (mask == SDHCI_RESET_ALL) {
		u16 tmp = 0;

		/*
		 * tune timing of read data/command when crc error happen
		 * no performance impact
		 */
		if (pdata && pdata->clk_delay_sel == 1) {
			tmp = readw(host->ioaddr + SD_CLOCK_BURST_SIZE_SETUP);

			tmp &= ~(SDCLK_DELAY_MASK << SDCLK_DELAY_SHIFT);
			tmp |= (pdata->clk_delay_cycles & SDCLK_DELAY_MASK)
				<< SDCLK_DELAY_SHIFT;
			tmp &= ~(SDCLK_SEL_MASK << SDCLK_SEL_SHIFT);
			tmp |= (1 & SDCLK_SEL_MASK) << SDCLK_SEL_SHIFT;

			writew(tmp, host->ioaddr + SD_CLOCK_BURST_SIZE_SETUP);
		}

		if (pdata && (pdata->flags & PXA_FLAG_ENABLE_CLOCK_GATING)) {
			tmp = readw(host->ioaddr + SD_FIFO_PARAM);
			tmp &= ~CLK_GATE_SETTING_BITS;
			writew(tmp, host->ioaddr + SD_FIFO_PARAM);
		} else {
			tmp = readw(host->ioaddr + SD_FIFO_PARAM);
			tmp &= ~CLK_GATE_SETTING_BITS;
			tmp |= CLK_GATE_SETTING_BITS;
			writew(tmp, host->ioaddr + SD_FIFO_PARAM);
		}
	}
}

static inline u16 pxav1_readw(struct sdhci_host *host, int reg)
{
	u32 temp;
	/* Workaround for data abort exception on SDH2 and SDH4 on PXA168 */
	if (reg == SDHCI_HOST_VERSION) {
		temp = readl(host->ioaddr + SDHCI_HOST_VERSION - 2) >> 16;
		return temp & 0xffff;
	}

	return readw(host->ioaddr + reg);
}

static void (*old_post_req)(struct mmc_host *mmc, struct mmc_request *mrq, int err);

static void pxav1_post_req(struct mmc_host *mmc, struct mmc_request *mrq, int err)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pxav2_host *pxav2_host;
	struct mmc_command dummy_cmd = {};
	int dummy_err;
	u16 tmp;

	// If this is an SDIO command, we have to do fixups to ensure we receive card IRQs afterward
	if (!err && mrq->cmd && !mrq->cmd->error &&
	    (mrq->cmd->opcode == SD_IO_RW_DIRECT ||
	    mrq->cmd->opcode == SD_IO_RW_EXTENDED)) {
		// Reset data port
		tmp = readw(host->ioaddr + SDHCI_TIMEOUT_CONTROL);
		tmp |= 0x400;
		writew(tmp, host->ioaddr + SDHCI_TIMEOUT_CONTROL);

		// Now the clock will be stopped, so we need to restart the clock
		// by sending a dummy CMD0
		pxav2_host = sdhci_pltfm_priv(sdhci_priv(host));

		// Set as high output rather than MMC function while we do CMD0
		if (pxav2_host->pinctrl && pxav2_host->pins_sdio_irq_fix)
			pinctrl_select_state(pxav2_host->pinctrl, pxav2_host->pins_sdio_irq_fix);

		dummy_cmd.opcode = MMC_GO_IDLE_STATE;
		dummy_cmd.arg = 0;
		dummy_cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

		dummy_err = mmc_wait_for_cmd(host->mmc, &dummy_cmd, 0);
		if (dummy_err)
			printk("Error waiting: %d", err);

		// Set as MMC function after dummy command is complete
		if (pxav2_host->pinctrl && pxav2_host->pins_default)
			pinctrl_select_state(pxav2_host->pinctrl, pxav2_host->pins_default);
	}

	/* pass onto SDHCI host driver now */
	if (old_post_req)
		old_post_req(mmc, mrq, err);
}

static void pxav2_mmc_set_bus_width(struct sdhci_host *host, int width)
{
	u8 ctrl;
	u16 tmp;

	ctrl = readb(host->ioaddr + SDHCI_HOST_CONTROL);
	tmp = readw(host->ioaddr + SD_CE_ATA_2);
	if (width == MMC_BUS_WIDTH_8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		tmp |= MMC_CARD | MMC_WIDTH;
	} else {
		tmp &= ~(MMC_CARD | MMC_WIDTH);
		if (width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	writew(tmp, host->ioaddr + SD_CE_ATA_2);
	writeb(ctrl, host->ioaddr + SDHCI_HOST_CONTROL);
}

static const struct sdhci_ops pxav1_sdhci_ops = {
	.read_w        = pxav1_readw,
	.set_clock     = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = pxav2_mmc_set_bus_width,
	.reset         = pxav2_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_ops pxav2_sdhci_ops = {
	.set_clock     = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = pxav2_mmc_set_bus_width,
	.reset         = pxav2_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

#ifdef CONFIG_OF
static const struct of_device_id sdhci_pxav2_of_match[] = {
	{
		.compatible = "mrvl,pxav2-mmc",
	},
	{
		.compatible = "mrvl,pxav1-mmc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_pxav2_of_match);

static struct sdhci_pxa_platdata *pxav2_get_mmc_pdata(struct device *dev)
{
	struct sdhci_pxa_platdata *pdata;
	struct device_node *np = dev->of_node;
	u32 bus_width;
	u32 clk_delay_cycles;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_find_property(np, "non-removable", NULL))
		pdata->flags |= PXA_FLAG_CARD_PERMANENT;

	of_property_read_u32(np, "bus-width", &bus_width);
	if (bus_width == 8)
		pdata->flags |= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT;

	of_property_read_u32(np, "mrvl,clk-delay-cycles", &clk_delay_cycles);
	if (clk_delay_cycles > 0) {
		pdata->clk_delay_sel = 1;
		pdata->clk_delay_cycles = clk_delay_cycles;
	}

	return pdata;
}
#else
static inline struct sdhci_pxa_platdata *pxav2_get_mmc_pdata(struct device *dev)
{
	return NULL;
}
#endif

static int sdhci_pxav2_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_pxav2_host *pxav2_host;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = NULL;
	const struct of_device_id *match;

	int ret;
	struct clk *clk;

	host = sdhci_pltfm_init(pdev, NULL, sizeof(*pxav2_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	pxav2_host = sdhci_pltfm_priv(pltfm_host);

	clk = devm_clk_get(dev, "PXA-SDHCLK");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(clk);
		goto free;
	}
	pltfm_host->clk = clk;
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable io clock\n");
		goto free;
	}

	pxav2_host->axi_clk = devm_clk_get(dev, "PXA-SDHCLK-AXI");
	if (!IS_ERR(pxav2_host->axi_clk)) {
		ret = clk_prepare_enable(pxav2_host->axi_clk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable AXI clock\n");
			goto disable_clk;
		}
	}

	host->quirks = SDHCI_QUIRK_BROKEN_ADMA
		| SDHCI_QUIRK_BROKEN_TIMEOUT_VAL
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN;

	match = of_match_device(of_match_ptr(sdhci_pxav2_of_match), &pdev->dev);
	if (match) {
		pdata = pxav2_get_mmc_pdata(dev);
	}
	if (pdata) {
		if (pdata->flags & PXA_FLAG_CARD_PERMANENT) {
			/* on-chip device */
			host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
			host->mmc->caps |= MMC_CAP_NONREMOVABLE;
		}

		/* If slot design supports 8 bit data, indicate this to MMC. */
		if (pdata->flags & PXA_FLAG_SD_8_BIT_CAPABLE_SLOT)
			host->mmc->caps |= MMC_CAP_8_BIT_DATA;

		if (pdata->quirks)
			host->quirks |= pdata->quirks;
		if (pdata->host_caps)
			host->mmc->caps |= pdata->host_caps;
		if (pdata->pm_caps)
			host->mmc->pm_caps |= pdata->pm_caps;
	}

	if (match && of_device_is_compatible(dev->of_node, "mrvl,pxav1-mmc")) {
		host->quirks |= SDHCI_QUIRK_NO_BUSY_IRQ | SDHCI_QUIRK_NO_HISPD_BIT;
		host->ops = &pxav1_sdhci_ops;
		old_post_req = host->mmc_host_ops.post_req;
		host->mmc_host_ops.post_req = pxav1_post_req;

		/* set up optional pinctrl for PXA168 SDIO IRQ fix */
		pxav2_host->pinctrl = devm_pinctrl_get(&pdev->dev);
		if (!IS_ERR(pxav2_host->pinctrl)) {
			pxav2_host->pins_sdio_irq_fix = pinctrl_lookup_state(pxav2_host->pinctrl, "state_sdio_irq_fix");
			if (IS_ERR(pxav2_host->pins_sdio_irq_fix))
				pxav2_host->pins_sdio_irq_fix = NULL;
			pxav2_host->pins_default = pinctrl_lookup_state(pxav2_host->pinctrl, "default");
			if (IS_ERR(pxav2_host->pins_default))
				pxav2_host->pins_default = NULL;
		} else
			pxav2_host->pinctrl = NULL;
	} else {
		host->ops = &pxav2_sdhci_ops;
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto disable_axi_clk;

	return 0;

disable_axi_clk:
	if (!IS_ERR(pxav2_host->axi_clk))
		clk_disable_unprepare(pxav2_host->axi_clk);
disable_clk:
	clk_disable_unprepare(clk);
free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_pxav2_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxav2_host *pxav2_host = sdhci_pltfm_priv(pltfm_host);
	struct clk *axi_clk = pxav2_host->axi_clk;

	int ret = sdhci_pltfm_unregister(pdev);

	if (!IS_ERR(axi_clk))
		clk_disable_unprepare(axi_clk);

	return ret;
}

static struct platform_driver sdhci_pxav2_driver = {
	.driver		= {
		.name	= "sdhci-pxav2",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(sdhci_pxav2_of_match),
		.pm	= &sdhci_pltfm_pmops,
	},
	.probe		= sdhci_pxav2_probe,
	.remove		= sdhci_pxav2_remove,
};

module_platform_driver(sdhci_pxav2_driver);

MODULE_DESCRIPTION("SDHCI driver for pxav2");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL v2");

