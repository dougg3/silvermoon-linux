// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pxa-ssp.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2005,2008 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * TODO:
 *  o Test network mode for > 16bit sample size
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/of.h>
#include <linux/dmaengine.h>
#include <linux/of_device.h>

#include <asm/irq.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/pxa2xx-lib.h>
#include <sound/dmaengine_pcm.h>

// TODO: Fix this to go through common clock API
#define APB_VIRT_BASE		IOMEM(0xfe000000)
#define MPMU_VIRT_BASE		(APB_VIRT_BASE + 0x50000)
#define MPMU_REG(off)		(MPMU_VIRT_BASE + (off))
#define MPMU_ASYSDR		MPMU_REG(0x1050)
#define MPMU_ASSPDR		MPMU_REG(0x1054)

#include "pxa-ssp.h"

/*
 * SSP audio private data
 */
struct ssp_priv {
	struct ssp_device *ssp;
	struct clk *extclk;
	unsigned long ssp_clk;
	unsigned int sysclk;
	unsigned int dai_fmt;
	unsigned int configured_dai_fmt;
	struct snd_dmaengine_dai_dma_data pcm_stereo_out;
	struct snd_dmaengine_dai_dma_data pcm_stereo_in;
#ifdef CONFIG_PM
	uint32_t	cr0;
	uint32_t	cr1;
	uint32_t	to;
	uint32_t	psp;
#endif
};

/* For simplicity, provide calculations for MCLK assuming MCLK = 256fs */
struct pxa168_ssp_mclk {
	unsigned int mclk;
	unsigned int mclk_denom;
	unsigned int mclk_num;
};

static const struct pxa168_ssp_mclk mclk_conf[] = {
	{12288000,  64, 1625},		/* Used for 48000 */
	{11289600, 294, 8125},		/* Used for 44100 */
	{8192000,  128, 4875},		/* Used for 32000 */
	{6144000,   32, 1625},		/* Used for 24000 */
	{5644800,  147, 8125},		/* Used for 22050 */
	{4096000,   64, 4875},		/* Used for 16000 */
	{2822400,  147, 16250},	/* Used for 11025 */
	{2048000,   32, 4875},		/* Used for 8000 */
	{0,        294, 8125},		/* When we are asked to "shut off", leave MCLK running
					 * in order for the codec to be happy */
};

/*
 * Set the SSP ports SYSCLK only from Audio SYSCLK.
 */
static int pxa168_ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
				     unsigned int freq, int dir)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	unsigned int sscr0, data, asysdr, asspdr;
	int mclk_index;

	dev_dbg(ssp->dev, "%s id: %d, clk_id %d, freq %u\n",
		__func__, cpu_dai->id, clk_id, freq);

	/* Locate the entry for the matching frequency */
	for (mclk_index = 0; mclk_index < ARRAY_SIZE(mclk_conf); mclk_index++) {
		if (freq == mclk_conf[mclk_index].mclk)
			break;
	}

	if (mclk_index >= ARRAY_SIZE(mclk_conf)) {
		dev_warn(ssp->dev, "Wrong frequency on SYSCLK\n");
		return -EINVAL;
	}
	asysdr = (mclk_conf[mclk_index].mclk_num << 16)
		| mclk_conf[mclk_index].mclk_denom;
	asspdr = 0;
	/* If ASYSCLK is supplied by pxa168, ASSPDR should be configured. */
	if (freq != 0 && dir == SND_SOC_CLOCK_OUT)
		/* Currently assume a divider of 8 to work properly with 256fs */
		asspdr = (8 << 16) | 1;

	/* clear ECS, NCS, ACS */
	sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	data = sscr0 & ~(SSCR0_ECS | SSCR0_NCS | SSCR0_ACS);
	if (sscr0 != data)
		pxa_ssp_write_reg(ssp, SSCR0, data);

	/* update divider register in MPMU */
	__raw_writel(asysdr, MPMU_ASYSDR);
	__raw_writel(asspdr, MPMU_ASSPDR);

	return 0;
}

static void dump_registers(struct ssp_device *ssp)
{
	dev_dbg(ssp->dev, "SSCR0 0x%08x SSCR1 0x%08x SSTO 0x%08x\n",
		 pxa_ssp_read_reg(ssp, SSCR0), pxa_ssp_read_reg(ssp, SSCR1),
		 pxa_ssp_read_reg(ssp, SSTO));

	dev_dbg(ssp->dev, "SSPSP 0x%08x SSSR 0x%08x SSACD 0x%08x\n",
		 pxa_ssp_read_reg(ssp, SSPSP), pxa_ssp_read_reg(ssp, SSSR),
		 pxa_ssp_read_reg(ssp, SSACD));
}

static void pxa_ssp_set_dma_params(struct ssp_device *ssp, int width4,
			int out, struct snd_dmaengine_dai_dma_data *dma)
{
	dma->addr_width = width4 ? DMA_SLAVE_BUSWIDTH_4_BYTES :
				   DMA_SLAVE_BUSWIDTH_2_BYTES;
	dma->maxburst = 16;
	dma->addr = ssp->phys_base + SSDR;
}

static int pxa_ssp_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	int ret = 0;

	if (!snd_soc_dai_active(cpu_dai)) {
		clk_prepare_enable(ssp->clk);
		pxa_ssp_disable(ssp);
	}

	clk_prepare_enable(priv->extclk);

	return ret;
}

static void pxa_ssp_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;

	if (!snd_soc_dai_active(cpu_dai)) {
		pxa_ssp_disable(ssp);
		clk_disable_unprepare(ssp->clk);
	}

	clk_disable_unprepare(priv->extclk);
}

#ifdef CONFIG_PM

static int pxa_ssp_suspend(struct snd_soc_component *component)
{
	struct ssp_priv *priv = snd_soc_component_get_drvdata(component);
	struct ssp_device *ssp = priv->ssp;

	if (!snd_soc_component_active(component))
		clk_prepare_enable(ssp->clk);

	priv->cr0 = __raw_readl(ssp->mmio_base + SSCR0);
	priv->cr1 = __raw_readl(ssp->mmio_base + SSCR1);
	priv->to  = __raw_readl(ssp->mmio_base + SSTO);
	priv->psp = __raw_readl(ssp->mmio_base + SSPSP);

	pxa_ssp_disable(ssp);
	clk_disable_unprepare(ssp->clk);
	return 0;
}

static int pxa_ssp_resume(struct snd_soc_component *component)
{
	struct ssp_priv *priv = snd_soc_component_get_drvdata(component);
	struct ssp_device *ssp = priv->ssp;
	uint32_t sssr = SSSR_ROR | SSSR_TUR | SSSR_BCE;

	clk_prepare_enable(ssp->clk);

	__raw_writel(sssr, ssp->mmio_base + SSSR);
	__raw_writel(priv->cr0 & ~SSCR0_SSE, ssp->mmio_base + SSCR0);
	__raw_writel(priv->cr1, ssp->mmio_base + SSCR1);
	__raw_writel(priv->to,  ssp->mmio_base + SSTO);
	__raw_writel(priv->psp, ssp->mmio_base + SSPSP);

	if (snd_soc_component_active(component))
		pxa_ssp_enable(ssp);
	else
		clk_disable_unprepare(ssp->clk);

	return 0;
}

#else
#define pxa_ssp_suspend	NULL
#define pxa_ssp_resume	NULL
#endif

/*
 * ssp_set_clkdiv - set SSP clock divider
 * @div: serial clock rate divider
 */
static void pxa_ssp_set_scr(struct ssp_device *ssp, u32 div)
{
	u32 sscr0 = pxa_ssp_read_reg(ssp, SSCR0);

	if (ssp->type == PXA25x_SSP) {
		sscr0 &= ~0x0000ff00;
		sscr0 |= ((div - 2)/2) << 8; /* 2..512 */
	} else {
		sscr0 &= ~0x000fff00;
		sscr0 |= (div - 1) << 8;     /* 1..4096 */
	}
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);
}

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa_ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;

	u32 sscr0 = pxa_ssp_read_reg(ssp, SSCR0) &
		~(SSCR0_ECS | SSCR0_NCS | SSCR0_MOD | SSCR0_ACS);

	if (priv->extclk) {
		int ret;

		/*
		 * For DT based boards, if an extclk is given, use it
		 * here and configure PXA_SSP_CLK_EXT.
		 */

		ret = clk_set_rate(priv->extclk, freq);
		if (ret < 0)
			return ret;

		clk_id = PXA_SSP_CLK_EXT;
	}

	dev_dbg(ssp->dev,
		"pxa_ssp_set_dai_sysclk id: %d, clk_id %d, freq %u\n",
		cpu_dai->id, clk_id, freq);

	switch (clk_id) {
	case PXA_SSP_CLK_NET_PLL:
		sscr0 |= SSCR0_MOD;
		break;
	case PXA_SSP_CLK_PLL:
		/* Internal PLL is fixed */
		if (ssp->type == PXA25x_SSP)
			priv->sysclk = 1843200;
		else
			priv->sysclk = 13000000;
		break;
	case PXA_SSP_CLK_EXT:
		priv->sysclk = freq;
		sscr0 |= SSCR0_ECS;
		break;
	case PXA_SSP_CLK_NET:
		priv->sysclk = freq;
		sscr0 |= SSCR0_NCS | SSCR0_MOD;
		break;
	case PXA_SSP_CLK_AUDIO:
		priv->sysclk = 0;
		pxa_ssp_set_scr(ssp, 1);
		sscr0 |= SSCR0_ACS;
		break;
	default:
		return -ENODEV;
	}

	/* The SSP clock must be disabled when changing SSP clock mode
	 * on PXA2xx.  On PXA3xx it must be enabled when doing so. */
	if (ssp->type != PXA3xx_SSP)
		clk_disable_unprepare(ssp->clk);
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);
	if (ssp->type != PXA3xx_SSP)
		clk_prepare_enable(ssp->clk);

	return 0;
}

/*
 * Configure the PLL frequency pxa27x and (afaik - pxa320 only)
 */
static int pxa_ssp_set_pll(struct ssp_priv *priv, unsigned int freq)
{
	struct ssp_device *ssp = priv->ssp;
	u32 ssacd = pxa_ssp_read_reg(ssp, SSACD) & ~0x70;

	if (ssp->type == PXA3xx_SSP)
		pxa_ssp_write_reg(ssp, SSACDD, 0);

	switch (freq) {
	case 5622000:
		break;
	case 11345000:
		ssacd |= (0x1 << 4);
		break;
	case 12235000:
		ssacd |= (0x2 << 4);
		break;
	case 14857000:
		ssacd |= (0x3 << 4);
		break;
	case 32842000:
		ssacd |= (0x4 << 4);
		break;
	case 48000000:
		ssacd |= (0x5 << 4);
		break;
	case 0:
		/* Disable */
		break;

	default:
		/* PXA3xx has a clock ditherer which can be used to generate
		 * a wider range of frequencies - calculate a value for it.
		 */
		if (ssp->type == PXA3xx_SSP) {
			u32 val;
			u64 tmp = 19968;

			tmp *= 1000000;
			do_div(tmp, freq);
			val = tmp;

			val = (val << 16) | 64;
			pxa_ssp_write_reg(ssp, SSACDD, val);

			ssacd |= (0x6 << 4);

			dev_dbg(ssp->dev,
				"Using SSACDD %x to supply %uHz\n",
				val, freq);
			break;
		}

		return -EINVAL;
	}

	pxa_ssp_write_reg(ssp, SSACD, ssacd);

	return 0;
}

/*
 * Set the active slots in TDM/Network mode
 */
static int pxa_ssp_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0;

	sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	sscr0 &= ~(SSCR0_MOD | SSCR0_SlotsPerFrm(8) | SSCR0_EDSS | SSCR0_DSS);

	/* set slot width */
	if (slot_width > 16)
		sscr0 |= SSCR0_EDSS | SSCR0_DataSize(slot_width - 16);
	else
		sscr0 |= SSCR0_DataSize(slot_width);

	if (slots > 1 || ssp->type == PXA168_SSP) {
		/* enable network mode */
		sscr0 |= SSCR0_MOD;

		/* set number of active slots */
		sscr0 |= SSCR0_SlotsPerFrm(slots);

		/* set active slot mask */
		pxa_ssp_write_reg(ssp, SSTSA, tx_mask);
		pxa_ssp_write_reg(ssp, SSRSA, rx_mask);
	}
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);

	return 0;
}

/*
 * Tristate the SSP DAI lines
 */
static int pxa_ssp_set_dai_tristate(struct snd_soc_dai *cpu_dai,
	int tristate)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr1;

	sscr1 = pxa_ssp_read_reg(ssp, SSCR1);
	if (tristate)
		sscr1 &= ~SSCR1_TTE;
	else
		sscr1 |= SSCR1_TTE;
	pxa_ssp_write_reg(ssp, SSCR1, sscr1);

	return 0;
}

static int pxa_ssp_set_dai_fmt(struct snd_soc_dai *cpu_dai,
			       unsigned int fmt)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
	case SND_SOC_DAIFMT_BC_FP:
	case SND_SOC_DAIFMT_BP_FP:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		break;

	default:
		return -EINVAL;
	}

	/* Settings will be applied in hw_params() */
	priv->dai_fmt = fmt;

	return 0;
}

/*
 * Set up the SSP DAI format.
 * The SSP Port must be inactive before calling this function as the
 * physical interface format is changed.
 */
static int pxa_ssp_configure_dai_fmt(struct ssp_priv *priv)
{
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0, sscr1, sspsp, scfr;

	/* check if we need to change anything at all */
	if (priv->configured_dai_fmt == priv->dai_fmt)
		return 0;

	/* reset port settings */
	sscr0 = pxa_ssp_read_reg(ssp, SSCR0) &
		~(SSCR0_PSP | SSCR0_MOD);
	sscr1 = pxa_ssp_read_reg(ssp, SSCR1) &
		~(SSCR1_SCLKDIR | SSCR1_SFRMDIR | SSCR1_SCFR |
		  SSCR1_RWOT | SSCR1_TRAIL | SSCR1_TFT | SSCR1_RFT);
	sspsp = pxa_ssp_read_reg(ssp, SSPSP) &
		~(SSPSP_SFRMP | SSPSP_SCMODE(3));

	sscr1 |= SSCR1_RxTresh(8) | SSCR1_TxTresh(7);

	switch (priv->dai_fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SFRMDIR | SSCR1_SCFR;
		break;
	case SND_SOC_DAIFMT_BC_FP:
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SCFR;
		break;
	case SND_SOC_DAIFMT_BP_FP:
		break;
	default:
		return -EINVAL;
	}

	switch (priv->dai_fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		sspsp |= SSPSP_SFRMP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		sspsp |= SSPSP_SCMODE(2);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		sspsp |= SSPSP_SCMODE(2) | SSPSP_SFRMP;
		break;
	default:
		return -EINVAL;
	}

	switch (priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
		sscr0 |= SSCR0_PSP;
		/* Use network mode for I2S emulation on the PXA168 */
		if (ssp->type == PXA168_SSP)
			sscr0 |= SSCR0_MOD;
		sscr1 |= SSCR1_RWOT | SSCR1_TRAIL;
		/* See hw_params() */
		break;

	case SND_SOC_DAIFMT_DSP_A:
		sspsp |= SSPSP_FSRT;
		fallthrough;
	case SND_SOC_DAIFMT_DSP_B:
		sscr0 |= SSCR0_MOD | SSCR0_PSP;
		sscr1 |= SSCR1_TRAIL | SSCR1_RWOT;
		break;

	default:
		return -EINVAL;
	}

	pxa_ssp_write_reg(ssp, SSCR0, sscr0);
	pxa_ssp_write_reg(ssp, SSCR1, sscr1);
	pxa_ssp_write_reg(ssp, SSPSP, sspsp);

	switch (priv->dai_fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
	case SND_SOC_DAIFMT_BC_FP:
		scfr = pxa_ssp_read_reg(ssp, SSCR1) | SSCR1_SCFR;
		pxa_ssp_write_reg(ssp, SSCR1, scfr);

		while (pxa_ssp_read_reg(ssp, SSSR) & SSSR_BSY)
			cpu_relax();
		break;
	}

	dump_registers(ssp);

	/* Since we are configuring the timings for the format by hand
	 * we have to defer some things until hw_params() where we
	 * know parameters like the sample size.
	 */
	priv->configured_dai_fmt = priv->dai_fmt;

	return 0;
}

struct pxa_ssp_clock_mode {
	int rate;
	int pll;
	u8 acds;
	u8 scdb;
};

static const struct pxa_ssp_clock_mode pxa_ssp_clock_modes[] = {
	{ .rate =  8000, .pll = 32842000, .acds = SSACD_ACDS_32, .scdb = SSACD_SCDB_4X },
	{ .rate = 11025, .pll =  5622000, .acds = SSACD_ACDS_4,  .scdb = SSACD_SCDB_4X },
	{ .rate = 16000, .pll = 32842000, .acds = SSACD_ACDS_16, .scdb = SSACD_SCDB_4X },
	{ .rate = 22050, .pll =  5622000, .acds = SSACD_ACDS_2,  .scdb = SSACD_SCDB_4X },
	{ .rate = 44100, .pll = 11345000, .acds = SSACD_ACDS_2,  .scdb = SSACD_SCDB_4X },
	{ .rate = 48000, .pll = 12235000, .acds = SSACD_ACDS_2,  .scdb = SSACD_SCDB_4X },
	{ .rate = 96000, .pll = 12235000, .acds = SSACD_ACDS_4,  .scdb = SSACD_SCDB_1X },
	{}
};

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	int chn = params_channels(params);
	u32 sscr0, sspsp;
	int width = snd_pcm_format_physical_width(params_format(params));
	int ttsa = pxa_ssp_read_reg(ssp, SSTSA) & 0xf;
	struct snd_dmaengine_dai_dma_data *dma_data;
	int rate = params_rate(params);
	int bclk = rate * chn * (width / 8);
	int ret;

	dma_data = snd_soc_dai_get_dma_data(cpu_dai, substream);

	/* Network mode with one active slot (ttsa == 1) can be used
	 * to force 16-bit frame width on the wire (for S16_LE), even
	 * with two channels. Use 16-bit DMA transfers for this case.
	 * Also use 16-bit DMA transfers on PXA168 with S16_LE, even
	 * when there are two channels with network mode. Combining
	 * them into a single 32-bit transfer doesn't work properly.
	 */
	pxa_ssp_set_dma_params(ssp,
		((ssp->type != PXA168_SSP) && (chn == 2) && (ttsa != 1)) || (width == 32),
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK, dma_data);

	/* we can only change the settings if the port is not in use */
	if (pxa_ssp_read_reg(ssp, SSCR0) & SSCR0_SSE)
		return 0;

	ret = pxa_ssp_configure_dai_fmt(priv);
	if (ret < 0)
		return ret;

	/* Configure network mode on the PXA168 */
	if (ssp->type == PXA168_SSP) {
		/* If we have 2 channels, set up 2 time slots. If we have 1 channel,
		 * set up only one time slot. This seems silly, but the PXA168
		 * runs into random garbled audio if the second time slot is left active
		 * when playing mono sound. Only happens after recording. Weird. */
		pxa_ssp_set_dai_tdm_slot(cpu_dai, chn > 1 ? 3 : 1, chn > 1 ? 3 : 1, chn, width);
		pxa_ssp_set_dai_tristate(cpu_dai, 0);
		ttsa = pxa_ssp_read_reg(ssp, SSTSA) & 0xf;
	}

	/* clear selected SSP bits */
	sscr0 = pxa_ssp_read_reg(ssp, SSCR0) & ~(SSCR0_DSS | SSCR0_EDSS);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (ssp->type == PXA3xx_SSP)
			sscr0 |= SSCR0_FPCKE;
		sscr0 |= SSCR0_DataSize(16);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sscr0 |= (SSCR0_EDSS | SSCR0_DataSize(8));
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sscr0 |= (SSCR0_EDSS | SSCR0_DataSize(16));
		break;
	}
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);

	if (sscr0 & SSCR0_ACS) {
		ret = pxa_ssp_set_pll(priv, bclk);

		/*
		 * If we were able to generate the bclk directly,
		 * all is fine. Otherwise, look up the closest rate
		 * from the table and also set the dividers.
		 */

		if (ret < 0) {
			const struct pxa_ssp_clock_mode *m;
			int ssacd;

			for (m = pxa_ssp_clock_modes; m->rate; m++) {
				if (m->rate == rate)
					break;
			}

			if (!m->rate)
				return -EINVAL;

			ret = pxa_ssp_set_pll(priv, bclk);
			if (ret < 0)
				return ret;

			ssacd = pxa_ssp_read_reg(ssp, SSACD);
			ssacd &= ~(SSACD_ACDS(7) | SSACD_SCDB_1X);
			ssacd |= SSACD_ACDS(m->acds);
			ssacd |= m->scdb;
			pxa_ssp_write_reg(ssp, SSACD, ssacd);
		}
	} else if (sscr0 & SSCR0_ECS) {
		/*
		 * For setups with external clocking, the PLL and its diviers
		 * are not active. Instead, the SCR bits in SSCR0 can be used
		 * to divide the clock.
		 */
		pxa_ssp_set_scr(ssp, bclk / rate);
	}

	switch (priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sspsp = pxa_ssp_read_reg(ssp, SSPSP);

		if (((priv->sysclk / bclk) == 64) && (width == 16)) {
			/* This is a special case where the bitclk is 64fs
			 * and we're not dealing with 2*32 bits of audio
			 * samples.
			 *
			 * The SSP values used for that are all found out by
			 * trying and failing a lot; some of the registers
			 * needed for that mode are only available on PXA3xx.
			 */
			if (ssp->type != PXA3xx_SSP)
				return -EINVAL;

			sspsp |= SSPSP_SFRMWDTH(width * 2);
			sspsp |= SSPSP_SFRMDLY(width * 4);
			sspsp |= SSPSP_EDMYSTOP(3);
			sspsp |= SSPSP_DMYSTOP(3);
			sspsp |= SSPSP_DMYSTRT(1);
		} else {
			/* The frame width is the width the LRCLK is
			 * asserted for; the delay is expressed in
			 * half cycle units.  We need the extra cycle
			 * because the data starts clocking out one BCLK
			 * after LRCLK changes polarity.
			 */
			sspsp |= SSPSP_SFRMWDTH(width + 1);
			sspsp |= SSPSP_SFRMDLY((width + 1) * 2);
			sspsp |= SSPSP_DMYSTRT(1);
		}

		pxa_ssp_write_reg(ssp, SSPSP, sspsp);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sspsp = pxa_ssp_read_reg(ssp, SSPSP);

		/* Clear out the bits we're going to write to */
		sspsp &= ~SSPSP_SFRMWDTH(0x3F);
		sspsp &= ~SSPSP_SFRMDLY(0x7F);
		sspsp &= ~SSPSP_DMYSTRT(0x3);
		sspsp &= ~SSPSP_DMYSTOP(0x3);
		sspsp &= ~SSPSP_EDMYSTOP(0x7);

		/* Make LRCLK high for left channel, low for right channel */
		sspsp |= SSPSP_SFRMWDTH(width);
		/* If 1 channel, add a 2nd channel's worth of dummy data so we
		 * don't need to set a different bit clock for mono. This won't
		 * work for 32-bit though because the DMYSTOP range is 0-31.
		 * If someone needs 32, they'll need to figure out how. */
		if (chn == 1 && width < 32) {
			sspsp |= SSPSP_DMYSTOP(width & 0x3);
			sspsp |= SSPSP_EDMYSTOP((width >> 2) & 0x7);
		}

		pxa_ssp_write_reg(ssp, SSPSP, sspsp);
		break;
	default:
		break;
	}

	/* When we use a network mode, we always require TDM slots
	 * - complain loudly and fail if they've not been set up yet.
	 */
	if ((sscr0 & SSCR0_MOD) && !ttsa) {
		dev_err(ssp->dev, "No TDM timeslot configured\n");
		return -EINVAL;
	}

	dump_registers(ssp);

	return 0;
}

static void pxa_ssp_set_running_bit(struct snd_pcm_substream *substream,
				    struct ssp_device *ssp, int value)
{
	uint32_t sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	uint32_t sscr1 = pxa_ssp_read_reg(ssp, SSCR1);
	uint32_t sspsp = pxa_ssp_read_reg(ssp, SSPSP);
	uint32_t sssr = pxa_ssp_read_reg(ssp, SSSR);

	if (value && (sscr0 & SSCR0_SSE))
		pxa_ssp_write_reg(ssp, SSCR0, sscr0 & ~SSCR0_SSE);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (value)
			sscr1 |= SSCR1_TSRE;
		else
			sscr1 &= ~SSCR1_TSRE;
	} else {
		if (value)
			sscr1 |= SSCR1_RSRE;
		else
			sscr1 &= ~SSCR1_RSRE;
	}

	pxa_ssp_write_reg(ssp, SSCR1, sscr1);

	if (value) {
		pxa_ssp_write_reg(ssp, SSSR, sssr);
		pxa_ssp_write_reg(ssp, SSPSP, sspsp);
		pxa_ssp_write_reg(ssp, SSCR0, sscr0 | SSCR0_SSE);
	}
}

static int pxa_ssp_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *cpu_dai)
{
	int ret = 0;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	int val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		pxa_ssp_enable(ssp);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pxa_ssp_set_running_bit(substream, ssp, 1);
		val = pxa_ssp_read_reg(ssp, SSSR);
		pxa_ssp_write_reg(ssp, SSSR, val);
		break;
	case SNDRV_PCM_TRIGGER_START:
		pxa_ssp_set_running_bit(substream, ssp, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pxa_ssp_set_running_bit(substream, ssp, 0);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pxa_ssp_disable(ssp);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pxa_ssp_set_running_bit(substream, ssp, 0);
		break;

	default:
		ret = -EINVAL;
	}

	dump_registers(ssp);

	return ret;
}

static int pxa_ssp_probe(struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct ssp_priv *priv;
	int ret;

	priv = kzalloc(sizeof(struct ssp_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (dev->of_node) {
		struct device_node *ssp_handle;

		ssp_handle = of_parse_phandle(dev->of_node, "port", 0);
		if (!ssp_handle) {
			dev_err(dev, "unable to get 'port' phandle\n");
			ret = -ENODEV;
			goto err_priv;
		}

		priv->ssp = pxa_ssp_request_of(ssp_handle, "SoC audio");
		if (priv->ssp == NULL) {
			ret = -ENODEV;
			goto err_priv;
		}

		priv->extclk = devm_clk_get(dev, "extclk");
		if (IS_ERR(priv->extclk)) {
			ret = PTR_ERR(priv->extclk);
			if (ret == -EPROBE_DEFER)
				goto err_priv;

			priv->extclk = NULL;
		}
	} else {
		priv->ssp = pxa_ssp_request(dai->id + 1, "SoC audio");
		if (priv->ssp == NULL) {
			ret = -ENODEV;
			goto err_priv;
		}
	}

	priv->dai_fmt = (unsigned int) -1;
	snd_soc_dai_set_drvdata(dai, priv);

	/* On the PXA168, we should make sure the master clock is running.
	 * The codec might need it. Just choose a default value. */
	if (priv->ssp->type == PXA168_SSP)
		pxa168_ssp_set_dai_sysclk(dai, 0, 0, SND_SOC_CLOCK_IN);

	priv->pcm_stereo_out.chan_name = "tx";
	priv->pcm_stereo_in.chan_name = "rx";
	snd_soc_dai_init_dma_data(dai, &priv->pcm_stereo_out, &priv->pcm_stereo_in);

	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static int pxa_ssp_remove(struct snd_soc_dai *dai)
{
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(dai);

	pxa_ssp_free(priv->ssp);
	kfree(priv);
	return 0;
}

#define PXA_SSP_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			  SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
			  SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 |	\
			  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define PXA_SSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops pxa_ssp_dai_ops = {
	.probe		= pxa_ssp_probe,
	.remove		= pxa_ssp_remove,
	.startup	= pxa_ssp_startup,
	.shutdown	= pxa_ssp_shutdown,
	.trigger	= pxa_ssp_trigger,
	.hw_params	= pxa_ssp_hw_params,
	.set_sysclk	= pxa_ssp_set_dai_sysclk,
	.set_fmt	= pxa_ssp_set_dai_fmt,
	.set_tdm_slot	= pxa_ssp_set_dai_tdm_slot,
	.set_tristate	= pxa_ssp_set_dai_tristate,
};

static struct snd_soc_dai_driver pxa_ssp_dai = {
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			 .channels_min = 1,
			 .channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa_ssp_dai_ops,
};

static const struct snd_soc_component_driver pxa_ssp_component = {
	.name			= "pxa-ssp",
	.pcm_construct		= pxa2xx_soc_pcm_new,
	.open			= pxa2xx_soc_pcm_open,
	.close			= pxa2xx_soc_pcm_close,
	.hw_params		= pxa2xx_soc_pcm_hw_params,
	.prepare		= pxa2xx_soc_pcm_prepare,
	.trigger		= pxa2xx_soc_pcm_trigger,
	.pointer		= pxa2xx_soc_pcm_pointer,
	.suspend		= pxa_ssp_suspend,
	.resume			= pxa_ssp_resume,
	.legacy_dai_naming	= 1,
};

/* DAI driver for PXA168. It's mostly the same as PXA2xx/3xx */

#define PXA168_SSP_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			  SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
			  SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000)

#define PXA168_SSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | NDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops pxa168_ssp_dai_ops = {
	.probe		= pxa_ssp_probe,
	.remove		= pxa_ssp_remove,
	.startup	= pxa_ssp_startup,
	.shutdown	= pxa_ssp_shutdown,
	.trigger	= pxa_ssp_trigger,
	.hw_params	= pxa_ssp_hw_params,
	.set_sysclk	= pxa168_ssp_set_dai_sysclk,
	.set_fmt	= pxa_ssp_set_dai_fmt,
	.set_tdm_slot	= pxa_ssp_set_dai_tdm_slot,
	.set_tristate	= pxa_ssp_set_dai_tristate,
};

static struct snd_soc_dai_driver pxa168_ssp_dai = {
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			 .channels_min = 1,
			 .channels_max = 2,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa168_ssp_dai_ops,
};

#ifdef CONFIG_OF
static const struct of_device_id pxa_ssp_of_ids[] = {
	{ .compatible = "mrvl,pxa-ssp-dai", .data = &pxa_ssp_dai },
	{ .compatible = "mrvl,pxa168-ssp-dai", .data = &pxa168_ssp_dai },
	{}
};
MODULE_DEVICE_TABLE(of, pxa_ssp_of_ids);
#endif

static int asoc_ssp_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_driver = &pxa_ssp_dai;
	struct device *dev = &pdev->dev;
	if (dev->of_node) {
		const struct of_device_id *id =
			of_match_device(of_match_ptr(pxa_ssp_of_ids), dev);
		dai_driver = (struct snd_soc_dai_driver *)id->data;
	}

	return devm_snd_soc_register_component(&pdev->dev, &pxa_ssp_component,
					       dai_driver, 1);
}

static struct platform_driver asoc_ssp_driver = {
	.driver = {
		.name = "pxa-ssp-dai",
		.of_match_table = of_match_ptr(pxa_ssp_of_ids),
	},

	.probe = asoc_ssp_probe,
};

module_platform_driver(asoc_ssp_driver);

/* Module information */
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("PXA SSP/PCM SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa-ssp-dai");
