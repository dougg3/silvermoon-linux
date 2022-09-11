// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Russell King
 * Copyright (C) 2022 Doug Brown
 *
 * Armada 168 variant support
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <drm/drm_probe_helper.h>
#include "armada_crtc.h"
#include "armada_drm.h"
#include "armada_hw.h"

struct armada168_variant_data {
	struct clk *clks[4];
	struct clk *sel_clk;
	struct clk *periph_clk;
	bool inhibit_next_periph_clk_enable;
};

static int armada168_crtc_init(struct armada_crtc *dcrtc, struct device *dev)
{
	struct armada168_variant_data *v;
	struct clk *clk;
	int idx;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;

	dcrtc->variant_data = v;

	if (dev->of_node) {
		struct property *prop;
		const char *s;

		of_property_for_each_string(dev->of_node, "clock-names", prop,
					    s) {
			if (!strcmp(s, "pclk"))
				idx = 0;
			else if (!strcmp(s, "ahbbus"))
				idx = 1;
			else if (!strcmp(s, "plldivider"))
				idx = 2;
			else if (!strcmp(s, "axibus"))
				idx = 3;
			else if (!strcmp(s, "periph"))
				idx = 4;
			else
				continue;

			clk = devm_clk_get(dev, s);
			if (IS_ERR(clk))
				return PTR_ERR(clk) == -ENOENT ? -EPROBE_DEFER :
					PTR_ERR(clk);
			if (idx < 4)
				v->clks[idx] = clk;
			else
				v->periph_clk = clk;
		}

		/* If we have a peripheral clock and we're trying to preserve the startup
		 * framebuffer, enable the clock now and inhibit the next enable */
		if (v->periph_clk &&
		    of_get_property(dev->of_node, "preserve-startup-fb", NULL)) {
			clk_prepare_enable(v->periph_clk);
			v->inhibit_next_periph_clk_enable = true;
		}

	} else {
		clk = devm_clk_get(dev, "ext_ref_clk1");
		if (IS_ERR(clk))
			return PTR_ERR(clk) == -ENOENT ? -EPROBE_DEFER :
				PTR_ERR(clk);

		v->clks[0] = clk;
	}

	return 0;
}

static const u32 armada168_clk_sels[] = {
	SCLK_16X_PCLK,
	SCLK_16X_AHB,
	SCLK_16X_PLL,
	SCLK_16X_AXI,
};

static const struct armada_clocking_params armada168_clocking = {
	/* HDMI requires -0.6%..+0.5% */
	.permillage_min = 994,
	.permillage_max = 1005,
	.settable = BIT(0),
	.div_max = SCLK_16X_INT_DIV_MASK,
};

/*
 * Armada510 specific SCLK register selection.
 * This gets called with sclk = NULL to test whether the mode is
 * supportable, and again with sclk != NULL to set the clocks up for
 * that.  The former can return an error, but the latter is expected
 * not to.
 */
static int armada168_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	struct armada168_variant_data *v = dcrtc->variant_data;
	unsigned long desired_khz = mode->crtc_clock;
	struct armada_clk_result res;
	int ret, idx;

	idx = armada_crtc_select_clock(dcrtc, &res, &armada168_clocking,
				       v->clks, ARRAY_SIZE(v->clks),
				       desired_khz);
	if (idx < 0)
		return idx;

	ret = clk_prepare_enable(res.clk);
	if (ret)
		return ret;

	if (sclk) {
		clk_set_rate(res.clk, res.desired_clk_hz);

		*sclk = res.div | armada168_clk_sels[idx];

		/* We are now using this clock */
		v->sel_clk = res.clk;
		swap(dcrtc->clk, res.clk);
	}

	clk_disable_unprepare(res.clk);

	return 0;
}

static void armada168_crtc_disable(struct armada_crtc *dcrtc)
{
	struct armada168_variant_data *v = dcrtc->variant_data;

	if (v->periph_clk)
		clk_disable_unprepare(v->periph_clk);

	if (dcrtc->clk) {
		clk_disable_unprepare(dcrtc->clk);
		dcrtc->clk = NULL;
	}
}

static void armada168_crtc_enable(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode)
{
	struct armada168_variant_data *v = dcrtc->variant_data;

	if (!dcrtc->clk && v->sel_clk) {
		if (!WARN_ON(clk_prepare_enable(v->sel_clk)))
			dcrtc->clk = v->sel_clk;
	}

	/* Enable the peripheral clock for the LCD as well.
	 * Note that we may have already enabled it, so inhibit if necessary */
	if (v->periph_clk && !v->inhibit_next_periph_clk_enable)
		clk_prepare_enable(v->periph_clk);
	else
		v->inhibit_next_periph_clk_enable = false;
}

const struct armada_variant armada168_ops = {
	.has_spu_adv_reg = false,
	.init = armada168_crtc_init,
	.compute_clock = armada168_crtc_compute_clock,
	.disable = armada168_crtc_disable,
	.enable = armada168_crtc_enable,
};
