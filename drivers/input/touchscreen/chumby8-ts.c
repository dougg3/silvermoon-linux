// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chumby 8 touchscreen driver
 *
 * Copyright (c) 2022 Doug Brown
 *
 * Based on:
 * - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *	Various changes: Imre Deak <imre.deak@nokia.com>
 * - silvermoon-tsb.c from Chumby kernel source
 *	Greg Hutchins
 */

#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#define TS_POLL_DELAY			5	/* ms delay before the first sample */
#define TS_POLL_PERIOD			1	/* ms delay between SPI transactions */
#define MAX_12BIT			((1 << 12) - 1) /* maximum possible value */
#define MIN_RAW_SAMPLE_VALUE		50	/* minimum valid response value for x and y */
#define MIN_RAW_PRESSURE		500	/* minimum valid response value for pressure */
#define COOLDOWN_DELAY			3	/* default settings used by original chumby driver */
#define NUM_SAMPS			10	/* default settings used by original chumby driver */
#define PENUP_THRESH			3980	/* default settings used by original chumby driver */
#define SETTLE_DELAY			500	/* default settings used by original chumby driver */
#define NOOPS_THRESHOLD		10	/* max number of invalid responses before retry */

/* SPI address space
 * These are the virtual registers that can be sent to the controller */
#define SPI_REGADR_NOOP		0
#define SPI_REGADR_LOOP		1
#define SPI_REGADR_DUMP_REQ		2
#define SPI_REGADR_COOLDOWN_DELAY	3
#define SPI_REGADR_NUMSAMPS		4
#define SPI_REGADR_PENUP_THRESH_HI	5
#define SPI_REGADR_PENUP_THRESH_LO	6
#define SPI_REGADR_SETTLE_DELAY	7
#define SPI_REGADR_INVALID_MAX_HI	8
#define SPI_REGADR_INVALID_MAX_LO	9
#define SPI_REGADR_INVALID_MIN_HI	10
#define SPI_REGADR_INVALID_MIN_LO	11
#define SPI_REGADR_LIGHTSENSOR_REQ	12

/* These are messages that are received from the controller.
 * They may be in response to any previous command. */
#define SPI_OUTPUT_NOOP		0
#define SPI_OUTPUT_X			1
#define SPI_OUTPUT_Y			2
#define SPI_OUTPUT_Z1			3
#define SPI_OUTPUT_Z2			4
#define SPI_OUTPUT_EOS			5
#define SPI_OUTPUT_LIGHT		6
#define SPI_OUTPUT_PRESSURE		7

#define WRITE_TS_REG(reg, val)	((reg << 8) | (val & 0xFF))
#define TS_OUTPUT_TYPE(x)		(x >> 12)
#define TS_OUTPUT_VALUE(x)		(x & 0x0FFF)

enum pollstate {
	requesting_state,
	reading_result,
};

struct chumby8_ts {
	struct input_dev	*input;
	char			phys[32];
	char			name[32];

	struct spi_device	*spi;

	u16			txbuf ____cacheline_aligned;
	u16			rxbuf ____cacheline_aligned;
	struct spi_transfer	xfer;
	struct spi_message	msg;
	wait_queue_head_t	wait;

	bool			pendown;
	u16			x;
	u16			y;
	u16			pressure;
	enum pollstate		state;
	u8			noops_received;

	struct touchscreen_properties core_prop;

	bool			stopped;

	int			gpio_pendown;
};

struct chumby8_ts_platform_data {
	int	gpio_pendown;		/* the GPIO used to decide the pendown state */
};

static int get_pendown_state(struct chumby8_ts *ts)
{
	return gpio_get_value(ts->gpio_pendown);
}

static void chumby8_ts_report_state(struct chumby8_ts *ts)
{
	struct input_dev *input = ts->input;
	if (!ts->pendown) {
		input_report_key(input, BTN_TOUCH, 1);
		ts->pendown = true;
	}

	touchscreen_report_pos(input, &ts->core_prop, ts->x, ts->y, false);
	input_report_abs(input, ABS_PRESSURE, ts->pressure);
	input_sync(input);
}

static void chumby8_ts_report_pen_up(struct chumby8_ts *ts)
{
	struct input_dev *input = ts->input;

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	ts->pendown = false;
}

static void chumby8_ts_stop(struct chumby8_ts *ts)
{
	/* Signal IRQ thread to stop polling and disable the handler. */
	ts->stopped = true;
	mb();
	wake_up(&ts->wait);
	disable_irq(ts->spi->irq);
}

static int chumby8_ts_rw(struct chumby8_ts *ts, u16 val, u16 *result)
{
	int error;
	ts->txbuf = val;

	error = spi_sync(ts->spi, &ts->msg);
	if (error) {
		return error;
	}

	if (result)
		*result = ts->rxbuf;
	return 0;
}

static void chumby8_ts_poll_state(struct chumby8_ts *ts)
{
	u16 val, result;
	int error;

	/* Request a new dump of state, or read out previous request */
	if (ts->state == requesting_state)
		val = WRITE_TS_REG(SPI_REGADR_DUMP_REQ, 0);
	else
		val = WRITE_TS_REG(SPI_REGADR_NOOP, 0);

	/* Do the actual (simple) transfer */
	error = chumby8_ts_rw(ts, val, &result);
	if (error) {
		dev_err(&ts->spi->dev, "spi_sync --> %d\n", error);
		return;
	}

	/* Update state variables */
	if (ts->state == requesting_state)
		ts->state = reading_result;
	else
		switch (TS_OUTPUT_TYPE(result)) {
			case SPI_OUTPUT_X:
				ts->noops_received = 0;
				ts->x = TS_OUTPUT_VALUE(result);
				break;
			case SPI_OUTPUT_Y:
				ts->noops_received = 0;
				ts->y = TS_OUTPUT_VALUE(result);
				break;
			case SPI_OUTPUT_PRESSURE:
				ts->noops_received = 0;
				ts->pressure = TS_OUTPUT_VALUE(result);
				/* this is the end of a full set of data. sanity check and report */
				if (ts->x >= MIN_RAW_SAMPLE_VALUE &&
				    ts->y >= MIN_RAW_SAMPLE_VALUE &&
				    ts->pressure >= MIN_RAW_PRESSURE &&
				    !ts->stopped) {
					chumby8_ts_report_state(ts);
				}
				/* request a new set of data next time */
				ts->state = requesting_state;
				break;
			case SPI_OUTPUT_NOOP:
			case SPI_OUTPUT_EOS:
			default:
				/* Count invalid responses received, in case it gets hung up */
				ts->noops_received++;
				break;
		}

	/* If we receive too many invalid responses in a row, re-request a dump */
	if (ts->noops_received >= NOOPS_THRESHOLD) {
		dev_warn(&ts->spi->dev, "noops reset\n");
		ts->noops_received = 0;
		ts->state = requesting_state;
	}
}

static irqreturn_t chumby8_ts_hard_irq(int irq, void *handle)
{
	struct chumby8_ts *ts = handle;

	return get_pendown_state(ts) ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}


static irqreturn_t chumby8_ts_irq(int irq, void *handle)
{
	struct chumby8_ts *ts = handle;

	/* start in a default state, ensure we don't use any coords from last time */
	ts->state = requesting_state;
	ts->noops_received = 0;
	ts->x = 0;
	ts->y = 0;
	ts->pressure = 0;

	/* Start with a small delay before checking pendown state */
	msleep(TS_POLL_DELAY);

	while (!ts->stopped && get_pendown_state(ts)) {

		/* pen is down, continue with the measurement */
		chumby8_ts_poll_state(ts);

		/* wait until we're stopped or it's time to poll again */
		wait_event_timeout(ts->wait, ts->stopped,
				   msecs_to_jiffies(TS_POLL_PERIOD));
	}

	if (ts->pendown && !ts->stopped)
		chumby8_ts_report_pen_up(ts);

	return IRQ_HANDLED;
}

static int chumby8_ts_setup_pendown(struct spi_device *spi,
				     struct chumby8_ts *ts,
				     const struct chumby8_ts_platform_data *pdata)
{
	int err;

	if (gpio_is_valid(pdata->gpio_pendown)) {
		err = devm_gpio_request_one(&spi->dev, pdata->gpio_pendown,
					    GPIOF_IN, "chumby8_ts_pendown");
		if (err) {
			dev_err(&spi->dev,
				"failed to request/setup pendown GPIO%d: %d\n",
				pdata->gpio_pendown, err);
			return err;
		}

		ts->gpio_pendown = pdata->gpio_pendown;
	} else {
		dev_err(&spi->dev, "no gpio_pendown\n");
		return -EINVAL;
	}

	return 0;
}

static int chumby8_ts_setup_spi_msg(struct chumby8_ts *ts)
{
	spi_message_init(&ts->msg);
	ts->xfer.tx_buf = &ts->txbuf;
	ts->xfer.rx_buf = &ts->rxbuf;
	ts->xfer.len = sizeof(ts->txbuf);
	spi_message_add_tail(&ts->xfer, &ts->msg);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id chumby8_ts_dt_ids[] = {
	{ .compatible = "chumby,chumby8ts" },
	{},
};
MODULE_DEVICE_TABLE(of, chumby8_ts_dt_ids);

static const struct chumby8_ts_platform_data *chumby8_ts_probe_dt(struct device *dev)
{
	struct chumby8_ts_platform_data *pdata;
	struct device_node *node = dev->of_node;

	if (!node) {
		dev_err(dev, "Device does not have associated DT data\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->gpio_pendown = of_get_named_gpio(dev->of_node, "pendown-gpio", 0);

	return pdata;
}
#else
static const struct chumby8_ts_platform_data *chumby8_ts_probe_dt(struct device *dev)
{
	dev_err(dev, "no platform data defined\n")_;
	return ERR_PTR(-EINVAL);
}
#endif

static int chumby8_ts_probe(struct spi_device *spi)
{
	const struct chumby8_ts_platform_data *pdata;
	struct chumby8_ts *ts;
	struct device *dev = &spi->dev;
	struct input_dev *input_dev;
	unsigned long irq_flags;
	int err;

	if (!spi->irq) {
		dev_dbg(dev, "no IRQ\n");
		return -EINVAL;
	}

	spi->bits_per_word = 16;
	spi->mode &= ~SPI_MODE_X_MASK;
	spi->mode |= SPI_MODE_1;
	/* Wait 7 clock times after asserting CS before we actually begin */
	spi->cs_setup.value = 7;
	spi->cs_setup.unit = SPI_DELAY_UNIT_SCK;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	ts = devm_kzalloc(dev, sizeof(struct chumby8_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;

	spi_set_drvdata(spi, ts);

	ts->spi = spi;
	ts->input = input_dev;

	init_waitqueue_head(&ts->wait);

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = chumby8_ts_probe_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	err = chumby8_ts_setup_pendown(spi, ts, pdata);
	if (err)
		return err;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));

	input_dev->name = "Chumby 8 touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_SPI;

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	/*
	 * Parse common framework properties. Must be done here to ensure the
	 * correct behaviour in case of using the legacy vendor bindings. The
	 * general binding value overrides the vendor specific one.
	 */
	touchscreen_parse_properties(ts->input, false, &ts->core_prop);

	chumby8_ts_setup_spi_msg(ts);
	
	/* Initial setup */
	chumby8_ts_rw(ts, WRITE_TS_REG(SPI_REGADR_COOLDOWN_DELAY, COOLDOWN_DELAY), NULL);
	mdelay(1);
	chumby8_ts_rw(ts, WRITE_TS_REG(SPI_REGADR_PENUP_THRESH_HI, (PENUP_THRESH >> 8)), NULL);
	mdelay(1);
	chumby8_ts_rw(ts, WRITE_TS_REG(SPI_REGADR_PENUP_THRESH_LO, (PENUP_THRESH & 0xFF)), NULL);
	mdelay(1);
	chumby8_ts_rw(ts, WRITE_TS_REG(SPI_REGADR_SETTLE_DELAY, SETTLE_DELAY/100), NULL);
	mdelay(1);
	chumby8_ts_rw(ts, WRITE_TS_REG(SPI_REGADR_NUMSAMPS, NUM_SAMPS), NULL);
	mdelay(1);

	/* IRQ for touch detection */
	irq_flags = IRQF_ONESHOT;

	err = devm_request_threaded_irq(dev, spi->irq,
					chumby8_ts_hard_irq, chumby8_ts_irq,
					irq_flags, dev->driver->name, ts);

	if (err) {
		dev_dbg(dev, "irq %d busy?\n", spi->irq);
		return err;
	}

	dev_info(dev, "touchscreen, irq %d\n", spi->irq);

	err = input_register_device(input_dev);
	if (err)
		return err;

	/*
	 * If device does not carry platform data we must have allocated it
	 * when parsing DT data.
	 */
	if (!dev_get_platdata(dev))
		devm_kfree(dev, (void *)pdata);

	return 0;
}

static void chumby8_ts_remove(struct spi_device *spi)
{
	struct chumby8_ts *ts = spi_get_drvdata(spi);

	chumby8_ts_stop(ts);
}

static const struct spi_device_id chumby8_ts_spi_id[] = {
	{ "chumby8ts" },
	{}
};
MODULE_DEVICE_TABLE(spi, chumby8_ts_spi_id);

static struct spi_driver chumby8_ts_driver = {
	.driver = {
		.name	= "chumby8_ts",
		.of_match_table = of_match_ptr(chumby8_ts_dt_ids),
	},
	.id_table	= chumby8_ts_spi_id,
	.probe		= chumby8_ts_probe,
	.remove	= chumby8_ts_remove,
};

module_spi_driver(chumby8_ts_driver);

MODULE_DESCRIPTION("Chumby 8 Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:chumby8-ts");
