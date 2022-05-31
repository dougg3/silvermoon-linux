#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/reboot.h>
#include <uapi/linux/serial_reg.h>
#include <asm/delay.h>

static struct chumby8_rebootcon {
	struct clk *clk;
	void __iomem *base;
	struct notifier_block nb;
} c8_poweroff;

static inline unsigned int chumby8_coproc_serial_in(int offset)
{
	offset <<= 2;
	return readl(c8_poweroff.base + offset);
}

static inline void chumby8_coproc_serial_out(int offset, int value)
{
	offset <<= 2;
	writel(value, c8_poweroff.base + offset);
}

static void chumby8_coproc_putc(char c)
{
	/* wait for transmit available */
	while (!(chumby8_coproc_serial_in(UART_LSR) & UART_LSR_THRE));

	/* send the char */
	chumby8_coproc_serial_out(UART_TX, c);
}

static int chumby8_coproc_getc(void)
{
	int c = -1;

	if (chumby8_coproc_serial_in(UART_LSR) & UART_LSR_DR) {
		c = chumby8_coproc_serial_in(UART_RX);
	}

	return c;
}

static void chumby8_write_coproc_data(const char *cmd, int ch_delay)
{
	while (*cmd)
	{
		udelay(ch_delay);
		chumby8_coproc_putc(*cmd++);
	}
}

static void chumby8_send_coproc_command(const char *cmd)
{
	int i, j, k;
	int q_found;
	int clk_rate;
	u16 divisor;

	/* Disable UART interrupts and FIFOs */
	chumby8_coproc_serial_out(UART_IER, UART_IER_UUE);
	chumby8_coproc_serial_out(UART_FCR, 0);

	/* Ensure UART is set for 115200, 8N1 */
	clk_rate = clk_get_rate(c8_poweroff.clk);
	divisor = clk_rate / 16 / 115200;
	chumby8_coproc_serial_out(UART_LCR, UART_LCR_DLAB | UART_LCR_WLEN8);
	chumby8_coproc_serial_out(UART_DLL, divisor & 0xFF);
	chumby8_coproc_serial_out(UART_DLM, divisor >> 8);
	chumby8_coproc_serial_out(UART_LCR, UART_LCR_WLEN8);

	/* Try several times, in case command is lost */
	for (i = 0; i < 8; i++) {
		/* Flush UART */
		while (chumby8_coproc_getc() >= 0);

		/* Send "!!!!", listen for '?' */
		q_found = 0;
		for (j = 0; j < 4 && !q_found; j++) {
			chumby8_write_coproc_data("!!!!", 0);
			for (k = 0; k < 10 && !q_found; k++) {
				/* If we read a ?, we're good to go */
				if (chumby8_coproc_getc() == '?') {
					q_found = 1;
					break;
				}
				udelay(300);
			}
		}

		/* Now send the actual command */
		chumby8_write_coproc_data(cmd, 1000);
	}
}

static int chumby8_reset(struct notifier_block *this, unsigned long mode,
			 void *cmd)
{
	chumby8_send_coproc_command("RSET\n\r");
	return NOTIFY_DONE;
}

static void chumby8_poweroff(void)
{
	chumby8_send_coproc_command("DOWN\n\r");
}

static int chumby8_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	c8_poweroff.base = devm_ioremap_resource(dev, res);
	if (IS_ERR(c8_poweroff.base))
		return PTR_ERR(c8_poweroff.base);

	c8_poweroff.clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c8_poweroff.clk))
		return PTR_ERR(c8_poweroff.clk);

	ret = clk_prepare_enable(c8_poweroff.clk);
	if (ret) {
		dev_err(dev, "Unable to enable Chumby 8 restart clock\n");
		return ret;
	}

	c8_poweroff.nb.notifier_call = chumby8_reset;
	c8_poweroff.nb.priority = 192;

	ret = register_restart_handler(&c8_poweroff.nb);
	if (ret) {
		dev_err(dev, "Unable to register Chumby 8 restart handler\n");
		goto clk_disable;
	}

	pm_power_off = chumby8_poweroff;

	dev_info(dev, "Chumby 8 reboot driver registered\n");

	return 0;

clk_disable:
	clk_disable_unprepare(c8_poweroff.clk);
	return ret;
}

static const struct of_device_id chumby8_reboot_of_match[] = {
	{
		.compatible = "chumby,chumby8-reboot-controller",
	},
	{}
};

static struct platform_driver chumby8_reboot_driver = {
	.probe = chumby8_reboot_probe,
	.driver = {
		.name = "chumby8-poweroff",
		.of_match_table = chumby8_reboot_of_match,
	},
};
builtin_platform_driver(chumby8_reboot_driver);
