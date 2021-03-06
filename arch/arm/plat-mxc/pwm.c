/*
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from PXA PWM driver by Eric Miao <eric.miao@marvell.com>
 *
 * Copyright 2009 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <asm/arch/hardware.h>


/* i.MX35 PWM function block: */

#define MX35_PWMCR                 0x00    /* PWM Control Register */
#define MX35_PWMSAR                0x0C    /* PWM Sample Register */
#define MX35_PWMPR                 0x10    /* PWM Period Register */
#define MX35_PWMCR_PRESCALER(x)    (((x - 1) & 0xFFF) << 4)

#define MX35_PWMCR_STOPEN		(1 << 25)
#define MX35_PWMCR_DOZEEN                (1 << 24)
#define MX35_PWMCR_WAITEN                (1 << 23)
#define MX35_PWMCR_DBGEN			(1 << 22)
#define MX35_PWMCR_CLKSRC_IPG		(1 << 16)
#define MX35_PWMCR_CLKSRC_IPG_HIGH	(2 << 16)
#define MX35_PWMCR_CLKSRC_IPG_32k	(3 << 16)
#define MX35_PWMCR_EN			(1 << 0)

struct pwm_device {
	struct list_head	node;
	struct platform_device *pdev;

	const char	*label;
	struct clk	*clk;

	int		clk_enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long long c;
	unsigned long period_cycles, duty_cycles, prescale;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	c = clk_get_rate(pwm->clk);
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = (unsigned long long)period_cycles * duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	writel(duty_cycles, pwm->mmio_base + MX35_PWMSAR);
	writel(period_cycles, pwm->mmio_base + MX35_PWMPR);
	writel(MX35_PWMCR_PRESCALER(prescale) |
		MX35_PWMCR_CLKSRC_IPG_HIGH |
		MX35_PWMCR_STOPEN | MX35_PWMCR_DOZEEN |
		MX35_PWMCR_WAITEN | MX35_PWMCR_DBGEN,
		pwm->mmio_base + MX35_PWMCR);

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	unsigned long reg;
	int rc = 0;

	if (!pwm->clk_enabled) {
		rc = clk_enable(pwm->clk);
		if (!rc)
			pwm->clk_enabled = 1;
	}

	reg = readl(pwm->mmio_base + MX35_PWMCR);
	reg |= MX35_PWMCR_EN;
	writel(reg, pwm->mmio_base + MX35_PWMCR);
	return rc;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	unsigned long reg;

	if (pwm->clk_enabled) {
		clk_disable(pwm->clk);
		pwm->clk_enabled = 0;
	}

	reg = readl(pwm->mmio_base + MX35_PWMCR);
	reg &= ~MX35_PWMCR_EN;
	writel(reg, pwm->mmio_base + MX35_PWMCR);

}
EXPORT_SYMBOL(pwm_disable);

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		pr_warning("PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

static int __devinit mxc_pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;
	int ret = 0, irq = 0;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->clk = clk_get(&pdev->dev, "pwm_clk");

	if (IS_ERR(pwm->clk)) {
		ret = PTR_ERR(pwm->clk);
		goto err_free;
	}

	pwm->clk_enabled = 0;

	pwm->use_count = 0;
	pwm->pwm_id = pdev->id;
	pwm->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		return -ENODEV;
	};

	irq = r->start;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	pwm->mmio_base = ioremap(r->start, resource_size(r));
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);

	printk(KERN_INFO "MXC PWM Initialized\n");

	platform_set_drvdata(pdev, pwm);
	return 0;

err_free_mem:
	release_mem_region(r->start, resource_size(r));
err_free_clk:
	clk_put(pwm->clk);
err_free:
	kfree(pwm);
	return ret;
}

static int __devexit mxc_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	clk_put(pwm->clk);

	kfree(pwm);
	return 0;
}

static struct platform_driver mxc_pwm_driver = {
	.driver		= {
		.name	= "mxc_pwm",
	},
	.probe		= mxc_pwm_probe,
	.remove		= __devexit_p(mxc_pwm_remove),
};

static int __init mxc_pwm_init(void)
{
	return platform_driver_register(&mxc_pwm_driver);
}
arch_initcall(mxc_pwm_init);

static void __exit mxc_pwm_exit(void)
{
	platform_driver_unregister(&mxc_pwm_driver);
}
module_exit(mxc_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
