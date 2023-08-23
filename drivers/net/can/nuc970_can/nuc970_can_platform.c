/*
 *  linux/drivers/net/can/nuc970_can/nuc970_can_platform.c
 *
 *  NUC970/N9H30 CAN driver
 *
 *
 *  Copyright (C) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>

#include <linux/can/dev.h>

#include "nuc970_can.h"

#define DCAN_RAM_INIT_BIT BIT(3)

/* 16-bit c_can registers can be arranged differently in the memory
 * architecture of different implementations. For example: 16-bit
 * registers can be aligned to a 16-bit boundary or 32-bit boundary etc.
 * Handle the same by providing a common read/write interface.
 */
static u16 c_can_plat_read_reg_aligned_to_32bit(const struct c_can_priv *priv,
						enum reg index)
{
	return readw(priv->base + 2 * priv->regs[index]);
}

static void c_can_plat_write_reg_aligned_to_32bit(const struct c_can_priv *priv,
						  enum reg index, u16 val)
{
	writew(val, priv->base + 2 * priv->regs[index]);
}

static u32 c_can_plat_read_reg32(const struct c_can_priv *priv, enum reg index)
{
	u32 val;

	val = priv->read_reg(priv, index);
	val |= ((u32)priv->read_reg(priv, index + 1)) << 16;

	return val;
}

static void c_can_plat_write_reg32(const struct c_can_priv *priv,
				   enum reg index, u32 val)
{
	priv->write_reg(priv, index + 1, val >> 16);
	priv->write_reg(priv, index, val);
}

static const struct c_can_driver_data c_can_drvdata = {
	.id = BOSCH_C_CAN,
};

static const struct platform_device_id c_can_id_table[] = {
	{
		.name = KBUILD_MODNAME,
		.driver_data = (kernel_ulong_t)&c_can_drvdata,
	},
	{
		.name = "c_can",
		.driver_data = (kernel_ulong_t)&c_can_drvdata,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, c_can_id_table);

static const struct of_device_id c_can_of_table[] = {
	{ .compatible = "nuvoton,c_can", .data = &c_can_drvdata },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, c_can_of_table);

static int c_can_plat_probe(struct platform_device *pdev)
{
	int ret;
	void __iomem *addr;
	struct net_device *dev;
	struct c_can_priv *priv;
	const struct of_device_id *match;
	struct resource *mem;
	int irq;
	struct clk *clk;
	struct pinctrl *pinctrl;
	const struct c_can_driver_data *drvdata;
	//struct device_node *np = pdev->dev.of_node;

	match = of_match_device(c_can_of_table, &pdev->dev);
	if (match) {
		drvdata = match->data;
	} else if (pdev->id_entry->driver_data) {
		drvdata = (struct c_can_driver_data *)
			platform_get_device_id(pdev)->driver_data;
	} else {
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	/* get the platform data */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		ret = -ENODEV;
		goto exit;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(addr)) {
		ret =  PTR_ERR(addr);
		goto exit;
	}

	if((u32)addr == 0xf800b000) {
		clk = clk_get(NULL, "can0");
		clk_prepare(clk);
		clk_enable(clk);
	}
	else if ((u32)addr == 0xf800b400) {
		clk = clk_get(NULL, "can1");
		clk_prepare(clk);
		clk_enable(clk);
	}

	/* allocate the c_can device */
	dev = alloc_c_can_dev();
	if (!dev) {
		ret = -ENOMEM;
		goto exit;
	}

	mem->flags |= IORESOURCE_MEM_32BIT;

	priv = netdev_priv(dev);

	priv->regs = reg_map_c_can;
	priv->read_reg = c_can_plat_read_reg_aligned_to_32bit;
	priv->write_reg = c_can_plat_write_reg_aligned_to_32bit;
	priv->read_reg32 = c_can_plat_read_reg32;
	priv->write_reg32 = c_can_plat_write_reg32;

	dev->irq = irq;
	priv->base = addr;
	priv->device = &pdev->dev;
	priv->can.clock.freq = clk_get_rate(clk);
	priv->priv = clk;
	priv->type = drvdata->id;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	pm_runtime_enable(priv->device);
	ret = register_c_can_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto exit_free_device;
	}

	dev_info(&pdev->dev, "%s device registered (regs=%p, irq=%d)\n",
		 KBUILD_MODNAME, priv->base, dev->irq);
	return 0;

exit_free_device:
	pm_runtime_disable(priv->device);
	free_c_can_dev(dev);
exit:
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int c_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct c_can_priv *priv = netdev_priv(dev);

	unregister_c_can_dev(dev);
	pm_runtime_disable(priv->device);
	free_c_can_dev(dev);

	return 0;
}

#ifdef CONFIG_PM
static int c_can_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int c_can_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define c_can_suspend NULL
#define c_can_resume NULL
#endif

static struct platform_driver c_can_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = c_can_of_table,
	},
	.probe = c_can_plat_probe,
	.remove = c_can_plat_remove,
	.suspend = c_can_suspend,
	.resume = c_can_resume,
	.id_table = c_can_id_table,
};

module_platform_driver(c_can_plat_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Platform CAN bus driver for Bosch C_CAN controller");

