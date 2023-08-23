/*
 * linux/driver/usb/host/ohci-nuc970.c
 *
 * Copyright (c) 2023 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <linux/clk.h>

#include <mach/map.h>

/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_nuc970_probe(const struct hc_driver *driver, struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	u32 val32[2];
	int retval;

	devm_pinctrl_get_select_default(&pdev->dev);

	if (of_property_read_u32_array(pdev->dev.of_node, "ov_active", val32, 1) == 0) {
		// printk("Over-current active level %s...\n", val32[0] ? "high" : "low");
		if (val32[0]) {
			/* set over-current active high */
			__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
		} else {
			/* set over-current active low */
			__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
		}
	}

	clk_prepare(clk_get(NULL, "usb_eclk"));
	clk_enable(clk_get(NULL, "usb_eclk"));

	/* enable USB Host clock */
	clk_prepare(clk_get(NULL, "usbh_hclk"));
	clk_enable(clk_get(NULL, "usbh_hclk"));

	/* enable PHY 0/1 */
	__raw_writel(0x160, (volatile void __iomem *)(NUC970_VA_EHCI+0xC4));
	__raw_writel(0x520, (volatile void __iomem *)(NUC970_VA_EHCI+0xC8));

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug("resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	retval = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (retval)
		goto err1;

	hcd = usb_create_hcd(driver, &pdev->dev, "nuc970-ohci");
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("ohci probe request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ohci error mapping memory\n");
		retval = -ENOMEM;
		goto err2;
	}

	retval = ohci_setup(hcd);
	if (retval)
		goto err2;

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

	if (retval == 0) {
		printk("NUC970 OHCI init done.\n");
		return retval;
	}

	pr_debug("Removing nuc970 OHCI USB Controller\n");

	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:

	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_nuc970_remove(struct usb_hcd *hcd,
                                  struct platform_device *dev)
{
	usb_remove_hcd(hcd);

	//pr_debug("stopping PPC-SOC USB Controller\n");

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

static const struct hc_driver ohci_nuc970_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"Nuvoton NUC970 OHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_DMA | HCD_MEMORY | HCD_BH,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_start,
	.stop =			ohci_stop,
	.shutdown = 		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

static int ohci_hcd_nuc970_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_nuc970_probe(&ohci_nuc970_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_nuc970_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_nuc970_remove(hcd, pdev);
	return 0;
}


#if defined(CONFIG_PM)

static int ohci_nuc970_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	int  ret;

	ret = ohci_suspend(hcd, do_wakeup);

	/* Suspend PHY0 and PHY1; this will turn off PHY power. */
	__raw_writel(0x60, NUC970_VA_EHCI+0xC4);
	__raw_writel(0x20, NUC970_VA_EHCI+0xC8);

	return ret;
}

static int ohci_nuc970_pm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	/* re-enable PHY0 and PHY1 */
	__raw_writel(0x160, NUC970_VA_EHCI+0xC4);
	__raw_writel(0x520, NUC970_VA_EHCI+0xC8);

	ohci_resume(hcd, false);

	return 0;
}
#else
#define ohci_nuc970_pm_suspend  NULL
#define ohci_nuc970_pm_resume   NULL
#endif

static const struct dev_pm_ops ohci_nuc970_dev_pm_ops = {
	.suspend         = ohci_nuc970_pm_suspend,
	.resume          = ohci_nuc970_pm_resume,
};


static const struct of_device_id nuc970_ohci_of_match[] = {
	{ .compatible = "nuvoton,nuc970-ohci" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_ohci_of_match);


static struct platform_driver ohci_hcd_nuc970_driver = {
	.probe      = ohci_hcd_nuc970_drv_probe,
	.remove     = ohci_hcd_nuc970_drv_remove,

	.driver     = {
		.name   = "nuc970-ohci",
		.pm     = &ohci_nuc970_dev_pm_ops,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_ohci_of_match),
	},
};
