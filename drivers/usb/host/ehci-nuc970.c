/*
 * linux/driver/usb/host/ehci-nuc970.c
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
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>

#include <linux/clk.h>
#include <mach/irqs.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include <mach/regs-timer.h>
#include <mach/regs-gpio.h>

//#define PORT_DEBUG

#ifdef PORT_DEBUG
#include <linux/kthread.h>
static int port_dump_thread(void *__unused)
{
	while (1) {
		printk("EHCI: 0x%x 0x%x\n", __raw_readl(NUC970_VA_EHCI+0x64), __raw_readl(NUC970_VA_EHCI+0x68));
		printk("OHCI: 0x%x 0x%x\n", __raw_readl(NUC970_VA_OHCI+0x54), __raw_readl(NUC970_VA_OHCI+0x58));
		msleep(20000);
	}
	return 0;
}
#endif

static int usb_nuc970_probe(const struct hc_driver *driver,
			    struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	u32  physical_map_ehci;
	int retval;
	u32   val32[2];

	if (IS_ERR(clk_get(NULL, "usbh_hclk"))) {
		printk("clk_get error!!\n");
		return -1;
	}

	/* Enable USB Host clock */
	clk_prepare(clk_get(NULL, "usb_eclk"));
	clk_enable(clk_get(NULL, "usb_eclk"));

	clk_prepare(clk_get(NULL, "usbh_hclk"));
	clk_enable(clk_get(NULL, "usbh_hclk"));

	if (of_property_read_u32_array(pdev->dev.of_node, "ov_active", val32, 1) != 0) {
		printk("%s - can not get ov_active setting!\n", __func__);
		return -EINVAL;
	}
	printk("NUC970 EHCI over-current active level %s...\n", val32[0] ? "high" : "low");

	if (val32[0]) {
		/* set over-current active high */
		__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) & ~0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
	} else {
		/* set over-current active low */
		__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
	}

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

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("ehci probe request_mem_region failed");
		retval = -EBUSY;
		goto err2;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		pr_debug("ehci error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + 0x20;

	/* enable PHY 0/1 */
	physical_map_ehci = (u32)ehci->caps;
	__raw_writel(0x160, (volatile void __iomem *)physical_map_ehci+0xC4);
	__raw_writel(0x520, (volatile void __iomem *)physical_map_ehci+0xC8);

	//__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0xfc0000, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);
	ehci->sbrn = 0x20;

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

	if (retval != 0)
		goto err4;

#ifdef PORT_DEBUG
	kthread_run(port_dump_thread, NULL, "khubd");
#endif
	printk("NUC970 EHCI init done.\n");
	return retval;

err4:
	iounmap(hcd->regs);
err3:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
	usb_put_hcd(hcd);
err1:
	return retval;
}

void usb_nuc970_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}


static const struct hc_driver ehci_nuc970_hc_driver = {
	.description = hcd_name,
	.product_desc = "Nuvoton NUC970 EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_MEMORY | HCD_DMA | HCD_USB2 | HCD_BH,
	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_setup,
	.start =		ehci_run,
	.stop =			ehci_stop,
	.shutdown =		ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,
	.endpoint_reset =	ehci_endpoint_reset,
	.clear_tt_buffer_complete =	ehci_clear_tt_buffer_complete,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
	.relinquish_port =	ehci_relinquish_port,
	.port_handed_over =	ehci_port_handed_over,
	.get_resuming_ports =	ehci_get_resuming_ports,

	/*
	 * device support
	 */
	.free_dev =		ehci_remove_device,

#ifdef  CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
};

static int ehci_nuc970_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_nuc970_probe(&ehci_nuc970_hc_driver, pdev);
}

static int ehci_nuc970_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_nuc970_remove(hcd, pdev);

	return 0;
}

#ifdef CONFIG_PM
static int ehci_nuc970_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	int  ret;

	ret = ehci_suspend(hcd, do_wakeup);

#if !defined(CONFIG_USB_NUC970_OHCI)
	/* Suspend PHY0 and PHY1; this will turn off PHY power. */
	/* If NUC970 OHCI enabled, this job will be left to NUC970 OHCI driver. */
	__raw_writel(0x60, NUC970_VA_EHCI+0xC4);
	__raw_writel(0x20, NUC970_VA_EHCI+0xC8);
#endif
	return ret;
}

static int ehci_nuc970_pm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	/* re-enable PHY0 and PHY1 */
	__raw_writel(0x160, NUC970_VA_EHCI+0xC4);
	__raw_writel(0x520, NUC970_VA_EHCI+0xC8);

	ehci_resume(hcd, false);

	return 0;
}
#else
#define ehci_nuc970_pm_suspend  NULL
#define ehci_nuc970_pm_resume   NULL
#endif

static const struct dev_pm_ops ehci_nuc970_dev_pm_ops = {
	.suspend         = ehci_nuc970_pm_suspend,
	.resume          = ehci_nuc970_pm_resume,
};


static const struct of_device_id nuc970_ehci_of_match[] = {
	{ .compatible = "nuvoton,nuc970-ehci" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_ehci_of_match);


static struct platform_driver ehci_hcd_nuc970_driver = {
	.probe = ehci_nuc970_probe,
	.remove = ehci_nuc970_remove,
	.driver = {
		.name = "nuc970-ehci",
		.pm = &ehci_nuc970_dev_pm_ops,
		.owner= THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_ehci_of_match),
	},
};

