/*
 *
 * Copyright (c) 2023 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  Description:
 *    Nuvoton LCD Controller Driver
 *  Author:
 *    Wang Qiang (rurality.linux@gmail.com) 2009/12/11
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <mach/regs-lcd.h>

#include "nuc970fb.h"

/*
 *  Initialize the nuc970 video (dual) buffer address
 */
static void nuc970fb_set_lcdaddr(struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;
	unsigned long vbaddr1, vbaddr2;

	vbaddr1  = info->fix.smem_start;
	vbaddr2  = info->fix.smem_start;
	vbaddr2 += info->fix.line_length * info->var.yres;

	/* set frambuffer start phy addr*/
	writel(vbaddr1, regs + REG_LCM_VA_BADDR0);
	writel(vbaddr2, regs + REG_LCM_VA_BADDR1);
	dev_dbg(fbi->dev, "vbaddr1   :0x%08x\n", readl(regs + REG_LCM_VA_BADDR0));
	dev_dbg(fbi->dev, "vbaddr2   :0x%08x\n", readl(regs + REG_LCM_VA_BADDR1));
	writel(fbi->regs.lcd_va_fbctrl, regs + REG_LCM_VA_FBCTRL);
	writel(fbi->regs.lcd_va_scale, regs + REG_LCM_VA_SCALE);
	dev_dbg(fbi->dev, "fbctrl    = 0x%08x\n", readl(regs + REG_LCM_VA_FBCTRL));
	dev_dbg(fbi->dev, "scale     = 0x%08x\n", readl(regs + REG_LCM_VA_SCALE));
}

/*
 *	Check the video params of 'var'.
 */
static int nuc970fb_check_var(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	struct nuc970fb_mach_info *mach_info = fbi->mach_info;
	struct nuc970fb_display *display = NULL;
	struct nuc970fb_display *default_display = mach_info->displays +
						   mach_info->default_display;
	int i;

	dev_dbg(fbi->dev, "check_var(var=%p, info=%p)\n", var, info);

	/* validate x/y resolution */
	/* choose default mode if possible */
	if (var->xres == default_display->xres &&
		var->yres == default_display->yres &&
		var->bits_per_pixel == default_display->bpp)
		display = default_display;
	else
		for (i = 0; i < mach_info->num_displays; i++)
			if (var->xres == mach_info->displays[i].xres &&
				var->yres == mach_info->displays[i].yres &&
				var->bits_per_pixel == mach_info->displays[i].bpp) {
				display = mach_info->displays + i;
				break;
			}

	if (display == NULL) {
		printk(KERN_ERR "wrong resolution or depth %dx%d at %d bit per pixel\n",
			var->xres, var->yres, var->bits_per_pixel);
		return -EINVAL;
	}

	/* it should be the same size as the display */
	var->xres_virtual	= display->xres;
#ifdef CONFIG_NUC970_DUAL_FB
	var->yres_virtual	= display->yres * 2;
#else
	var->yres_virtual	= display->yres;
#endif
	var->height		= display->height;
	var->width		= display->width;

	/* copy lcd settings */
	var->pixclock		= display->pixclock;
	var->left_margin	= display->left_margin;
	var->right_margin	= display->right_margin;
	var->upper_margin	= display->upper_margin;
	var->lower_margin	= display->lower_margin;
	var->vsync_len		= display->vsync_len;
	var->hsync_len		= display->hsync_len;

	var->transp.offset	= 0;
	var->transp.length	= 0;

	fbi->regs.lcd_dccs = display->dccs;

#ifdef CONFIG_NUC970_DUAL_FB
	fbi->regs.lcd_dccs |= LCM_DCCS_DISP_INT_EN;
#endif
	fbi->regs.lcd_device_ctrl = display->devctl;
	fbi->regs.lcd_va_fbctrl = display->fbctrl;
	fbi->regs.lcd_va_scale = display->scale;

	/* set R/G/B possions */
	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
	default:
		var->red.offset 	= 0;
		var->red.length 	= var->bits_per_pixel;
		var->green 		= var->red;
		var->blue		= var->red;
		break;
	case 12:
		var->red.length		= 4;
		var->green.length	= 4;
		var->blue.length	= 4;
		var->red.offset		= 8;
		var->green.offset	= 4;
		var->blue.offset	= 0;
		break;
	case 16:
		var->red.length		= 5;
		var->green.length	= 6;
		var->blue.length	= 5;
		var->red.offset		= 11;
		var->green.offset	= 5;
		var->blue.offset	= 0;
		break;
	case 18:
		var->red.length		= 6;
		var->green.length	= 6;
		var->blue.length	= 6;
		var->red.offset		= 12;
		var->green.offset	= 6;
		var->blue.offset	= 0;
		break;
	case 32:
		var->red.length		= 8;
		var->green.length	= 8;
		var->blue.length	= 8;
		var->red.offset		= 16;
		var->green.offset	= 8;
		var->blue.offset	= 0;
		break;
	}

	return 0;
}

/*
 *	Calculate lcd register values from var setting & save into hw
 */
static void nuc970fb_calculate_lcd_regs(const struct fb_info *info,
					struct nuc970fb_hw *regs)
{
	const struct fb_var_screeninfo *var = &info->var;
	int vtt = var->height + var->upper_margin + var->lower_margin;
	int htt = var->width + var->left_margin + var->right_margin;
	int hsync = var->width + var->right_margin;
	int vsync = var->height + var->lower_margin;

	regs->lcd_crtc_size = LCM_CRTC_SIZE_VTTVAL(vtt) |
				  LCM_CRTC_SIZE_HTTVAL(htt);

	regs->lcd_crtc_dend = LCM_CRTC_DEND_VDENDVAL(var->height) |
				  LCM_CRTC_DEND_HDENDVAL(var->width);
	regs->lcd_crtc_hr = LCM_CRTC_HR_EVAL(var->width + 5) |
				LCM_CRTC_HR_SVAL(var->width + 1);
	regs->lcd_crtc_hsync = LCM_CRTC_HSYNC_EVAL(hsync + var->hsync_len) |
				   LCM_CRTC_HSYNC_SVAL(hsync);
	regs->lcd_crtc_vr = LCM_CRTC_VR_EVAL(vsync + var->vsync_len) |
				LCM_CRTC_VR_SVAL(vsync);

#if 0
	printk("%d\tvtt = height + upper_margin + lower_margin\n", vtt);
	printk("%d\tvsync = height + lower_margin\n", vsync);
	printk("%d\tvtt = width + left_margin + right_margin\n", htt);
	printk("%d\thsync = width + right_margin\n", hsync);

	printk("crtc_size :0x%08x\n", regs->lcd_crtc_size);
	printk("crtc_dend :0x%08x\n", regs->lcd_crtc_dend);
	printk("crtc_vr   :0x%08x\n", regs->lcd_crtc_hr);
	printk("crtc_hsync:0x%08x\n", regs->lcd_crtc_hsync);
	printk("crtc_vr   :0x%08x\n", regs->lcd_crtc_vr);

	printk("vtt(crtc_size[26:16]),htt(crtc_size[10:0]): 0x%08x(%d), 0x%08x(%d)\n", vtt, vtt, htt, htt);
	printk("vdend(lcd_crtc_dend[26:16]),hdend(lcd_crtc_dend[10:0]): 0x%08x(%d), 0x%08x(%d)\n", var->height, var->height, var->width, var->width);
	printk("hre(lcd_crtc_hr[26:16]+5), hrs(lcd_crtc_hr[10:0]+1): 0x%08x(%d), 0x%08x(%d)\n", var->width + 5, var->width + 5, var->width + 1, var->width + 1);
	printk("hsync_e(lcd_crtc_hsync[26:16]),hsync_s(lcd_crtc_hsync[10:0]): 0x%08x(%d), 0x%08x(%d)\n", hsync + var->hsync_len, hsync + var->hsync_len, hsync, hsync);
	printk("vre(lcd_crtc_vr[26:16]),vrs(lcd_crtc_vr[10:0]):0x%08x(%d),0x%08x(%d)\n",vsync + var->vsync_len, vsync + var->vsync_len, vsync, vsync);
#endif
}

/*
 *	Activate (set) the controller from the given framebuffer
 *	information
 */
static void nuc970fb_activate_var(struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;

	nuc970fb_calculate_lcd_regs(info, &fbi->regs);

	/* set the new lcd registers*/
	dev_dbg(fbi->dev, "new lcd register set:\n");
	dev_dbg(fbi->dev, "dccs       = [ 0x%08x ]\n", fbi->regs.lcd_dccs);
	dev_dbg(fbi->dev, "dev_ctl    = [ 0x%08x ]\n", fbi->regs.lcd_device_ctrl);
	dev_dbg(fbi->dev, "crtc_size  = [ 0x%08x ]\n", fbi->regs.lcd_crtc_size);
	dev_dbg(fbi->dev, "crtc_dend  = [ 0x%08x ]\n", fbi->regs.lcd_crtc_dend);
	dev_dbg(fbi->dev, "crtc_hr    = [ 0x%08x ]\n", fbi->regs.lcd_crtc_hr);
	dev_dbg(fbi->dev, "crtc_hsync = [ 0x%08x ]\n", fbi->regs.lcd_crtc_hsync);
	dev_dbg(fbi->dev, "crtc_vr    = [ 0x%08x ]\n", fbi->regs.lcd_crtc_vr);

	writel(fbi->regs.lcd_device_ctrl, regs + REG_LCM_DEV_CTRL);
	writel(fbi->regs.lcd_crtc_size, regs + REG_LCM_CRTC_SIZE);
	writel(fbi->regs.lcd_crtc_dend, regs + REG_LCM_CRTC_DEND);
	writel(fbi->regs.lcd_crtc_hr, regs + REG_LCM_CRTC_HR);
	writel(fbi->regs.lcd_crtc_hsync, regs + REG_LCM_CRTC_HSYNC);
	writel(fbi->regs.lcd_crtc_vr, regs + REG_LCM_CRTC_VR);

	/* set lcd address pointers */
	nuc970fb_set_lcdaddr(info);

	writel(fbi->regs.lcd_dccs, regs + REG_LCM_DCCS);
}

/*
 * Alters the hardware state.
 */
static int nuc970fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
	case 32:
	case 24:
	case 18:
	case 16:
	case 12:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
		info->fix.visual = FB_VISUAL_MONO01;
		break;
	default:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}

	info->fix.line_length = (var->xres_virtual * var->bits_per_pixel) / 8;

	/* activate this new configuration */
	nuc970fb_activate_var(info);
	return 0;
}

static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int nuc970fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			      unsigned blue, unsigned transp, struct fb_info *info)
{
	unsigned int val;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseuo-palette */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);
			pal[regno] = val;
		}
		break;

	default:
		return 1;   /* unknown type */
	}
	return 0;
}

/**
 *      nuc970fb_blank
 *
 */
static int nuc970fb_blank(int blank_mode, struct fb_info *info)
{

	return 0;
}


#ifdef CONFIG_NUC970_DUAL_FB
static int nuc970fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	unsigned long flags;

	spin_lock_irqsave(&fbi->lock, flags);

	fbi->dual_fb_base = var->yoffset * info->fix.line_length;

	spin_unlock_irqrestore(&fbi->lock, flags);

	//printk("pan_display: [0x%x],  %d, %d - %d, %d\n", fbi->dual_fb_base, var->xoffset, var->yoffset, info->var.xoffset, info->var.yoffset);
	//printk("[0x%x, 0x%x]\n", readl(fbi->io + REG_LCM_DCCS), readl(fbi->io +  REG_LCM_INT_CS));
	return 0;
}
#endif  /* CONFIG_NUC970_DUAL_FB */


static struct fb_ops nuc970fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= nuc970fb_check_var,
	.fb_set_par	= nuc970fb_set_par,
	.fb_blank	= nuc970fb_blank,
#ifdef CONFIG_NUC970_DUAL_FB
	.fb_pan_display = nuc970fb_pan_display,
#endif
	.fb_setcolreg	= nuc970fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static inline void modify_gpio(void __iomem *reg,
				   unsigned long set, unsigned long mask)
{
	unsigned long tmp;
	tmp = readl(reg) & ~mask;
	writel(tmp | set, reg);
}

/*
 * Initialise LCD-related registers
 */
static int nuc970fb_init_registers(struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;

	/*reset the display engine*/
	writel(0, regs + REG_LCM_DCCS);
	writel(readl(regs + REG_LCM_DCCS) | LCM_DCCS_ENG_RST,
		   regs + REG_LCM_DCCS);
	ndelay(100);
	writel(readl(regs + REG_LCM_DCCS) & (~LCM_DCCS_ENG_RST),
		   regs + REG_LCM_DCCS);
	ndelay(100);

	writel(0, regs + REG_LCM_DEV_CTRL);

	return 0;
}

/*
 *    Alloc the SDRAM region of nuc970 for the frame buffer.
 *    The buffer should be a non-cached, non-buffered, memory region
 *    to allow palette and pixel writes without flushing the cache.
 */
static int nuc970fb_map_video_memory(struct platform_device *pdev, struct fb_info *info)
{
	struct device_node *np;
	struct resource r;
	ssize_t size;
	int ret;

	/* Get reserved memory region from Device-tree */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		return -ENOENT;
	}

	ret = of_address_to_resource(np, 0, &r);
	if (ret) {
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		return -ENOENT;
	}

	info->fix.smem_start = r.start;
	size = resource_size(&r);
	info->screen_base = memremap(r.start, size, MEMREMAP_WB);
	if (!info->screen_base) {
		pr_err("%s: Failed to memremap framebuffer\n", __func__);
		return -ENOMEM;
	}

	memset(info->screen_base, 0x00, size);

	dev_info(&pdev->dev, "Reserved fb memory at 0x%lx, 0x%x bytes, mapped to 0x%x\n",
		 info->fix.smem_start, size, (int)info->screen_base);
	return 0;
}

static inline void nuc970fb_unmap_video_memory(struct fb_info *info)
{
	memunmap(info->screen_base);
}

static irqreturn_t nuc970fb_irqhandler(int irq, void *dev_id)
{
	struct nuc970fb_info *fbi = dev_id;
	void __iomem *regs = fbi->io;
	void __iomem *irq_base = fbi->irq_base;
	unsigned long lcdirq = readl(regs + REG_LCM_INT_CS);

//	printk("[0x%x, 0x%x] lcdirq = 0x%x\n", readl(fbi->io + REG_LCM_DCCS), readl(fbi->io +  REG_LCM_INT_CS), lcdirq);
	if (lcdirq & LCM_INT_CS_DISP_F_STATUS)
	{
		writel(readl(irq_base) | 1<<30, irq_base);
#ifdef CONFIG_PM
		if(fbi->powerdown) {
			complete(&fbi->completion);
		}
#endif
		/* wait VA_EN low */
		if ((readl(regs + REG_LCM_DCCS) &
			LCM_DCCS_SINGLE) == LCM_DCCS_SINGLE)
			while ((readl(regs + REG_LCM_DCCS) &
				   LCM_DCCS_VA_EN) == LCM_DCCS_VA_EN);

		/* display_out-enable */
		writel(readl(regs + REG_LCM_DCCS) | LCM_DCCS_DISP_OUT_EN,
			regs + REG_LCM_DCCS);
		/* va-enable*/
		writel(readl(regs + REG_LCM_DCCS) | LCM_DCCS_VA_EN,
			regs + REG_LCM_DCCS);

#ifdef CONFIG_NUC970_DUAL_FB
		if (fbi->dual_fb_base == 0)
		{
			/* Starting fetch data from VA_BADDR0 */
			writel(readl(regs + REG_LCM_VA_FBCTRL) & ~LCM_VA_FBCTRL_START_BUF, regs + REG_LCM_VA_FBCTRL);
		}
		else
		{
			/* Starting fetch data from VA_BADDR1 */
			writel(readl(regs + REG_LCM_VA_BADDR0) + fbi->dual_fb_base, regs + REG_LCM_VA_BADDR1);
			writel(readl(regs + REG_LCM_VA_FBCTRL) | LCM_VA_FBCTRL_START_BUF, regs + REG_LCM_VA_FBCTRL);
		}
#endif

	}
	else if (lcdirq & LCM_INT_CS_UNDERRUN_INT)
	{
		writel(readl(irq_base) | LCM_INT_CS_UNDERRUN_INT, irq_base);
	}
	else if (lcdirq & LCM_INT_CS_BUS_ERROR_INT)
	{
		writel(readl(irq_base) | LCM_INT_CS_BUS_ERROR_INT, irq_base);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_CPU_FREQ

static int nuc970fb_cpufreq_transition(struct notifier_block *nb,
					   unsigned long val, void *data)
{
	struct nuc970fb_info *info;
	struct fb_info *fbinfo;
	long delta_f;
	info = container_of(nb, struct nuc970fb_info, freq_transition);
	fbinfo = platform_get_drvdata(to_platform_device(info->dev));

	delta_f = info->clk_rate - clk_get_rate(info->clk);

	if ((val == CPUFREQ_POSTCHANGE && delta_f > 0) ||
	   (val == CPUFREQ_PRECHANGE && delta_f < 0)) {
		info->clk_rate = clk_get_rate(info->clk);
		nuc970fb_activate_var(fbinfo);
	}

	return 0;
}

static inline int nuc970fb_cpufreq_register(struct nuc970fb_info *fbi)
{
	fbi->freq_transition.notifier_call = nuc970fb_cpufreq_transition;
	return cpufreq_register_notifier(&fbi->freq_transition,
				  CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void nuc970fb_cpufreq_deregister(struct nuc970fb_info *fbi)
{
	cpufreq_unregister_notifier(&fbi->freq_transition,
					CPUFREQ_TRANSITION_NOTIFIER);
}
#else
static inline int nuc970fb_cpufreq_transition(struct notifier_block *nb,
					   unsigned long val, void *data)
{
	return 0;
}

static inline int nuc970fb_cpufreq_register(struct nuc970fb_info *fbi)
{
	return 0;
}

static inline void nuc970fb_cpufreq_deregister(struct nuc970fb_info *info)
{
}
#endif

static char driver_name[] = "nuc970fb";

#ifdef CONFIG_OF
static struct nuc970fb_mach_info *nuc970fb_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970fb_mach_info *mach_info;
	struct nuc970fb_display *display;
	u32 temp;

	mach_info = devm_kzalloc(dev, sizeof(*mach_info), GFP_KERNEL);
	if (!mach_info) {
		dev_err(dev, "memory allocation for fb_info failed\n");
		return ERR_PTR(-ENOMEM);
	}

	display = devm_kzalloc(dev, sizeof(*display), GFP_KERNEL);
	if (!display) {
		dev_err(dev, "memory allocation for fb_info failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mach_info->displays = display;
	mach_info->num_displays = 1;//fixed to only 1 display
	mach_info->default_display = 0;

	if (of_property_read_u32(dev->of_node, "type", &temp)) {
		dev_warn(dev, "can't get type from dt\n");
		goto read_error;
	} else {
		display->type = temp;
	}

	if (of_property_read_u32(dev->of_node, "bpp", &temp)) {
		dev_warn(dev, "can't get bpp from dt\n");
		goto read_error;
	} else {
		display->bpp = temp;
	}

	if (of_property_read_u32(dev->of_node, "width", &temp)) {
		dev_warn(dev, "can't get width from dt\n");
		goto read_error;
	} else {
		display->width = temp;
	}

	if (of_property_read_u32(dev->of_node, "height", &temp)) {
		dev_warn(dev, "can't get height from dt\n");
		goto read_error;
	} else {
		display->height = temp;
	}

	if (of_property_read_u32(dev->of_node, "xres", &temp)) {
		dev_warn(dev, "can't get xres from dt\n");
		goto read_error;
	} else {
		display->xres = temp;
	}

	if (of_property_read_u32(dev->of_node, "yres", &temp)) {
		dev_warn(dev, "can't get yres from dt\n");
		goto read_error;
	} else {
		display->yres = temp;
	}

	if (of_property_read_u32(dev->of_node, "pixclock", &temp)) {
		dev_warn(dev, "can't get pixclock from dt\n");
		goto read_error;
	} else {
		display->pixclock = temp;
	}

	if (of_property_read_u32(dev->of_node, "left_margin", &temp)) {
		dev_warn(dev, "can't get left_margin from dt\n");
		goto read_error;
	} else {
		display->left_margin = temp;
	}

	if (of_property_read_u32(dev->of_node, "right_margin", &temp)) {
		dev_warn(dev, "can't get right_margin from dt\n");
		goto read_error;
	} else {
		display->right_margin = temp;
	}

	if (of_property_read_u32(dev->of_node, "hsync_len", &temp)) {
		dev_warn(dev, "can't get hsync_len from dt\n");
		goto read_error;
	} else {
		display->hsync_len = temp;
	}

	if (of_property_read_u32(dev->of_node, "upper_margin", &temp)) {
		dev_warn(dev, "can't get upper_margin from dt\n");
		goto read_error;
	} else {
		display->upper_margin = temp;
	}

	if (of_property_read_u32(dev->of_node, "lower_margin", &temp)) {
		dev_warn(dev, "can't get lower_margin from dt\n");
		goto read_error;
	} else {
		display->lower_margin = temp;
	}

	if (of_property_read_u32(dev->of_node, "vsync_len", &temp)) {
		dev_warn(dev, "can't get vsync_len from dt\n");
		goto read_error;
	} else {
		display->vsync_len = temp;
	}

	if (of_property_read_u32(dev->of_node, "dccs", &temp)) {
		dev_warn(dev, "can't get dccs from dt\n");
		goto read_error;
	} else {
		display->dccs = temp;
	}

	if (of_property_read_u32(dev->of_node, "fbctrl", &temp)) {
		dev_warn(dev, "can't get fbctrl from dt\n");
		goto read_error;
	} else {
		display->fbctrl = temp;
	}

	if (of_property_read_u32(dev->of_node, "devctl", &temp)) {
		dev_warn(dev, "can't get devctl from dt\n");
		goto read_error;
	} else {
		display->devctl = temp;
	}

	if (of_property_read_u32(dev->of_node, "scale", &temp)) {
		dev_warn(dev, "can't get scale from dt\n");
		goto read_error;
	} else {
		display->scale = temp;
	}

	if (of_property_read_u32(dev->of_node, "gpio_blen", &temp)) {
		dev_warn(dev, "can't get gpio_blen from dt\n");
		goto read_error;
	} else {
		mach_info->gpio_blen = temp;
	}

	if (of_property_read_u32(dev->of_node, "gpio_lcs", &temp)) {
		dev_warn(dev, "can't get gpio_lcs from dt\n");
		goto read_error;
	} else {
		mach_info->gpio_lcs = temp;
	}

	dev_dbg(dev, "%dx%d type:%d, bpp:%d\n", display->width, display->height, display->type, display->bpp);
	dev_dbg(dev, "%dx%d @%ld Hz\n", display->xres, display->yres, display->pixclock);

	dev_dbg(dev, "left_margin:%d\n", display->left_margin);
	dev_dbg(dev, "right_margin:%d\n", display->right_margin);
	dev_dbg(dev, "hsync_len:%d\n", display->hsync_len);
	dev_dbg(dev, "upper_margin:%d\n", display->upper_margin);
	dev_dbg(dev, "lower_margin:%d\n", display->lower_margin);
	dev_dbg(dev, "vsync_len:%d\n", display->vsync_len);

	return mach_info;

read_error:
	devm_kfree(dev, display);
	devm_kfree(dev, mach_info);

	return ERR_PTR(-EINVAL);
}
#else
static struct nuc970fb_mach_info *nuc970fb_parse_dt(struct device *dev)
{
	return dev->platform_data;
}
#endif

static int nuc970fb_probe(struct platform_device *pdev)
{
	struct nuc970fb_info *fbi;
	struct nuc970fb_display *display;
	struct fb_info *fbinfo;
	struct nuc970fb_mach_info *mach_info;
	struct resource *res;
	int ret;
	int irq;
	int i;
	int size;
	struct clk *clkmux, *clkuplldiv;
	struct pinctrl *pinctrl;

	dev_dbg(&pdev->dev, "devinit\n");

	mach_info = nuc970fb_parse_dt(pdev);
	if (mach_info == NULL) {
		dev_err(&pdev->dev,
			"no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}
	//printk("[%s] %x\n", __func__, mach_info->displays[0].bpp);

	if (mach_info->default_display > mach_info->num_displays) {
		dev_err(&pdev->dev,
			"default display No. is %d but only %d displays \n",
			mach_info->default_display, mach_info->num_displays);
		return -EINVAL;
	}

	display = mach_info->displays + mach_info->default_display;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for device\n");
		return -ENOENT;
	}

	fbinfo = framebuffer_alloc(sizeof(struct nuc970fb_info), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;

	platform_set_drvdata(pdev, fbinfo);

	fbi = fbinfo->par;
	fbi->dev = &pdev->dev;
	fbi->mach_info = mach_info;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	size = resource_size(res);
	fbi->mem = request_mem_region(res->start, size, pdev->name);
	if (fbi->mem == NULL) {
		dev_err(&pdev->dev, "failed to alloc memory region\n");
		ret = -ENOENT;
		goto free_fb;
	}

	fbi->io = ioremap(res->start, size);
	if (fbi->io == NULL) {
		dev_err(&pdev->dev, "ioremap() of lcd registers failed\n");
		ret = -ENXIO;
		goto release_mem_region;
	}

	fbi->irq_base = fbi->io + REG_LCM_INT_CS;

	spin_lock_init(&fbi->lock);
	fbi->dual_fb_base = 0;

	/* Stop the LCD */
	writel(0, fbi->io + REG_LCM_DCCS);

	/* fill the fbinfo*/
	strcpy(fbinfo->fix.id, driver_name);
	fbinfo->fix.type		= FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux		= 0;
	fbinfo->fix.xpanstep		= 0;
#ifdef CONFIG_NUC970_DUAL_FB
	fbinfo->fix.ypanstep		= 1;
#else
	fbinfo->fix.ypanstep		= 0;
#endif
	fbinfo->fix.ywrapstep		= 0;
	fbinfo->fix.accel		= FB_ACCEL_NONE;
	fbinfo->var.nonstd		= 0;
	fbinfo->var.activate		= FB_ACTIVATE_NOW|FB_ACTIVATE_FORCE;
	fbinfo->var.accel_flags		= 0;
	fbinfo->var.vmode		= FB_VMODE_NONINTERLACED;
	fbinfo->fbops			= &nuc970fb_ops;
	fbinfo->flags			= FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette		= &fbi->pseudo_pal;
#ifdef CONFIG_PM
	fbi->powerdown			= 0;
#endif
#ifdef CONFIG_PM
	init_completion(&fbi->completion);
#endif
	ret = request_irq(irq, nuc970fb_irqhandler, 0,
			  pdev->name, fbi);
	if (ret) {
		dev_err(&pdev->dev, "cannot register irq handler %d -err %d\n",
			irq, ret);
		ret = -EBUSY;
		goto release_regs;
	}

	clk_prepare(clk_get(NULL, "lcd_hclk"));
	clk_enable(clk_get(NULL, "lcd_hclk"));
	fbi->clk = clk_get(NULL, "lcd_eclk");

	if(display->pixclock > 12000000)
	{
		// change clock source to upll
		clkmux = clk_get(NULL, "lcd_eclk_mux");
		if (IS_ERR(clkmux)) {
			printk(KERN_ERR "Failed to get lcd clock mux control\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}

		clkuplldiv = clk_get(NULL, "lcd_uplldiv");
		if (IS_ERR(clkuplldiv)) {
			printk(KERN_ERR "Failed to get lcd clock divider control\n");
			ret = PTR_ERR(clkuplldiv);
			return ret;
		}

		// select lcd clock from upll
		clk_set_parent(clkmux, clkuplldiv);
	}

	clk_set_rate(fbi->clk, display->pixclock);

	clk_prepare(fbi->clk);
	clk_enable(fbi->clk);
	fbi->clk_rate = clk_get_rate(fbi->clk);
	dev_dbg(&pdev->dev, "got and enabled clock @%ld Hz\n", fbi->clk_rate);

	/* calutate the video buffer size */
	for (i = 0; i < mach_info->num_displays; i++) {
		unsigned long smem_len = mach_info->displays[i].xres;
		smem_len *= mach_info->displays[i].yres;
		smem_len *= mach_info->displays[i].bpp;
		smem_len >>= 3;
		if (fbinfo->fix.smem_len < smem_len)
			fbinfo->fix.smem_len = smem_len;
	}

#ifdef CONFIG_NUC970_DUAL_FB
	fbinfo->fix.smem_len *= 2;
#endif

	/* Initialize Video Memory */
	ret = nuc970fb_map_video_memory(pdev, fbinfo);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate video RAM: %x\n", ret);
		goto release_clock;
	}

	dev_dbg(&pdev->dev, "got video memory\n");

	fbinfo->var.xres = display->xres;
	fbinfo->var.yres = display->yres;
	fbinfo->var.bits_per_pixel = display->bpp;

	nuc970fb_init_registers(fbinfo);

	nuc970fb_check_var(&fbinfo->var, fbinfo);

	ret = fb_set_var(fbinfo, &fbinfo->var);
	if (ret) {
		dev_warn(&pdev->dev, "unable to set display parameters\n");
		goto free_video_memory;
	}

	ret = nuc970fb_cpufreq_register(fbi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register cpufreq\n");
		goto free_video_memory;
	}

	ret = register_framebuffer(fbinfo);
	if (ret) {
		printk(KERN_ERR "failed to register framebuffer device: %d\n",
			ret);
		goto free_cpufreq;
	}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}

#if !defined(CONFIG_FRAMEBUFFER_CONSOLE) && defined(CONFIG_LOGO)
			if (fb_prepare_logo(fbinfo, FB_ROTATE_UR)) {
				/* Start display and show logo on boot */
				fb_set_cmap(&fbinfo->cmap, fbinfo);
				fb_show_logo(fbinfo, FB_ROTATE_UR);
			}
#endif

	dev_info(&pdev->dev, "fb%d: FrameBuffer device\n",
		fbinfo->node);

#if defined(CONFIG_PM) || defined(CONFIG_NUC970_DUAL_FB)
	writel(readl(fbi->io + REG_LCM_DCCS) | LCM_DCCS_DISP_INT_EN, fbi->io + REG_LCM_DCCS);
	writel(readl(fbi->io +  REG_LCM_INT_CS) | LCM_INT_CS_DISP_F_EN, fbi->io + REG_LCM_INT_CS);
#endif
	return 0;

free_cpufreq:
	nuc970fb_cpufreq_deregister(fbi);
free_video_memory:
	nuc970fb_unmap_video_memory(fbinfo);
release_clock:
	clk_disable(fbi->clk);
	clk_put(fbi->clk);
	free_irq(irq, fbi);
release_regs:
	iounmap(fbi->io);
release_mem_region:
	release_mem_region(res->start, size);
free_fb:
	framebuffer_release(fbinfo);
	return ret;
}

/*
 * shutdown the lcd controller
 */
static void nuc970fb_stop_lcd(struct fb_info *info)
{
	struct nuc970fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;

	writel((~LCM_DCCS_DISP_INT_EN) | (~LCM_DCCS_VA_EN) | (~LCM_DCCS_OSD_EN),
		regs + REG_LCM_DCCS);
}

/*
 *  Cleanup
 */
static int nuc970fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct nuc970fb_info *fbi = fbinfo->par;
	int irq;

	nuc970fb_stop_lcd(fbinfo);
	msleep(1);

	unregister_framebuffer(fbinfo);
	nuc970fb_cpufreq_deregister(fbi);
	nuc970fb_unmap_video_memory(fbinfo);

	iounmap(fbi->io);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, fbi);

	release_resource(fbi->mem);
	kfree(fbi->mem);

#if defined(CONFIG_OF)
	devm_kfree(&pdev->dev, fbi->mach_info->displays);
	devm_kfree(&pdev->dev, fbi->mach_info);
#endif

	platform_set_drvdata(pdev, NULL);
	framebuffer_release(fbinfo);

	return 0;
}

#ifdef CONFIG_PM
/*
 *	suspend and resume support for the lcd controller
 */
#define NUC970_FB_TIMEOUT	(msecs_to_jiffies(5000))
static int nuc970fb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct nuc970fb_info *info = fbinfo->par;
	unsigned long timeout;

	printk(KERN_INFO "fb suspend\n");
	info->powerdown = 1;

	timeout = wait_for_completion_interruptible_timeout
			 (&info->completion, NUC970_FB_TIMEOUT);

	nuc970fb_stop_lcd(fbinfo);

	return 0;
}

static int nuc970fb_resume(struct platform_device *dev)
{
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct nuc970fb_info *info = fbinfo->par;

	printk(KERN_INFO "fb resume\n");

	info->powerdown = 0;

	nuc970fb_init_registers(fbinfo);
	nuc970fb_activate_var(fbinfo);

	writel(readl(info->io + REG_LCM_DCCS) | LCM_DCCS_DISP_INT_EN, info->io + REG_LCM_DCCS);
	writel(readl(info->io +  REG_LCM_INT_CS) | LCM_INT_CS_DISP_F_EN, info->io + REG_LCM_INT_CS);

	return 0;
}

#else
#define nuc970fb_suspend NULL
#define nuc970fb_resume  NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id nuc970_lcd_of_match[] = {
	{   .compatible = "nuvoton,nuc970-lcd"},
	{  },
};
MODULE_DEVICE_TABLE(of, nuc970_lcd_of_match);
#endif

static struct platform_driver nuc970fb_driver = {
	.probe		= nuc970fb_probe,
	.remove		= nuc970fb_remove,
	.suspend	= nuc970fb_suspend,
	.resume		= nuc970fb_resume,
	.driver		= {
		.name	= "nuc970-lcd",
		.owner	= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc970_lcd_of_match),
#endif
	},
};

module_platform_driver(nuc970fb_driver);

MODULE_DESCRIPTION("Framebuffer driver for the nuc970/N9H30");
MODULE_LICENSE("GPL");
