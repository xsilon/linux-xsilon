/*
 * FT313 HCD (Host Controller Driver) Platform Driver for FTDI FT313H.
 *
 * Copyright (C) 2011 Chang Yang <chang.yang@ftdichip.com>
 *
 * This code is *strongly* based on EHCI-HCD code by David Brownell since
 * the chip is a quasi-EHCI compatible.
 *
 * Licensed under GPL version 2 only.
 */

/* this file is part of ft313-hcd.c */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/ulpi.h>
//#include <plat/usb.h>
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

void* g_old_dma_mask = NULL;
//static struct platform_driver ft313_han9250_evm_driver;



/* Called during probe() after chip reset completes.
 */
static int ft313_platform_setup(struct usb_hcd *hcd)
{
	struct ft313_hcd *ft313 = hcd_to_ft313(hcd);
	int retval;
	u32 temp32;
	u16 tmp;

	printk(KERN_NOTICE "FT313: Setup\n");
	spin_lock_init(&ft313->reg_lock);
	ft313->cfg = hcd->regs + FT313_CONFIG_OFFSET;

	/* FixMe: */
	// Some HW init function like HW reset, set chip mode (16/8 bit)
	// Interrupt setting (edge trigger or level trigger etc.)
	// may needed here!
#ifdef FT313_IN_8_BIT_MODE
	// Dummy read to wakeup chip!
	ioread8(&ft313->cfg->sw_reset);
	mdelay(10);

	u8 temp;
	iowrite8(RESET_ALL, &ft313->cfg->sw_reset); // Reset FT313
	mdelay(200);
	temp = ioread8(&ft313->cfg->sw_reset);
	temp |= DATA_BUS_WIDTH;
	temp &= ~(RESET_ATX | RESET_HC | RESET_ALL);
	iowrite8(temp, &ft313->cfg->sw_reset);

#else
	// Dummy read to wakeup chip!
	ft313_reg_read16(ft313, &ft313->cfg->sw_reset);
	mdelay(10);

	//iowrite16(RESET_ALL, &ft313->cfg->sw_reset);
	ft313_reg_write16(ft313, RESET_ALL, &ft313->cfg->sw_reset);
	mdelay(200);

	ft313_reg_read16(ft313, &ft313->cfg->sw_reset);
#endif

	tmp = ft313_reg_read16(ft313, &ft313->cfg->hw_mode);
	ft313_reg_write16(ft313,
			  tmp | DACK_POL| INTF_LOCK | INTR_POL | INTR_EDGE | GLOBAL_INTR_EN, //INTR_EDGE use level interrupt
			  &ft313->cfg->hw_mode); // Enable global interrupt
	tmp = ft313_reg_read16(ft313, &ft313->cfg->hw_mode);
	printk(KERN_NOTICE "FT313: HWMODE 0x%04x\n", tmp);


	//Turn VBUS on and set BCD mode
	tmp = ft313_reg_read16(ft313, &ft313->cfg->config);
	DEBUG_MSG("bcd_mode is %s\n", bcd_mode);

	if (!strcmp(bcd_mode, "Disable")) {
		tmp &= ~BCD_EN; // Disable BCD
	} else if (!strcmp(bcd_mode, "Enable")) {
		tmp |= BCD_EN; // Enable BCD, actual mode setting by BCD Mode Pins
		tmp &= ~BCD_MODE_CTRL;
	} else if (!strcmp(bcd_mode, "SDP")) {
		tmp &= ~(3 << 13); // Clear bit [14:13]
		tmp |= (BCD_MODE_CTRL | BCD_MODE_SDP | BCD_EN);
	} else if (!strcmp(bcd_mode, "DCP")) {
		tmp &= ~(3 << 13); // Clear bit [14:13]
		tmp |= (BCD_MODE_CTRL | BCD_MODE_DCP | BCD_EN);
	} else if (!strcmp(bcd_mode, "CDP1")) {
		tmp &= ~(3 << 13); // Clear bit [14:13]
		tmp |= (BCD_MODE_CTRL | BCD_MODE_CDP1 | BCD_EN);
	} else if (!strcmp(bcd_mode, "CDP2")) {
		tmp &= ~(3 << 13); // Clear bit [14:13]
		tmp |= (BCD_MODE_CTRL | BCD_MODE_CDP2 | BCD_EN);
	}

	ft313_reg_write16(ft313, ~VBUS_OFF & tmp, &ft313->cfg->config);
	printk(KERN_NOTICE "FT313: VBUS ON\n");

	/* Set EDGEINTC */
	//ft313_reg_write16(ft313, 0x0008, &ft313->cfg->edge_int_ctrl); // 8 clock cycles
	//tmp = ft313_reg_read16(ft313, &ft313->cfg->edge_int_ctrl);
	//printk(KERN_NOTICE "FT313: EDGEINTC 0x%04x\n", tmp);


	temp32 = ft313_reg_read32(ft313, &ft313->cfg->chip_id);
	printk(KERN_NOTICE "FT313: CHIP ID %08x\n", temp32);

	ft313->caps = hcd->regs + FT313_CAP_OFFSET;
	ft313->regs = hcd->regs + CAPLENGTH(ft313_reg_read32(ft313, &ft313->caps->hc_capbase));

	/* cache this readonly data; minimize chip reads */
	ft313->hcs_params = ft313_reg_read32(ft313, &ft313->caps->hcs_params);

	retval = ft313_halt(ft313);
	if (retval)
		return retval;

	/* data structure init */
	retval = ft313_init(hcd);
	hcd->has_tt = 1;  // host include transaction-translator, will change speed to FS/LS
	ft313->need_io_watchdog = 1;

	if (retval)
		return retval;

	retval = ft313_reset(ft313);

	if (retval)
		return retval;

#ifdef CONFIG_PM
	ft313->wakeup_wq_name = FT313_WK_NAME;
	ft313->wakeup_wq = create_singlethread_workqueue(FT313_WK_NAME);
	if (ft313->wakeup_wq == NULL) {
		ERROR_MSG("FT313 Wakeup Workqueue creation failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&ft313->wakeup_work, ft313_wakeup_wq_handler);
#endif

	return 0;
}

void ft313_han9250_shutdown(struct platform_device *pdev)
{
	struct usb_hcd		*hcd;

	hcd = dev_get_drvdata(&pdev->dev);
	if (!hcd)
		return;

	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) &&
			hcd->driver->shutdown) {
		hcd->driver->shutdown(hcd);
	}
}

static int ft313_update_device(struct usb_hcd *hcd, struct usb_device *udev)
{
//	struct ft313_hcd *ft313 = hcd_to_ft313(hcd);
	int rc = 0;

	if (!udev->parent) /* udev is root hub itself, impossible */
		rc = -1;
	/* we only support lpm device connected to root hub yet */
//	if (ehci->has_lpm && !udev->parent->parent) {
//		rc = ehci_lpm_set_da(ehci, udev->devnum, udev->portnum);
//		if (!rc)
//			rc = ehci_lpm_check(ehci, udev->portnum);
//	}
	return rc;
}

static const struct hc_driver ft313_han9250_hc_driver = {
	.description =		"ft313-hcd",
	.product_desc =		"FT313 SPH Controller",
	.hcd_priv_size =	sizeof(struct ft313_hcd),

	/*
	 * Generic hardware linkage
	 */
	.irq =			ft313_irq,
//	.flags =		HCD_MEMORY | HCD_LOCAL_MEM | HCD_USB2,
	.flags =		HCD_MEMORY | HCD_USB2,

	/*
	 * Basic lifecycle operations
	 */
	.reset =		ft313_platform_setup,
	.start =		ft313_run,
	.stop =			ft313_stop,
	.shutdown =		ft313_shutdown,

	/*
	 * Managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ft313_urb_enqueue,
	.urb_dequeue =		ft313_urb_dequeue,
	.endpoint_disable =	ft313_endpoint_disable,
	.endpoint_reset =	ft313_endpoint_reset,

	/*
	 * Scheduling support
	 */
	.get_frame_number =	ft313_get_frame,

	/*
	 * Root hub support
	 */
	.hub_status_data =	ft313_hub_status_data,
	.hub_control =		ft313_hub_control,
#ifdef CONFIG_PM
	.bus_suspend =		ft313_bus_suspend,
	.bus_resume =		ft313_bus_resume,
#endif

	// OTG
	//.start_port_reset = ???,

	.relinquish_port =      ft313_relinquish_port,
	.port_handed_over =     ft313_port_handed_over,

	/*
	 * call back when device connected and addressed
	 */
	.update_device =	ft313_update_device,

	.clear_tt_buffer_complete	= ft313_clear_tt_buffer_complete,
};


static int ft313_han9250_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct resource				res;
	struct usb_hcd				*hcd;
//	void __iomem				*regs;
//	struct ft313_hcd			*ft313;
	int					ret = -ENODEV;
	int					irq;

	dev_notice(&pdev->dev, "FT313 HAN9250 driver init start\n");

	if (usb_disabled())
		return -ENODEV;
#if 0
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "FT313 irq fetch failed\n");
		return -ENODEV;
	}
#endif

	ret = of_address_to_resource(dn, 0, &res);
	if (ret)
		return ret;

	hcd = usb_create_hcd(&ft313_han9250_hc_driver, &pdev->dev,
				"ft313-ehci");
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = res.start;
	hcd->rsrc_len = resource_size(&res);

	irq = irq_of_parse_and_map(dn, 0);
	if (!irq) {
		dev_err(&pdev->dev, "%s: irq_of_parse_and_map failed\n",
			__FILE__);
		ret = -EBUSY;
		goto err_irq;
	}

	hcd->regs = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto err_irq;
	}
	dev_notice(&pdev->dev, "FT313 hcd->regs = %p\n", hcd->regs);

	hcd->irq = irq;
	hcd->state = HC_STATE_HALT;

	g_regbase = hcd->regs;
	g_print_cnt = 0;

	/* HW Reset active low */
	iowrite32(0, (__u32 __iomem *)g_regbase + 1);
	msleep(1000);
	iowrite32(1, (__u32 __iomem *)g_regbase + 1);
	dev_notice(&pdev->dev, "FT313 RESET REG 0x%08x\n", ioread32((__u32 __iomem *)g_regbase + 1));
	dev_notice(&pdev->dev, "FT313 irq is %d \n", irq);
//	ret = usb_add_hcd(hcd, irq, IRQF_SHARED | IRQF_TRIGGER_FALLING); // Call reset() and start() here!
	ret = usb_add_hcd(hcd, irq, 0); // Call reset() and start() here!
	if (ret != 0) {
		ALERT_MSG("usb add hcd fail!\n");
		goto err_irq;
	}



#if 0
	int count = 10;
	u32 tmp0, tmp1, tmp2, tmp3, tmp;
	while (count--) {
		ALERT_MSG("Read Chip ID\n");
		tmp0 = ioread8(regs + 0x80);
		tmp1 = ioread8(regs + 0x81);
		tmp2 = ioread8(regs + 0x82);
		tmp3 = ioread8(regs + 0x83);

		ALERT_MSG("tmp 0 to 3 are %X, %X, %X, %X repectively\n", tmp0, tmp1, tmp2, tmp3);

		tmp = (tmp3 << 24) | (tmp2 << 16) | (tmp1 << 8) | tmp0;
		ALERT_MSG("Chip ID is 0x%X \n", tmp);
		mdelay(1);
	}
#endif

#if 0
	u8 *pBuf1, *pBuf2;
	int i;
	pBuf1 = kmalloc(16 * 1024, GFP_KERNEL);
	pBuf2 = kmalloc(16 * 1024, GFP_KERNEL);

	ALERT_MSG("\n\n\n\n\nMemory test begin!\n\n\n\n\n");


	if (pBuf1 && pBuf2) {
		// Init data
		//for (i = 0; i < 4096 * 4; i++) pBuf1[i] = (u8)i;
		memset(pBuf1, 0xAA, 16 * 1024);
		memset(pBuf2, 0x55, 16 * 1024);

		// Data write
		iowrite8((16 * 1024) & 0x00FF, regs + 0x94);
		iowrite8(((16 * 1024) & 0xFF00) >> 8, regs + 0x95);

		iowrite8(4096 & 0x00FF, regs + 0x90);
		iowrite8((4096 & 0xFF00) >> 8, regs + 0x91);

		for (i = 0; i < 4096 * 4; i++)
			iowrite8(pBuf1[i], regs + 0x92);
		// Data read
		ALERT_MSG("Data read start\n");
		iowrite8(((16 * 1024) | 0x8000) & 0x00FF, regs + 0x94);
		iowrite8((((16 * 1024) | 0x8000) & 0xFF00) >> 8, regs + 0x95);

		iowrite8(4096 & 0x00FF, regs + 0x90);
		iowrite8((4096 & 0xFF00) >> 8, regs + 0x91);

		for (i = 0; i < 4096 * 4; i++)
			pBuf2[i] = ioread8(regs + 0x92);

		ALERT_MSG("Data read end\n");

		if (memcmp(pBuf1, pBuf2, 16 * 1024) == 0) {
			ALERT_MSG("\n\n\n Memory read/write test pass \n\n\n");
		}
		else {
			ALERT_MSG("\n\n\n Memory read/write test fail \n\n\n");
					// Data compare
			for (i = 0; i < 1024 * 16; i++) {
				if (pBuf1[i] != pBuf2[i]) {
					ALERT_MSG("pBuf1[%d] = 0x%X and pBuf2[%d] = 0x%X\n", i, pBuf1[i], i, pBuf2[i]);
				}
			}

		}
	}
	else {
		ALERT_MSG("Memory allocation error \n");
	}

	ALERT_MSG("\n\n\n\n\nMemory test end!\n\n\n\n\n");

	return -ENOMEM;
#endif

#ifdef DISABLE_HCD_DMA
	// FixMe: Disable DMA
	hcd->self.uses_dma = 0;
	g_old_dma_mask = hcd->self.controller->dma_mask;
	hcd->self.controller->dma_mask = NULL;
	//IRQF_TRIGGER_FALLING
#endif
	dev_notice(&pdev->dev, "FT313: Use DMA is %d\n", hcd->self.uses_dma);
	dev_notice(&pdev->dev, "FT313 driver init complete hcd->state is 0x%X\n", hcd->state);

	return ret;

//unmap_registers:
//	iounmap(regs);

//release_mem_region:
//	release_mem_region(res->start, resource_size(res));

//out_disable:
err_irq:
	usb_put_hcd(hcd);

	ALERT_MSG("ft313 init failed, ft313 device disabled\n");

	return ret;
}

static int 
ft313_han9250_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct device *dev = &pdev->dev;

	hcd = dev_get_drvdata(dev);
	if (!hcd)
		return -EINVAL;

#ifdef DISABLE_HCD_DMA
	hcd->self.controller->dma_mask = g_old_dma_mask;
	hcd->self.uses_dma = 1;
#endif
	usb_remove_hcd(hcd);

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

	usb_put_hcd(hcd);

	return 0;
}

static const struct of_device_id han9250a2_of_match[] = {
		{.compatible = "xlnx,usba-controller-1.00.a",},
	{},
};
MODULE_DEVICE_TABLE(of, ehci_hcd_xsilon_of_match);

/* platform driver */
static struct platform_driver ft313_han9250_evm_driver = {
	.driver		= {
		.name	= "ft313",
		.owner	= THIS_MODULE,
		.of_match_table = han9250a2_of_match,
	},
	.probe		= ft313_han9250_probe,
	.remove		= ft313_han9250_remove,
	.shutdown	= ft313_han9250_shutdown,
};

