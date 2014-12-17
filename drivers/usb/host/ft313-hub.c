/*
 * FT313 root hub management.
 *
 * Copyright (C) 2011 Chang Yang <chang.yang@ftdichip.com>
 *
 * This code is *strongly* based on EHCI-HCD code by David Brownell since
 * the chip is a quasi-EHCI compatible.
 *
 * Licensed under GPL version 2 only.
 */

/* this file is part of ft313-hcd.c */

//prepare memory data
#define FT313_TEST_PACKET_LENGTH	53
#ifdef FT313_IN_8_BIT_MODE
const u8 ft313_test_packet[FT313_TEST_PACKET_LENGTH] = {
#else
const u8 ft313_test_packet[FT313_TEST_PACKET_LENGTH + 1] = {
#endif
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
	0xFE,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x7F, 0xBF, 0xDF, 0xEF,
	0xF7, 0xFB, 0xFD, 0xFC,
	0x7E,
	0xBF, 0xDF, 0xEF,
	0xF7, 0xFB, 0xFD,
#ifdef FT313_IN_8_BIT_MODE
	0x7E	//active test packet is 53 bytes
#else
	0x7E,	//active test packet is 53 bytes
	0x00	// this is only for even array
#endif	
};

#ifdef CONFIG_PM

static int ft313_bus_suspend(struct usb_hcd *hcd)
{
	struct ft313_hcd	*ft313 = hcd_to_ft313 (hcd);
	int			port;
	int			changed;

	u16			hc_int_en = 0, hc_int_sts = 0, config = 0;
	u32 			temp = 0;

	FUN_ENTRY();

	DEBUG_MSG("suspend root hub\n");

	// Clear HCINTSTS register
	hc_int_sts = ft313_reg_read16(ft313, &ft313->cfg->hc_int_sts);
	if (0 != hc_int_sts)
		ft313_reg_write16(ft313, hc_int_sts, &ft313->cfg->hc_int_sts);

	if (time_before (jiffies, ft313->next_statechange))
		msleep(5);
	del_timer_sync(&ft313->watchdog);
	del_timer_sync(&ft313->iaa_watchdog);

	spin_lock_irq (&ft313->lock);

	/* Once the controller is stopped, port resumes that are already
	 * in progress won't complete.  Hence if remote wakeup is enabled
	 * for the root hub and any ports are in the middle of a resume or
	 * remote wakeup, we must fail the suspend.
	 */
	if (hcd->self.root_hub->do_remote_wakeup) {
		port = HCS_N_PORTS(ft313->hcs_params);
		while (port--) {
			if (ft313->reset_done[port] != 0) {
				spin_unlock_irq(&ft313->lock);
				DEBUG_MSG("suspend failed because "
					 "port %d is resuming\n",
					 port + 1);
				FUN_EXIT();
				return -EBUSY;
			}
		}
	}

	/* stop schedules, clean any completed work */
	if (HC_IS_RUNNING(hcd->state)) {
		ft313_quiesce (ft313);
		hcd->state = HC_STATE_QUIESCING;
	}
	ft313->command = ft313_reg_read32(ft313, &ft313->regs->command);
	ft313_work(ft313);

	// Stop ft313 RS bit before suspend port!!!
	ft313_halt (ft313);

	/* Unlike other USB host controller types, EHCI doesn't have
	 * any notion of "global" or bus-wide suspend.  The driver has
	 * to manually suspend all the active unsuspended ports, and
	 * then manually resume them in the bus_resume() routine.
	 */
	ft313->bus_suspended = 0;
	ft313->owned_ports = 0;
	changed = 0;
	port = HCS_N_PORTS(ft313->hcs_params);
	while (port--) {
		u32 __iomem	*reg = &ft313->regs->port_status [port];
		u32		t1 = ft313_reg_read32(ft313, reg) & ~PORT_RWC_BITS;
		u32		t2 = t1; // Wake-up enable bits not availabe in FT313!  & ~PORT_WAKE_BITS;

		/* keep track of which ports we suspend */
		if ((t1 & PO_EN) && !(t1 & PO_SUSP)) {
			t2 |= PO_SUSP;
			DEBUG_MSG("Port %d need suspend\n", port);
			set_bit(port, &ft313->bus_suspended);
		}

		/* enable remote wakeup on all ports, if told to do so */
		if (hcd->self.root_hub->do_remote_wakeup) {
			/* only enable appropriate wake bits, otherwise the
			 * hardware can not go phy low power mode. If a race
			 * condition happens here(connection change during bits
			 * set), the port change detection will finally fix it.
			 */
/*
			if (t1 & CONN_STS)
				t2 |= PORT_WKOC_E | PORT_WKDISC_E;
			else
				t2 |= PORT_WKOC_E | PORT_WKCONN_E;
*/
			hc_int_en = ft313_reg_read16(ft313, &ft313->cfg->hc_int_en);
			hc_int_en |= WAKEUPINT_EN;
			DEBUG_MSG("Enable the device connection/disconnection wakeup interrupt\n");
		}

		if (t1 != t2) {
			DEBUG_MSG("port %d, %08x -> %08x\n", port + 1, t1, t2);
			ft313_reg_write32(ft313, t2, reg);
			mdelay(5);
			changed = 1;
		}
	}

	/* Apparently some devices need a >= 1-uframe delay here */
	if (ft313->bus_suspended)
		udelay(150);

	/* turn off now-idle HC */
//	ft313_halt (ft313); // Move up as FT313 require RS bit to be zero b4 suspend port
	hcd->state = HC_STATE_SUSPENDED;

	if (ft313->reclaim)
		end_unlink_async(ft313);

	// Disable EHCI interrupt as FT313 has its own way to process
	ft313_reg_write32(ft313, 0, &ft313->regs->intr_enable);

	// Suspend FT313
	config &= ~(OSC_EN | PLL_EN | HC_CLK_EN); // Clear Osc, PLL and HC Clock
	config |= REG_PWR; // Regulator power still on
	ft313_reg_write16(ft313, config, &ft313->cfg->config);

	// Enable FT313 resume interrupt, wakeup on con/discon and remote wakeup as well as OC
	hc_int_en |= (CLKREADY_EN | WAKEUPINT_EN | REMOTEWKINT_EN | OCINT_EN);
	ft313_reg_write16(ft313, hc_int_en, &ft313->cfg->hc_int_en);

	// Set transceiver to suspend mode
	temp = ft313_reg_read32(ft313, &ft313->regs->eof_time);
	temp &= ~U_SUSP_N;
	ft313_reg_write32(ft313, temp, &ft313->regs->eof_time);

	ft313->next_statechange = jiffies + msecs_to_jiffies(10);
	spin_unlock_irq (&ft313->lock);

	/* ehci_work() may have re-enabled the watchdog timer, which we do not
	 * want, and so we must delete any pending watchdog timer events.
	 */
	del_timer_sync(&ft313->watchdog);
	FUN_EXIT();
	return 0;
}

static int ft313_bus_resume(struct usb_hcd *hcd)
{
	struct ft313_hcd	*ft313 = hcd_to_ft313 (hcd);
	u32			temp;
	u32			power_okay;
	int			i;
	u8			resume_needed = 0;

	int			count = 0;

	FUN_ENTRY();

	// Dummy read to wakeup chip
	DEBUG_MSG("Dummy read to wakeup FT313\n");
#ifdef FT313_IN_8_BIT_MODE
	temp = ft313_reg_read8(ft313, &ft313->cfg->sw_reset);
#else
	temp = ft313_reg_read16(ft313, &ft313->cfg->sw_reset);
#endif
	msleep(1);

	temp = ft313_reg_read32(ft313, &ft313->regs->eof_time);
	while (!(temp & U_SUSP_N) && count++ < 100) {
		temp |= U_SUSP_N;
		ft313_reg_write32(ft313, temp, &ft313->regs->eof_time);
		mdelay(1);
		temp = ft313_reg_read32(ft313, &ft313->regs->eof_time);
	};

	if (time_before (jiffies, ft313->next_statechange))
		msleep(5);
	spin_lock_irq (&ft313->lock);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		ALERT_MSG("HW is not accessible\n");
		spin_unlock_irq(&ft313->lock);
		FUN_EXIT();
		return -ESHUTDOWN;
	}

	/* Ideally and we've got a real resume here, and no port's power
	 * was lost.  (For PCI, that means Vaux was maintained.)  But we
	 * could instead be restoring a swsusp snapshot -- so that BIOS was
	 * the last user of the controller, not reset/pm hardware keeping
	 * state we gave to it.
	 */
	power_okay = ft313_reg_read32(ft313, &ft313->regs->intr_enable);
	DEBUG_MSG("resume root hub%s\n",
		 power_okay ? "" : " after power loss");

	/* at least some APM implementations will try to deliver
	 * IRQs right away, so delay them until we're ready.
	 */
	ft313_reg_write32(ft313, 0, &ft313->regs->intr_enable);

	/* re-init operational registers */
	ft313_reg_write32(ft313, ft313->periodic_ft313, &ft313->regs->frame_list);
	ft313_reg_write32(ft313, (u32) ft313->async->qh_ft313, &ft313->regs->async_next);

	/* restore CMD_RUN, framelist size, and irq threshold */
	ft313_reg_write32(ft313, ft313->command, &ft313->regs->command);

	/* Some controller/firmware combinations need a delay during which
	 * they set up the port statuses.  See Bugzilla #8190. */
	spin_unlock_irq(&ft313->lock);
	msleep(8);
	spin_lock_irq(&ft313->lock);

	/* manually resume the ports we suspended during bus_suspend() */
	i = HCS_N_PORTS (ft313->hcs_params);
	while (i--) {
		temp = ft313_reg_read32(ft313, &ft313->regs->port_status [i]);
		temp &= ~(PORT_RWC_BITS); // No wakeup bits in FT313H's portsc register
		if (test_bit(i, &ft313->bus_suspended)	&&
				(temp & CONN_STS)	&& //At least there should be something on port!
				(temp & PO_SUSP)) {
			temp |= F_PO_RESM;
			resume_needed = 1;
		}
		ft313_reg_write32(ft313, temp, &ft313->regs->port_status [i]);
	}

	/* msleep for 20ms only if code is trying to resume port */
	if (resume_needed) {
		spin_unlock_irq(&ft313->lock);
		msleep(20);
		spin_lock_irq(&ft313->lock);
	}

	i = HCS_N_PORTS (ft313->hcs_params);
	while (i--) {
		temp = ft313_reg_read32(ft313, &ft313->regs->port_status [i]);
		if (test_bit(i, &ft313->bus_suspended)	&&
				(temp & CONN_STS)	&& //At least there should be something on port!
				(temp & PO_SUSP)) {
			temp &= ~(PORT_RWC_BITS | F_PO_RESM);
			ft313_reg_write32(ft313, temp, &ft313->regs->port_status [i]);
			DEBUG_MSG("resumed port %d\n", i + 1);
		}
	}
	(void) ft313_reg_read32(ft313, &ft313->regs->command);

	/* maybe re-activate the schedule(s) */
	temp = 0;
	if (ft313->async->qh_next.qh)
		temp |= ASCH_EN;
	if (ft313->periodic_sched)
		temp |= PSCH_EN;
	if (temp) {
		ft313->command |= temp;
		ft313_reg_write32(ft313, ft313->command, &ft313->regs->command);
	}

	ft313->next_statechange = jiffies + msecs_to_jiffies(5);
	hcd->state = HC_STATE_RUNNING;

	/* Now we can safely re-enable irqs */
	ft313_reg_write32(ft313, INTR_MASK, &ft313->regs->intr_enable);
	ft313_reg_write16(ft313, FT313_INTR_MASK, &ft313->cfg->hc_int_en);

	spin_unlock_irq (&ft313->lock);
//	ehci_handover_companion_ports(ft313);
	FUN_EXIT();
	return 0;
}


void ft313_wakeup_wq_handler(struct work_struct *work)
{
	struct ft313_hcd	*ft313;
	struct usb_hcd		*hcd;
//	int			retval;

	FUN_ENTRY();

	ft313 = container_of(work, struct ft313_hcd, wakeup_work);
	hcd = ft313_to_hcd(ft313);

	ft313_bus_resume(hcd);
//	retval = ft313_bus_resume(hcd);

	FUN_EXIT();

//	return retval;
}

#endif

/*-------------------------------------------------------------------------*/

static int check_reset_complete (
	struct ft313_hcd	*ft313,
	int		index,
	u32 __iomem	*status_reg,
	int		port_status
) {
	if (!(port_status & CONN_STS))
		return port_status;

	/* if reset finished and it's still not enabled -- handoff */
	if (!(port_status & PO_EN)) {

		/* with integrated TT, there's nobody to hand it to! */
		if (ehci_is_TDI(ft313)) {
			ft313_dbg(ft313,
				"Failed to enable port %d on root hub TT\n",
				index+1);
			return port_status;
		}
	} else
		ft313_dbg (ft313, "port %d high speed\n", index + 1);

	return port_status;
}
/*-------------------------------------------------------------------------*/
/* build "status change" packet (one or two bytes) from HC registers */

static int
ft313_hub_status_data (struct usb_hcd *hcd, char *buf)
{
	struct ft313_hcd *ft313 = hcd_to_ft313 (hcd);
	u32		temp, status = 0;
	u32		mask;
	int		ports, i, retval = 1;
	unsigned long	flags;

	FUN_ENTRY();

	/* if !USB_SUSPEND, root hub timers won't get shut down ... */
	if (!HC_IS_RUNNING(hcd->state)) {
		DEBUG_MSG("HC is stopped, hcd->state is 0x%X\n", hcd->state);
		FUN_EXIT();
		return 0;
	}

	/* init status to no-changes */
	buf [0] = 0;
	ports = HCS_N_PORTS (ft313->hcs_params);
	if (ports > 7) {
		buf [1] = 0;
		retval++;
	}

	mask = CONN_CHG | PO_EN_CHG;/* | PORT_OCC; OCC is not supported in Faraday IP */

	/* no hub change reports (bit 0) for now (power, ...) */

	/* port N changes (bit N)? */
	spin_lock_irqsave (&ft313->lock, flags);

	for (i = 0; i < ports; i++) {
		/* leverage per-port change bits feature */
		temp = ft313_reg_read32(ft313, &ft313->regs->port_status [i]);

		/*
		 * Return status information even for ports with OWNER set.
		 * Otherwise khubd wouldn't see the disconnect event when a
		 * high-speed device is switched over to the companion
		 * controller by the user.
		 */

		if ((temp & mask) != 0 || test_bit(i, &ft313->port_c_suspend)
				|| (ft313->reset_done[i] && time_after_eq(
					jiffies, ft313->reset_done[i]))) {
			if (i < 7)
			    buf [0] |= 1 << (i + 1);
			else
			    buf [1] |= 1 << (i - 7);
			status = PO_CHG_DET;
		}
	}
	/* FIXME autosuspend idle root hubs */
	spin_unlock_irqrestore (&ft313->lock, flags);
	FUN_EXIT();
	return status ? retval : 0;
}

/*-------------------------------------------------------------------------*/

static void
ft313_hub_descriptor (
	struct ft313_hcd			*ft313,
	struct usb_hub_descriptor	*desc
) {
	int		ports = HCS_N_PORTS (ft313->hcs_params);
	u16		temp;

	desc->bDescriptorType = 0x29;
	desc->bPwrOn2PwrGood = 10;	/* ehci 1.0, 2.3.9 says 20ms max */
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = ports;
	temp = 1 + (ports / 8);
	desc->bDescLength = 7 + 2 * temp;

	/* two bitmaps:  ports removable, and usb 1.0 legacy PortPwrCtrlMask */
	memset(&desc->u.hs.DeviceRemovable[0], 0, temp);
	memset(&desc->u.hs.DeviceRemovable[temp], 0xff, temp);

	temp = 0x0008;			/* per-port overcurrent reporting */
	temp |= 0x0002;			/* no power switching */

	desc->wHubCharacteristics = cpu_to_le16(temp);
}

/*-------------------------------------------------------------------------*/
static int ft313_hub_control (
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ft313_hcd	*ft313 = hcd_to_ft313 (hcd);
	int		ports = HCS_N_PORTS (ft313->hcs_params);
	u32 __iomem	*status_reg = &ft313->regs->port_status[
				(wIndex & 0xff) - 1];
	u32		temp, status;
	u32		tmp;
	unsigned long	flags;
	int		retval = 0;
	unsigned	selector;

	FUN_ENTRY();

	DEBUG_MSG("typeReq: 0x%X, wValue: 0x%X, wIndex: 0x%X, wLength: 0x%X\n",
			typeReq, wValue, wIndex, wLength);

	/*
	 * FIXME:  support SetPortFeatures USB_PORT_FEAT_INDICATOR.
	 * HCS_INDICATOR may say we can change LEDs to off/amber/green.
	 * (track current state ourselves) ... blink for diagnostics,
	 * power, "this is the one", etc.  EHCI spec supports this.
	 */
	spin_lock_irqsave (&ft313->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		DEBUG_MSG("ClearHubFeature\n");
		switch (wValue) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		DEBUG_MSG("ClearPortFeature\n");
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		temp = ft313_reg_read32(ft313, status_reg);

		/*
		 * Even if OWNER is set, so the port is owned by the
		 * companion controller, khubd needs to be able to clear
		 * the port-change status bits (especially
		 * USB_PORT_STAT_C_CONNECTION).
		 */

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			DEBUG_MSG("USB_PORT_FEAT_ENABLE\n");
			ft313_reg_write32(ft313, temp & ~PO_EN, status_reg);
                        if (temp & PO_RESET) {
                           u32 retval;

			   /* force reset to complete */
			   ft313_reg_write32(ft313, temp & ~(PORT_RWC_BITS | PO_RESET),
			  		status_reg);
			   /* REVISIT:  some hardware needs 550+ usec to clear
			    * this bit; seems too long to spin routinely...
			    */
			   retval = handshake(ft313, status_reg,
			  		   PO_RESET, 0, 750);
			   if (retval != 0)
				printk("port %d reset error %d\n",
					   wIndex + 1, retval);
                           else
                                printk("PORT_RESET with C_PORT_ENABLE done\n");
                        }
			break;
		case USB_PORT_FEAT_C_ENABLE:
			DEBUG_MSG("USB_PORT_FEAT_C_ENABLE\n");
			ft313_reg_write32(ft313, (temp & ~PORT_RWC_BITS) | PO_EN_CHG,
					status_reg);
			break;
		case USB_PORT_FEAT_SUSPEND:
			DEBUG_MSG("USB_PORT_FEAT_SUSPEND\n");
			if (temp & PO_RESET)
				goto error;

			if (!(temp & PO_SUSP))
				break;
			if ((temp & PO_EN) == 0)
				goto error;

			/* resume signaling for 20 msec */
			temp &= ~(PORT_RWC_BITS); // FT313 do not have port_wake_bits | PORT_WAKE_BITS);
			ft313_reg_write32(ft313, temp | F_PO_RESM, status_reg);
			ft313->reset_done[wIndex] = jiffies
					+ msecs_to_jiffies(20);

			break;
		case USB_PORT_FEAT_C_SUSPEND:
			DEBUG_MSG("USB_PORT_FEAT_C_SUSPEND\n");
			clear_bit(wIndex, &ft313->port_c_suspend);
			break;
		case USB_PORT_FEAT_POWER:
			DEBUG_MSG("USB_PORT_FEAT_POWER\n");
			tmp = ft313_reg_read16(ft313, &ft313->cfg->config);
			if (!(tmp & VBUS_OFF))
				ft313_reg_write16(ft313, VBUS_OFF | tmp, &ft313->cfg->config);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			DEBUG_MSG("USB_PORT_FEAT_C_CONNECTION\n");
			ft313_reg_write32(ft313, (temp & ~PORT_RWC_BITS) | CONN_CHG,
					status_reg);
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			DEBUG_MSG("USB_PORT_FEAT_C_OVER_CURRENT\n");
		/*
			// FIXME: PORT_OCC not available
			ft313_reg_write32(ft313, (temp & ~PORT_RWC_BITS) | PORT_OCC,
					status_reg);
		*/
			break;
		case USB_PORT_FEAT_C_RESET:
			DEBUG_MSG("USB_PORT_FEAT_C_RESET\n");
			/* GetPortStatus clears reset */
			break;
		default:
			DEBUG_MSG("\n");
			goto error;
		}
		ft313_reg_read32(ft313, &ft313->regs->command);	/* unblock posted write */
		break;
	case GetHubDescriptor:
		DEBUG_MSG("GetHubDescriptor\n");
		ft313_hub_descriptor (ft313, (struct usb_hub_descriptor *)
			buf);
		break;
	case GetHubStatus:
		DEBUG_MSG("GetHubStatus\n");
		/* no hub-wide feature/status flags */
		memset (buf, 0, 4);
		//cpu_to_le32s ((u32 *) buf);
		break;
	case GetPortStatus:
		DEBUG_MSG("GetPortStatus\n");
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		status = 0;

		temp = ft313_reg_read32(ft313, status_reg);

		// wPortChange bits
		if (temp & CONN_CHG)
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		if (temp & PO_EN_CHG)
			status |= USB_PORT_STAT_C_ENABLE << 16;

		/* whoever resumes must GetPortStatus to complete it!! */
		if (temp & F_PO_RESM) {

			/* Remote Wakeup received? */
			if (!ft313->reset_done[wIndex]) {
				/* resume signaling for 20 msec */
				ft313->reset_done[wIndex] = jiffies
						+ msecs_to_jiffies(20);
				/* check the port again */
				mod_timer(&ft313_to_hcd(ft313)->rh_timer,
						ft313->reset_done[wIndex]);
			}

			/* resume completed? */
			else if (time_after_eq(jiffies,
					ft313->reset_done[wIndex])) {
				clear_bit(wIndex, &ft313->suspended_ports);
				set_bit(wIndex, &ft313->port_c_suspend);
				ft313->reset_done[wIndex] = 0;

				/* stop resume signaling */
				temp = ft313_reg_read32(ft313, status_reg);
				ft313_reg_write32(ft313,
					temp & ~(PORT_RWC_BITS | F_PO_RESM),
					status_reg);
				retval = handshake(ft313, status_reg,
					   F_PO_RESM, 0, 2000 /* 2msec */);
				if (retval != 0) {
					ERROR_MSG("port %d resume error %d\n",
						wIndex + 1, retval);
					goto error;
				}
				temp &= ~(PO_SUSP | F_PO_RESM | (3<<10));
			}
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if ((temp & PO_RESET)
				&& time_after_eq(jiffies,
					ft313->reset_done[wIndex])) {
			status |= USB_PORT_STAT_C_RESET << 16;
			ft313->reset_done [wIndex] = 0;

			/* force reset to complete */
			ft313_reg_write32(ft313, temp & ~(PORT_RWC_BITS | PO_RESET),
					status_reg);
			udelay(200);//Add delay here after port reset
			/* REVISIT:  some hardware needs 550+ usec to clear
			 * this bit; seems too long to spin routinely...
			 */
			retval = handshake(ft313, status_reg,
					PO_RESET, 0, 1000);
			if (retval != 0) {
				ft313_err (ft313, "port %d reset error %d\n",
					wIndex + 1, retval);
				goto error;
			}

			/* see what we found out */
			temp = check_reset_complete (ft313, wIndex, status_reg,
					ft313_reg_read32(ft313, status_reg));

			DEBUG_MSG("IP patch start\n");
			ft313_reg_write32(ft313,
					  ft313_reg_read32(ft313, &ft313->regs->command) | RS,
					  &ft313->regs->command);
			while(ft313_reg_read32(ft313, &ft313->regs->status) & HCHALTED);
			DEBUG_MSG("IP patch end\n");
		}

		if (!(temp & (F_PO_RESM|PO_RESET)))
			ft313->reset_done[wIndex] = 0;

		if ((temp & CONN_STS) && (temp & PO_EN_CHG)) {
			temp &= ~PORT_RWC_BITS;
			temp |= PO_EN_CHG; //Clear Port Enable/Disable Change bit (bit 3)
			ft313_reg_write32(ft313, temp, status_reg);
			ft313_dbg(ft313, "port %d --> companion\n", wIndex + 1);
			temp = ft313_reg_read32(ft313, status_reg);
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */

		if (temp & CONN_STS) {
			status |= USB_PORT_STAT_CONNECTION;
			// status may be from integrated TT
			status |= ft313_port_speed(ft313, temp);
		}
		if (temp & PO_EN)
			status |= USB_PORT_STAT_ENABLE;

		/* maybe the port was unsuspended without our knowledge */
		if (temp & (PO_SUSP|F_PO_RESM)) {
			status |= USB_PORT_STAT_SUSPEND;
		} else if (test_bit(wIndex, &ft313->suspended_ports)) {
			clear_bit(wIndex, &ft313->suspended_ports);
			ft313->reset_done[wIndex] = 0;
			if (temp & PO_EN)
				set_bit(wIndex, &ft313->port_c_suspend);
		}

		if (temp & PO_RESET)
			status |= USB_PORT_STAT_RESET;

		// Port power status
		tmp = ft313_reg_read16(ft313, &ft313->cfg->config);
		if (0 == (tmp & VBUS_OFF))
			status |= USB_PORT_STAT_POWER;

		if (test_bit(wIndex, &ft313->port_c_suspend))
			status |= USB_PORT_STAT_C_SUSPEND << 16;

#ifndef	VERBOSE_DEBUG
	if (status & ~0xffff)	/* only if wPortChange is interesting */
#endif
		dbg_port (ft313, "GetStatus", wIndex + 1, temp);

		put_unaligned_le32(status, buf);

		DEBUG_MSG("returned status value is 0x%X\n", status);

		break;
	case SetHubFeature:
		DEBUG_MSG("SetHubFeature\n");
		switch (wValue) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			goto error;
		}
		break;
	case SetPortFeature:
		DEBUG_MSG("SetPortFeature\n");
		selector = wIndex >> 8;
		wIndex &= 0xff;
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		temp = ft313_reg_read32(ft313, status_reg);

		temp &= ~PORT_RWC_BITS;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if ((temp & PO_EN) == 0
					|| (temp & PO_RESET) != 0)
				goto error;

			/* After above check the port must be connected.
			 * Set appropriate bit thus could put phy into low power
			 * mode if we have hostpc feature
			 */
			ft313_reg_write32(ft313, temp | PO_SUSP, status_reg);
			set_bit(wIndex, &ft313->suspended_ports);
			break;
		case USB_PORT_FEAT_POWER:
			DEBUG_MSG("wValue = USB_PORT_FEAT_POWER\n");
			tmp = ft313_reg_read16(ft313, &ft313->cfg->config);
			if (tmp & VBUS_OFF)
				ft313_reg_write16(ft313, ~VBUS_OFF & tmp, &ft313->cfg->config);
			break;
		case USB_PORT_FEAT_RESET:
			DEBUG_MSG("wValue = USB_PORT_FEAT_RESET\n");
			if (temp & F_PO_RESM)
				goto error;
// IP patch start
			{
				u32 tmp,count;
				DEBUG_MSG("IP patch start\n");
				/* Stop controller for reset */
				tmp = ft313_reg_read32(ft313, &ft313->regs->command);
				//printk("Stop for reset %x\n",temp);
				ft313_reg_write32(ft313, tmp & (~RS),&ft313->regs->command);
				count=0;
				tmp = 0;
				while (((tmp & HCHALTED) == 0) && (count<=10000)) {
					udelay(125);
					count++;
					if (count == 0)
						tmp = ft313_reg_read32(ft313, &ft313->regs->status);
					else
						tmp = _ft313_reg_read32(ft313, &ft313->regs->status);
				}
				DEBUG_MSG("Reset %d times\n",count);

				if (count>=10000) {
					u32 regcommand, regenable;
					u32 i,tmp;

					DEBUG_MSG("Host cannot enter HALT state, recover.....\n");
					regcommand = ft313_reg_read32(ft313, &ft313->regs->command);
					regenable = ft313_reg_read32(ft313, &ft313->regs->intr_enable);

					ft313_reg_write32(ft313, HC_RESET, &ft313->regs->command);
					i=0;
					tmp = HC_RESET;
					while (((tmp & HC_RESET)) && (i<=100)) {
						mdelay(60);
						i++;
						tmp = ft313_reg_read32(ft313, &ft313->regs->command);
					}
					DEBUG_MSG("FT313 Host IP Reset OK.....%d\n",i);
					ft313_reg_write32(ft313, regenable,&ft313->regs->intr_enable);
					ft313_reg_write32(ft313, regcommand,&ft313->regs->command);
					mdelay(1);
					ft313_reg_write32(ft313, regcommand | RS,&ft313->regs->command);

					DEBUG_MSG("Host recover OK\n");
				}
				DEBUG_MSG("IP patch end\n");
			}
// Faraday patch end
			temp |= PO_RESET;
			temp &= ~PO_EN;

			/*
			 * caller must wait, then call GetPortStatus
			 * usb 2.0 spec says 50 ms resets on root
			 */
			ft313->reset_done [wIndex] = jiffies
					+ msecs_to_jiffies (50);

			ft313_reg_write32(ft313, temp, status_reg);
			break;

		/* For downstream facing ports (these):  one hub port is put
		 * into test mode according to USB2 11.24.2.13, then the hub
		 * must be reset (which for root hub now means rmmod+modprobe,
		 * or else system reboot).  See EHCI 2.3.9 and 4.14 for info
		 * about the EHCI-specific stuff.
		 */
		case USB_PORT_FEAT_TEST:
			DEBUG_MSG("wValue = USB_PORT_FEAT_TEST\n");
			DEBUG_MSG("selector is %d\n", selector);
			if (!selector || selector > 5)
				goto error;
			ft313_quiesce(ft313);
			ft313_halt(ft313);

			switch (selector) {
				case 1: // USB_PID_TEST_SE0_NAK
					ALERT_MSG("USB_PID_TEST_SE0_NAK test \n");
					temp |= 1 << 16; // For FT313, just set bit 16 as one
					ft313_reg_write32(ft313, temp, status_reg);
					break;

				case 2: // USB_PID_TEST_J
					ALERT_MSG("USB_PID_TEST_J test \n");
					temp = ft313_reg_read32(ft313, &ft313->regs->test_mode);
					temp |= TST_JSTA;
					ft313_reg_write32(ft313, temp, &ft313->regs->test_mode);
					break;

				case 3: // USB_PID_TEST_K
					ALERT_MSG("USB_PID_TEST_K test \n");
					temp = ft313_reg_read32(ft313, &ft313->regs->test_mode);
					temp |= TST_KSTA;
					ft313_reg_write32(ft313, temp, &ft313->regs->test_mode);
					break;

				case 4: {// USB_PID_TEST_PACKET
					struct ft313_mem_blk *mem_blk;
					int i = 0;

					ALERT_MSG("USB_PID_TEST_PACKET test \n");

					mem_blk = allocate_mem_blk(ft313, BUFFER, sizeof(ft313_test_packet));
					if (NULL == mem_blk) {
						ALERT_MSG("Cannot allocate enough memory\n");
						goto error;
					}

					// Set TEST_PACKET + TST_LOOPBK
					ft313_reg_write32(ft313, TST_LOOPBK | TST_PKT, &ft313->regs->test_mode);

					// Copy test packet payload
					printk(KERN_ALERT "Write test packet from 0x%X with content:\n", mem_blk->offset);
					for (i = 0; i < sizeof(ft313_test_packet); i++) {
						printk(KERN_ALERT "Test_packet[%d] = 0x%X\n", i, ft313_test_packet[i]);
					}
					ft313_mem_write(ft313, (void*)ft313_test_packet, sizeof(ft313_test_packet), mem_blk->offset);

					// Set DMA memory address
					ft313_reg_write32(ft313, mem_blk->offset, &ft313->regs->testpmset2);
					// Program DMA Length and direction
					temp = (FT313_TEST_PACKET_LENGTH << 8) | DMA_TYPE;
					ft313_reg_write32(ft313, temp, &ft313->regs->testpmset1);

					// Trigger DMA
					temp = ft313_reg_read32(ft313, &ft313->regs->testpmset1);
					temp |= DMA_START;
					ft313_reg_write32(ft313, temp, &ft313->regs->testpmset1);
					udelay(100);

					free_mem_blk(ft313, mem_blk->offset);

					break;
				}

				case 5: // USB_PID_TEST_FORCE_ENABLE
					break;
			}

			break;

		default:
			goto error;
		}
		ft313_reg_read32(ft313, &ft313->regs->command);	/* unblock posted writes */
		break;

	default:
error:
		/* "stall" on error */
		retval = -EPIPE;
	}

	spin_unlock_irqrestore (&ft313->lock, flags);
	DEBUG_MSG("Return value is %d\n", retval);
	FUN_EXIT();
	return retval;
}

static void ft313_relinquish_port(struct usb_hcd *hcd, int portnum)
{
	struct ft313_hcd		*ft313 = hcd_to_ft313(hcd);

	if (ehci_is_TDI(ft313))
		return;
}

static int ft313_port_handed_over(struct usb_hcd *hcd, int portnum)
{
		return 0;
}
