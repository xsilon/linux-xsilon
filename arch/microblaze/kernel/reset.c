/*
 * Copyright (C) 2009 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2009 PetaLogix
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/prom.h>

#include <linux/io.h>


static void gpio_system_reset(void)
{
	/* Quick hack to reset system */
	void *reg;
	reg= ioremap_nocache(0x70800028, 4096);
	*((unsigned long *)reg)=0x0; 
}

void of_platform_reset_gpio_probe(void)
{
	/* Don't do anything here. */
	return;
}


void machine_restart(char *cmd)
{
	pr_notice("Machine restart...\n");
	gpio_system_reset();
	while (1)
		;
}

void machine_shutdown(void)
{
	pr_notice("Machine shutdown...\n");
	while (1)
		;
}

void machine_halt(void)
{
	pr_notice("Machine halt...\n");
	while (1)
		;
}

void machine_power_off(void)
{
	pr_notice("Machine power off...\n");
	while (1)
		;
}
