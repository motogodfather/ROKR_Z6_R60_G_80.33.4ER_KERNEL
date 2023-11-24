/*
 * linux/arch/arm/mach-mx3/pm.c
 *
 * MX3 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1510 by David Singleton:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Cleanup 2004 for OMAP1510/1610 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Modified for the MX31
 * Copyright 2005-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/arch/mxc_pm.h>

#define HAC_GRP  5
#define GEMK_GRP 5
#define GPIO_GRP 4
#define WDOG_GRP 4
#define GPT_GRP  2
#define UART_GRP 0

/*
 * TODO: whatta save?
 */

static int mx31_pm_enter(suspend_state_t state)
{
	printk("Hi, from mx31_pm_enter\n");
	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_pm_lowpower(STOP_MODE);
		break;

	case PM_SUSPEND_STANDBY:
		mxc_pm_lowpower(WAIT_MODE);
		break;

	case PM_SUSPEND_STOP:
		mxc_pm_lowpower(DSM_MODE);
		break;

	default:
		return -1;
	}
	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx31_pm_prepare(suspend_state_t state)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int mx31_pm_finish(suspend_state_t state)
{
	return 0;
}

struct pm_ops mx31_pm_ops = {
	.pm_disk_mode = 0,
	.prepare = mx31_pm_prepare,
	.enter = mx31_pm_enter,
	.finish = mx31_pm_finish,
};

static int __init mx31_pm_init(void)
{
	printk("Power Management for Freescale MX31\n");
	pm_set_ops(&mx31_pm_ops);

	return 0;
}

late_initcall(mx31_pm_init);
