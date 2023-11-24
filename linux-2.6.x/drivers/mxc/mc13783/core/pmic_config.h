/*
 * Copyright 2006 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

#ifndef __MC13783_CONFIG_H__
#define __MC13783_CONFIG_H__

 /*!
  * @file mc13783_config.h
  * @brief This file contains configuration define used by mc13783 protocol.
  *
  * @ingroup MC13783
  */

/*
 * Includes
 */
#include <linux/kernel.h>	/* printk() */
#include <linux/module.h>	/* modules */
#include <linux/init.h>		/* module_{init,exit}() */
#include <linux/devfs_fs_kernel.h>	/* devfs support */
#include <linux/slab.h>		/* kmalloc()/kfree() */
#include <asm/uaccess.h>	/* copy_{from,to}_user() */

#include <asm/arch/pmic_external.h>
#include <asm/arch/pmic_status.h>

#include "component_spec.h"

typedef struct timer_list osi_timer;

#define __ALL_TRACES__

//#define __TEST_CODE_ENABLE__

#define	_A(a)		"mc13783 : "a" \n"
#define _K_A(a)         _A(a)
#define _K_D(a)         KERN_DEBUG _A(a)
#define _K_I(a)         KERN_INFO _A(a)
#define _K_W(a)         KERN_WARNING _A(a)

/*
 * Bitfield macros that use rely on bitfield width/shift information
 * defined in mc13783_***_defs.h header files
 */
#define BITFMASK(field) (((1U << (field ## _WID)) - 1) << (field ## _LSH))
#define BITFVAL(field, val) ((val) << (field ## _LSH))
#define BITFEXT(var, bit) ((var & BITFMASK(bit)) >> (bit ## _LSH))

#define CHECK_ERROR(a)			\
do { 					\
        int ret = (a); 			\
        if(ret != PMIC_SUCCESS) 	\
	return ret; 			\
} while (0)

/* NULL pointer */
#if !defined(NULL)
#define NULL    			((void *) 0)
#endif

#ifdef __ALL_TRACES__
#define TRACEMSG(fmt,args...) 		printk(fmt,##args)
#else				/* __ALL_TRACES__ */
#define TRACEMSG(fmt,args...)
#endif				/* __ALL_TRACES__ */

#define TRACEMSG_ALL_TIME(fmt,args...)  printk(fmt,##args)

#endif				/* __MC13783_CONFIG_H__ */
