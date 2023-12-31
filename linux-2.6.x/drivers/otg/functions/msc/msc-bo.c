/*
 * otg/function/msc/msc-bo.c
 * @(#) balden@seth2.belcarratech.com|otg/functions/msc/msc-bo.c|20051116204958|23027
 *
 *      Copyright (c) 2003-2004 Belcarra
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
 *
 * Copyright 2006 - 2007 Motorola, Inc.
 *
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 02/18/2006         Motorola         Initial distribution 
 * 04/14/2006         Motorola         Cleaned up file 
 * 10/18/2006         Motorola         Add Open Src Software language
 * 12/11/2006         Motorola         Changes for Open src compliance.
 * 02/23/2007         Motorola         Changes for more data in BULK_OUT.
 * 03/07/2007         Motorola         Added use of CONFIG_OTG_MSC_NUM_PAGES.
 *
 * This Program is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of
 * MERCHANTIBILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at
 * your option) any later version.  You should have
 * received a copy of the GNU General Public License
 * along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave,
 * Cambridge, MA 02139, USA
 *
 */
/*!
 * @file otg/functions/msc/msc-bo.c
 * @brief Mass Storage Driver private defines
 *
 * This is a Mass Storage Class Function that uses the Bulk Only protocol.
 *
 *
 * @ingroup MSCFunction
 */

#include <otg/otg-compat.h>
#include <otg/usbp-chap9.h>
#include <otg/usbp-func.h>
#include <otg/otg-trace.h>

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <linux/random.h>
#include <linux/slab.h>


#include <linux/buffer_head.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <public/otg-node.h>

#include "msc-scsi.h"
#include "msc.h"
#include "msc-fd.h"
#include "crc.h"

//#include "rbc.h"

# define CONFIG_OTG_SERIAL_NUMBER_STR   "69D38D2036248D3A983C5CDA63888"

/*!
 * Mass Storage Class - Bulk Only
 *
 * Endpoint, Class, Interface, Configuration and Device descriptors/descriptions
 */

u8 msc_index[] = { BULK_OUT, BULK_IN, };

extern struct usbd_function_operations msc_function_ops;

static struct usbd_endpoint_request msc_if_endpoint_requests[ENDPOINTS+1] = {
        { BULK_OUT, 1, 0, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, PAGE_SIZE, CONFIG_OTG_MSC_NUM_PAGES * PAGE_SIZE, 0, },
        { BULK_IN, 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_BULK, PAGE_SIZE, PAGE_SIZE, 0, },
        { 0, },
};


#ifndef OTG_C99

//#warning "C99"
static struct usbd_alternate_description msc_data_alternate_descriptions[1];

struct usbd_interface_description msc_interfaces[] = {
        { alternates:sizeof (msc_data_alternate_descriptions) / sizeof (struct usbd_alternate_description),
                alternate_list:msc_data_alternate_descriptions,},
};

struct usbd_interface_driver msc_interface_driver;

void msc_global_init(void){
	
	ZERO(msc_data_alternate_descriptions[0]);

 	msc_data_alternate_descriptions[0].iInterface = CONFIG_OTG_MSC_INTF;
        msc_data_alternate_descriptions[0].bInterfaceClass = MASS_STORAGE_CLASS; 
        msc_data_alternate_descriptions[0].bInterfaceSubClass =  MASS_STORAGE_SUBCLASS_SCSI; 
        msc_data_alternate_descriptions[0].bInterfaceProtocol = MASS_STORAGE_PROTO_BULK_ONLY; 
        msc_data_alternate_descriptions[0].endpoints = 0x02,
        msc_data_alternate_descriptions[0].endpoint_index =  msc_index;

	
	
	ZERO(msc_interface_driver);
	
	msc_interface_driver.driver.name = MSC_DRIVER_NAME;                           
        msc_interface_driver.driver.fops = &msc_function_ops;                             
        msc_interface_driver.interfaces = 0x1;
        msc_interface_driver.interface_list = msc_interfaces;
        msc_interface_driver.endpointsRequested = ENDPOINTS;
        msc_interface_driver.requestedEndpoints = msc_if_endpoint_requests;
	
}



#else /* OTG_C99 */
//#warning "Not C99"

static struct usbd_alternate_description msc_data_alternate_descriptions[] = {
        { 
                .iInterface = "Motorola Mass Storage - Bulk Only",
                .bInterfaceClass = MASS_STORAGE_CLASS, 
                .bInterfaceSubClass =  MASS_STORAGE_SUBCLASS_SCSI, 
                .bInterfaceProtocol = MASS_STORAGE_PROTO_BULK_ONLY, 
                .endpoints = 0x02,
                .endpoint_index =  msc_index,
        },
};



struct usbd_interface_description msc_interfaces[] = {
        { alternates:sizeof (msc_data_alternate_descriptions) / sizeof (struct usbd_alternate_description),
                alternate_list:msc_data_alternate_descriptions,},
};


/*! msc_interface_driver - USB Device Core function driver definition
 */
struct usbd_interface_driver msc_interface_driver;

struct usbd_interface_driver msc_interface_driver = {
        .driver.name = MSC_DRIVER_NAME,                            /*! driver name */
        .driver.fops = &msc_function_ops,                             /*!< operations table */
        .interfaces = 0x1,
        .interface_list = msc_interfaces,
        .endpointsRequested = ENDPOINTS,
        .requestedEndpoints = msc_if_endpoint_requests,
        
        .bFunctionClass = MASS_STORAGE_CLASS, 
        .bFunctionSubClass = MASS_STORAGE_SUBCLASS_SCSI,
        .bFunctionProtocol = MASS_STORAGE_PROTO_BULK_ONLY, 
        .iFunction = "Motorola Mass Storage - Bulk Only",
};






#endif /* OTG_C99 */
