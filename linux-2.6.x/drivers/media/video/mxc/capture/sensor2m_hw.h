/*
 *  sensor2m_hw.h
 *
 *  Camera Module driver.
 *
 *  Copyright (C) 2006-2007 Motorola Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *

Revision History:
                   Modification     
Author                 Date        Description of Changes
----------------   ------------    -------------------------
Motorola            03/14/2006     Creation
Motorola            03/15/2006     Support fullsize capture
Motorola            12/07/2006     OSS code changes
Motorola            01/04/2007     OSS code changes

==================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _SENSOR2M_HW_H__
#define _SENSOR2M_HW_H__

#include "../../mxc_camera.h"

/***********************************************************************
 * 
 * Constants & Structures
 *
 ***********************************************************************/

#define MAX_WIDTH       1600
#define MAX_HEIGHT      1200
#define MAX_VF_WIDTH    (MAX_WIDTH/2) 
#define MAX_VF_HEIGHT   (MAX_HEIGHT/2)

#define MIN_WIDTH       64
#define MIN_HEIGHT      48

#define DEFT_WIDTH      1600
#define DEFT_HEIGHT     1200
#define DEFT_VF_WIDTH   320
#define DEFT_VF_HEIGHT  240


// Return codes
#define SENSOR_ERR_NONE         0
#define SENSOR_ERR_TIMEOUT      -1
#define SENSOR_ERR_PARAMETER    -2  

                                                                            
/***********************************************************************                   
 *                                                                                         
 * Function Prototype                 
 *                                    
 ***********************************************************************/

u16  sensor2m_reg_read(u16 reg_addr);
void sensor2m_reg_write(u16 reg_addr, u16 reg_value);


// Configuration Procedures
int sensor2m_get_device_id(u16 *id, u16 *rev);

int sensor2m_viewfinder_on( void );
int sensor2m_snapshot_trigger( void );

int sensor2m_set_time(u16 time);
int sensor2m_set_exposure_mode(int mode, u32 metime);

int sensor2m_sensor_size(window_size_t * win);
int sensor2m_output_size(window_size_t * win);
int sensor2m_capture_sensor_size(window_size_t * win);
int sensor2m_capture_size(window_size_t * win);

int sensor2m_set_style(PIC_STYLE_T style);
int sensor2m_set_light(PIC_WB_T light);
int sensor2m_set_bright(int bright);
int sensor2m_set_flicker(int bright);
int sensor2m_set_mirror(int rows, int columns);

int sensor2m_default_settings( void );
int sensor2m_enter_9(void);
int sensor2m_exit_9(void);

int sensor2m_set_zoom(int azoom, int bzoom);

#endif /* _SENSOR2M_HW_H__ */

