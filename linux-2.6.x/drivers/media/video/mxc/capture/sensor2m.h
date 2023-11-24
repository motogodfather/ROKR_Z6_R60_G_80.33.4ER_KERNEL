/*
 *  sensor2m.h
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
Motorola            01/04/2007     OSS code changes

==================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _SENSOR2M_H_
#define _SENSOR2M_H_

#include "../../mxc_camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_sensor2m_init(p_camera_context_t);
int camera_func_sensor2m_deinit(p_camera_context_t);
int camera_func_sensor2m_set_capture_format(p_camera_context_t);
int camera_func_sensor2m_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_sensor2m_stop_capture(p_camera_context_t);
int camera_func_sensor2m_pm_management(p_camera_context_t cam_ctx, int suspend);
int camera_func_sensor2m_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);

#endif /* _SENSOR2M_H_ */


