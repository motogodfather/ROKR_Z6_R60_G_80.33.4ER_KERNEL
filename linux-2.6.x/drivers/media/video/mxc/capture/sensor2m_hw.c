/*
 *  sensor2m_hw.c
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
Author                          Date        Description of Changes
-------------------------   ------------    -------------------------------------------
Motorola                     03/14/2006     File creation
Motorola                     06/09/2006     Support Sensor 2M
Motorola                     08/25/2006     Fix patch-loading for rev2
Motorola                     09/05/2006     the mdelay() in camera driver will
                                            decrease system's performance
Motorola                     10/11/2006     Apply patch version 16
Motorola                     11/07/2006     Increase capture rate to 13fps
Motorola                     12/07/2006     OSS code changes
Motorola                     12/14/2006     Improve capture time
Motorola                     01/04/2007     OSS code changes
Motorola                     01/17/2007     Improve zoom quality
Motorola                     02/16/2007     Tuning of imager quality settings
Motorola                     02/20/2007     Desense improvements

==================================================================================================
                                        INCLUDE FILES
==================================================================================================*/
#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h> 

#include "sensor2m_hw.h"

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned long UINT32;

typedef int SENSOR_FN_RETURN_TYPE_T;
#define SENSOR_FN_RETURN_TYPE_SUCCESS 0
#define SENSOR_FN_RETURN_TYPE_FAIL -1
#define SENSOR_RESTART_FAILURE -2

typedef enum
  {
    MMSS_EXPOSURE_NEG_2,
    MMSS_EXPOSURE_NEG_1,
    MMSS_EXPOSURE_0,
    MMSS_EXPOSURE_POS_1,
    MMSS_EXPOSURE_POS_2
  } MMSS_EXPOSURE_T;

typedef enum
  {
    MMSS_ZOOM_1X,
    MMSS_ZOOM_2X,
    MMSS_ZOOM_4X,
    MMSS_ZOOM_8X
  } MMSS_ZOOM_T;

typedef enum
  {
    MMSS_VIDEO_CIF,
    MMSS_VIDEO_QCIF,
    MMSS_VIDEO_SQCIF,
    MMSS_VIDEO_208_240,
    MMSS_VIDEO_QVGA,
    MMSS_VIDEO_QCIF2,
    MMSS_VIDEO_SQCIF2
  } MMSS_VIDEO_RESOLUTION_T;

typedef enum
  {
    MMSS_AMBIENT_LIGHTING_AUTOMATIC,
    MMSS_AMBIENT_LIGHTING_SUNNY,
    MMSS_AMBIENT_LIGHTING_CLOUDY,
    MMSS_AMBIENT_LIGHTING_INDOOR_HOME,
    MMSS_AMBIENT_LIGHTING_INDOOR_OFFICE,
    MMSS_AMBIENT_LIGHTING_NIGHT
  } MMSS_AMBIENT_LIGHTING_T;

typedef enum
  {
    MMSS_SOURCE_LIGHT_FREQUENCY_AUTOMATIC,
    MMSS_SOURCE_LIGHT_FREQUENCY_50HZ,
    MMSS_SOURCE_LIGHT_FREQUENCY_60HZ
  } MMSS_SOURCE_LIGHT_FREQUENCY_T;

typedef enum
  {
    MMSS_STYLE_COLOR,
    MMSS_STYLE_BLACK_WHITE,
    MMSS_STYLE_ANTIQUE,
    MMSS_STYLE_BLUISH,
    MMSS_STYLE_REDDISH,
    MMSS_STYLE_GREENISH,
    MMSS_STYLE_NEGATIVE
  } MMSS_STYLE_T;

typedef enum
  {
    MMSS_STILL_IMAGE_1600_1200,
    MMSS_STILL_IMAGE_SXGA,
    MMSS_STILL_IMAGE_1280_960,
    MMSS_STILL_IMAGE_1024_768,
    MMSS_STILL_IMAGE_VGA,
    MMSS_STILL_IMAGE_QVGA,
    MMSS_STILL_IMAGE_QQVGA
  } MMSS_STILL_IMAGE_RESOLUTION_T;

extern int i2c_sensor2m_read(u16 addr, u16 *pvalue);
extern int i2c_sensor2m_write(u16 addr, u16 value);
extern int mxc_camera_getmode(void);

static int sensor_id = -1;
static int sensor_revision = -1;

static u16 sensorVfWidth = MAX_VF_WIDTH;
static u16 sensorVfHeight = MAX_VF_HEIGHT;
static u16 sensorWidth = MAX_WIDTH;
static u16 sensorHeight = MAX_HEIGHT;
static u16 outWidth = DEFT_VF_WIDTH;
static u16 outHeight = DEFT_VF_HEIGHT;
static u16 captureWidth = DEFT_WIDTH;
static u16 captureHeight = DEFT_HEIGHT;
static u16 video_zoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
static u16 image_zoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
static UINT16 image_cropx0;
static UINT16 image_cropx1;
static UINT16 image_cropy0;
static UINT16 image_cropy1;
static UINT16 image_width;
static UINT16 image_height;

#ifdef  CONFIG_CAMERA_ROTATE_180
/* default to set rows and columns mirror if camera sensor rotate 180 degree */
static const int rotate_mirror_rows = 1;
static const int rotate_mirror_columns = 1;
#else
static const int rotate_mirror_rows = 0;
static const int rotate_mirror_columns = 0;
#endif
/* set through ioctl by application */
static int mirror_rows;
static int mirror_columns;

/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/
#define MAX_POLL_TRIES                   100

/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
                                 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     LOCAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
                                       GLOBAL FUNCTIONS
==================================================================================================*/

int sensor2m_enter_9(void)
{
  unsigned short value;
  unsigned long tries=0;
  SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;

  if((ret_type = i2c_sensor2m_write(0x338C, 0xA104))) goto done;
  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
  if(value == 0x0007)
    {
      if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
      value &= 0xFF3F;
      if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;
    }
  else if(value == 0x0003)
    {
      if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
      value |= 0x0080;
      if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;
    }

  if((ret_type = i2c_sensor2m_read(0x3202, &value))) goto done;
  value |= 0x0009;
  if((ret_type = i2c_sensor2m_write(0x3202, value))) goto done;

  do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3204, &value))) goto done;
    } while(((value&0x0001)!=0x0001) && (tries++<10));

  if((value&0x0001)!=0x0001)
    {
      printk("MXC_CAMERA: entering standby is not complete!\n");
    }

 done:
  if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
    {
      printk("MXC_CAMERA: standby_enter() failed!\n");
      return SENSOR_ERR_TIMEOUT;
    }

  return SENSOR_ERR_NONE;
}

int sensor2m_exit_9(void)
{
  unsigned short value;
  SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;

  if((ret_type = i2c_sensor2m_read(0x3202, &value))) goto done;
  value &= 0xFFF6;
  if((ret_type = i2c_sensor2m_write(0x3202, value))) goto done;

  if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
  if((value&0x00C0)!=0x0080)
    {
      if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
      if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done;
    }

  if((ret_type = i2c_sensor2m_read(0x301A, &value))) goto done;
  value |= 0x0080;
  if((ret_type = i2c_sensor2m_write(0x301A, value))) goto done;

 done:
  if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
    {
      printk("MXC_CAMERA: standby_exit() failed!\n");
      return SENSOR_ERR_TIMEOUT;
    }

  return SENSOR_ERR_NONE;
}

#ifdef CONFIG_MACH_ASCENSION
SENSOR_FN_RETURN_TYPE_T
sensor2m_initialize_sensor
(
 void
)
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 value = 0; 
    UINT32 poll_counter = 0;

    if((ret_type = i2c_sensor2m_write(0x3214, 0x0777))) goto done; 
      
    if((ret_type = i2c_sensor2m_write(0x301A, 0x0ACC))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3202, 0x0008))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x33F4, 0x031D))) goto done;  

    if(sensor_revision == 2)
      {
	if((ret_type = i2c_sensor2m_read(0x3386, &value))) goto done;;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3386, value))) goto done;

	if((ret_type = i2c_sensor2m_write(0x338C, 0x0400))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x308F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xC3FF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xED8F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x358F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x308F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x158F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0410))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xCC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x07BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0560))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBD9E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x9FF6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0322))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x30E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0AF6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0420))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0239))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xC101))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x2605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x23E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0A7D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0321))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x2720))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0430))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xF602))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x39E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x028F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0B30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xFE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x37EE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0440))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x045F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xAD00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x30E6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0A4F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xED08))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC11))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xA308))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xDD56))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0450))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x30C6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x133A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x3539))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x3C3C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x34BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xAB16))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x30E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x02CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0460))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x011F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xBD82))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xCF5F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x03CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x011F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0470))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xB94F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xE303))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xBD82))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBB30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xE602))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x3838))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3139))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0480))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xB9F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x01BB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x2306))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xBBF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x01B9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xB9F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0490))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x01BA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x2406))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBAF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x01B9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xCE01))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x4F1C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x5308))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x1302))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0406))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xEE00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xEE06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xAD00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x393C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3C3C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x3CC6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04B0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x01F7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0321))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xC60A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF703))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x22F7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0323))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xCC03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0330))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04C0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xFE10))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x50EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x04FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x02FF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xFE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xFFEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x00FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04D0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0301))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x5F4F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x06EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xF303))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x018F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04E0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x00EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xE304))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x18ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x00EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04F0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x06C3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0001))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xED06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x8300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0F25))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xDCEE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x04CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0400))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0500))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xCC03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x03DD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x52CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0328))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x02FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x1050))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0510))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xFD03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x24FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0324))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xFD03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x265F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x4F30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0520))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x05F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0326))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x8FEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0030))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0530))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x05E3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0218))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x8FEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x01ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0540))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0683))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x000A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x25DC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xEE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xCC04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x56ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0230))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0550))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xCC04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x7EED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x10CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0328))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xFD01))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x4F38))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3838))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x3839))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0560))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x3736))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x8F30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xE300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x8F18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x8F18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x3018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xE300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0570))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x3233))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x36A6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xA700))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0918))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x09C0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0124))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0580))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x8001))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x24EE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x8584))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0039))) goto done;
	
	if((ret_type = i2c_sensor2m_read(0x3386, &value))) goto done;
	value &= 0xFFFE;
	if((ret_type = i2c_sensor2m_write(0x3386, value))) goto done;
	
	if((ret_type = i2c_sensor2m_write(0x338C, 0x104D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
      }

    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F09))) goto done;
    if((ret_type = i2c_sensor2m_write(0x341C, 0x0AE5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F09))) goto done;
    msleep(5);
    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F08))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA104))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x03 &&
               poll_counter++ < 50);
	       
    if((value&0x000F)!= 0x03)
      printk("MXC_CAMERA: preview mode switch failed in initialize_sensor!\n");

    if(sensor_revision == 2)
      {
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2003))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x04AB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA002))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done;
      }

    if((ret_type = i2c_sensor2m_write(0x338C, 0x2715))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0185))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2717))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2112))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x046C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x271B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0122))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x271D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x007B))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x271F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x013F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2721))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00AB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2723))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x03B1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2725))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04E2))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2727))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2020))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2729))) goto done;        
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2020))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272B))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x1020))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272D))) goto done;        	
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2007))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2737))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x07BF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2739))) goto done;        	
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2111))) goto done;         
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0024))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x273D))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0120))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2741))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0169))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2745))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x052C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2747))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x09C3))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x222E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0073))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA408))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0016))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA409))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0018))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001C))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x2411))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0073))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2413))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2415))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0073))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2417))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA410))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done; 
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2751))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2753))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0320))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2755))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2757))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0258))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x275F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2761))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0640))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2763))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2765))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04B0))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2703))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0320))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2705))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0258))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2707))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0640))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2709))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04B0))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x270D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x270F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2711))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04BD))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2713))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x064D))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2731))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2733))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04BB))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2735))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x064B))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA115))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00EF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA116))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0030))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA117))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0055))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA118))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA119))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0010))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xAB05))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA770))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0014))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA771))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0025))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA772))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA773))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0057))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA774))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x006E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA775))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0081))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA776))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0091))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA777))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x009F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA778))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00AB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA779))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00B6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00C1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00CB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00DF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00E8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA780))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F9))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA781))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00FF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA782))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA783))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0014))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA784))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0025))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA785))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA786))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0057))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA787))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x006E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA788))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0081))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA789))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0091))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x009F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00AB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00B6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00C1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00CB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA790))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00DF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA791))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00E8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA792))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA793))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F9))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA794))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00FF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done;
    msleep(200);
    if((ret_type = i2c_sensor2m_write(0x35A4, 0x0596))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA118))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA119))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA13E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA13F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA140))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA141))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA142))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA143))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA144))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA145))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA146))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA147))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003A))) goto done;
			
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA20E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0074))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    value |= 0x0072;
    if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x017D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCC))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0189))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC7))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFF31))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0237))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0045))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x004E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0030))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE4))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFEC))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x004D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x009C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFF17))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE7))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00c8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00E1))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA364))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA365))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA366))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA367))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA368))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA369))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA206))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0040))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA404))) goto done; 
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    value &= 0xFFBF;
    if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;        
    msleep(200);    
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done; 
    msleep(200);

    if((ret_type = i2c_sensor2m_write(0x33F4, 0x031D))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x338C, 0x1078))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0FFF))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x1070))) goto done;
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;

    if((value&0x001C)==0x04)
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x21A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3296))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9664))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x5028))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2878))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x7850))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x00F5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00CA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0E9B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0C82))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0xFFF3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x08FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x1D28))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x2A29))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x2E33))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x2F3A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x09E4))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0xE98F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x0529))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x05D2))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x0410))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x0605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x00F4))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x0091))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0FB5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0FC5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0xAAD7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x01E8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x1722))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x2620))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x2029))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x2D1F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0xF6FA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0x0E10))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x0655))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x052A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x0532))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x002B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x00FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x0072))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0ED8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0C62))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x3AFD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x05F2))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x141A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x271E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x2023))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x101B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0xFBFB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x0D20))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x0033))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x03D6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0512))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x0504))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x00D3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x0080))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0EB9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0DA8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0xFAF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x060A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x1D12))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x2C33))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x2021))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x161C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0xFFEA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0xFBCF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x00E2))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x0678))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x04B5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0653))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else if ((value&0x001C)==0x00)
      {
    if((ret_type = i2c_sensor2m_write(0x34CE, 0x81E8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34D0, 0x6332))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34D2, 0x3395))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34D4, 0x9866))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34D6, 0x4924))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34D8, 0x276D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34DA, 0x754E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34DC, 0xFBFE))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34DE, 0x00DF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34E6, 0x00D3))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34EE, 0x0F15))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34F6, 0x0953))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3500, 0x03BD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3508, 0xFEDC))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3510, 0x3B4C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3518, 0x475C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3520, 0x5E52))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3528, 0x6557))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3530, 0x1AC0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3538, 0xEFAF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x354C, 0x0506))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3544, 0x04FC))) goto done;
    if((ret_type = i2c_sensor2m_write(0x355C, 0x048A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3554, 0x051E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34E0, 0x00B4))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34E8, 0x0074))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34F0, 0x002E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34F8, 0x0C2C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3502, 0x00AD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x350A, 0x03BB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3512, 0x2055))) goto done;
    if((ret_type = i2c_sensor2m_write(0x351A, 0x3139))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3522, 0x3832))) goto done;
    if((ret_type = i2c_sensor2m_write(0x352A, 0x3C2F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3532, 0x19F3))) goto done;
    if((ret_type = i2c_sensor2m_write(0x353A, 0xF510))) goto done;
    if((ret_type = i2c_sensor2m_write(0x354E, 0x05D6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3546, 0x0445))) goto done;
    if((ret_type = i2c_sensor2m_write(0x355E, 0x0550))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3556, 0x020F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34E4, 0x0097))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34EC, 0x0048))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34F4, 0x009F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34FC, 0x0C0D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3506, 0x31AD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x350E, 0xFBC1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3516, 0x2F43))) goto done;
    if((ret_type = i2c_sensor2m_write(0x351E, 0x3730))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3526, 0x152D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x352E, 0x1F25))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3536, 0xFF0E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x353E, 0xF4DF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3552, 0x0506))) goto done;
    if((ret_type = i2c_sensor2m_write(0x354A, 0x031B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3562, 0x05DD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x355A, 0x04D0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34E2, 0x00C0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34EA, 0x0067))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34F2, 0x0E49))) goto done;
    if((ret_type = i2c_sensor2m_write(0x34FA, 0x0CB7))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3504, 0xEEEA))) goto done;
    if((ret_type = i2c_sensor2m_write(0x350C, 0x0DF4))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3514, 0x3032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x351C, 0x3F46))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3524, 0x2D3D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x352C, 0x403A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3534, 0x10C2))) goto done;
    if((ret_type = i2c_sensor2m_write(0x353C, 0xCFDA))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3550, 0x043D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3548, 0x04FA))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3560, 0x052C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3558, 0x057B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3540, 0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else if ((value&0x001C)==0x0C)
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x21A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3297))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9765))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x4B26))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2671))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x714C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x0103))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00E0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0ED6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0CC0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0xECE7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x09F9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x282D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x322F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x2934))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x3335))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x1AE8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0xF7AF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x05F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x05F6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x0571))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x05BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x010F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00AA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0F47))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0CE1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0x08D3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x04F8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x202A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x1920))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x2926))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x251D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0x100B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0xFFFF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x064F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x0533))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x05B3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x00A9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x00E1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x007C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0F45))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0CA1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x17E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x13F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x2621))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x1C1D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x1C20))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x111C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x0903))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x11F5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x054F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x0289))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x0542))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x00F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x008E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0E47))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0C5D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0x13F8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x160F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x2926))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x2224))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x232A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x131E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0x1200))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0xEFB2))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x007E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x068E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x059A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x2158))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3296))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9664))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x5028))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2878))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x7850))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x0173))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00CD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0D8B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0DB8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0x0236))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x160A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x0D12))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x1315))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x1714))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x1416))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x0E0D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0x120B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x00F9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x002C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x012B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x0422))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x018B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00A7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0DC5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0DF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0x0F29))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x0A0D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x0B0F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x1413))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x1616))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x0C0A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0x0B0C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0x1224))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x0186))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x0164))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x00B5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x0130))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x016F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x0082))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0D98))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0E74))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x0537))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x0C0C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x0A0F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x1210))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x0E10))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x0B09))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x0C09))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x1F22))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x02CF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x036A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0211))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x016D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x0169))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x00A5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0DDA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0E05))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0x0C31))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x0B08))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x0B08))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x1717))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x1215))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x0C0D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0x0B09))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0x141C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x01D3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x00F4))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x0184))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0091))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
	if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
	value |= 0x0004;
	if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA130))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA132))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA134))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA137))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA218))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A4))) goto done;

    if((ret_type = i2c_sensor2m_read(0x3386, &value))) goto done;
    if(value&0xE000)
      {
	printk("MXC_CAMERA: sensor restart occurred, error=0x%04x\n", value);
	return SENSOR_RESTART_FAILURE;
      }

 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: initialize_sensor() failed!\n");

    return  ret_type;
    
}
#else
SENSOR_FN_RETURN_TYPE_T
sensor2m_initialize_sensor
(
 void
)
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 value = 0; 
    UINT32 poll_counter = 0;

    if((ret_type = i2c_sensor2m_write(0x3214, 0x0777))) goto done; 
      
    if((ret_type = i2c_sensor2m_write(0x301A, 0x0ACC))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3202, 0x0008))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x33F4, 0x031D))) goto done;  

    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F09))) goto done;
    if((ret_type = i2c_sensor2m_write(0x341C, 0x0AE5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F09))) goto done;
    msleep(5);
    if((ret_type = i2c_sensor2m_write(0x341E, 0x8F08))) goto done;

    if(sensor_revision <= 2)
      {
	if((ret_type = i2c_sensor2m_read(0x3386, &value))) goto done;;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3386, value))) goto done;

	if((ret_type = i2c_sensor2m_write(0x338C, 0x0400))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x308F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xC3FF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xED8F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x358F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x308F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x158F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0410))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xCC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x07BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0560))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBD9E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x9FF6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0322))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x30E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0AF6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0420))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0239))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xC101))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x2605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x23E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0A7D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0321))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x2720))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0430))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xF602))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x39E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x028F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0B30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xFE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x37EE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0440))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x045F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xAD00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x30E6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0A4F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xED08))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC11))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xA308))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xDD56))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0450))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x30C6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x133A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x3539))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x3C3C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x34BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xAB16))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x30E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x02CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0460))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x011F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xBD82))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xCF5F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x03CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x011F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0470))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xB94F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xE303))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xBD82))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBB30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xE602))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x3838))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3139))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0480))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xB9F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x01BB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x2306))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xBBF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x01B9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xB9F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0490))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x01BA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x2406))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xF601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xBAF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x01B9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xCE01))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x4F1C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x5308))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x1302))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0406))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xEE00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xEE06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xAD00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x393C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3C3C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x3CC6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04B0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x01F7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0321))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xC60A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xF703))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x22F7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0323))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xCC03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0330))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04C0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xFE10))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x50EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x04FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x02FF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xFE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xFFEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x00FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04D0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0301))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x5F4F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x06EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xF303))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x018F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04E0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x00EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0605))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xE304))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x18ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x00EC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x04F0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x06C3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0001))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xED06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x8300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0F25))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xDCEE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x04CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x0400))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0500))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xCC03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x03DD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x52CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0328))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x30ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x02FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x1050))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0510))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xFD03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x24FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0324))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xEC00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xFD03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x265F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x4F30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0520))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xED06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x05F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0326))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x8FEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x0030))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0530))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x05E3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x0218))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x8FEC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xED00))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0xEC06))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xC300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x01ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0540))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0683))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x000A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x25DC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xEE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xCC04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x56ED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0230))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xEE02))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0550))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0xCC04))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x7EED))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x10CC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x0328))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0xFD01))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x4F38))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x3838))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x3839))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0560))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x3736))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x8F30))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0xE300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0x8F18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x8F18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x3018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0xE300))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0x188F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0570))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x3233))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x36A6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3394, 0x0018))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3396, 0xA700))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3398, 0x0918))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339A, 0x09C0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339C, 0x0124))) goto done;
	if((ret_type = i2c_sensor2m_write(0x339E, 0xF432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x0580))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x8001))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3392, 0x24EE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0x8584))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0039))) goto done;
	
	if((ret_type = i2c_sensor2m_read(0x3386, &value))) goto done;
	value &= 0xFFFE;
	if((ret_type = i2c_sensor2m_write(0x3386, value))) goto done;
	
	if((ret_type = i2c_sensor2m_write(0x338C, 0x104D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
      }

    if((ret_type = i2c_sensor2m_write(0x338C,0xA104))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x03 &&
               poll_counter++ < 50);
	       
    if((value&0x000F)!= 0x03)
      printk("MXC_CAMERA: preview mode switch failed in initialize_sensor!\n");

    if(sensor_revision <= 2)
      {
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2003))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x04AB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA002))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done;
      }

    if((ret_type = i2c_sensor2m_write(0x338C, 0x2715))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0017))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2717))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2111))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x046C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x271B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x024F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x271D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0102))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x271F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0279))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2721))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0155))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2723))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x046E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2725))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0824))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2727))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2020))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2729))) goto done;        
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2020))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272B))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x1020))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272D))) goto done;        	
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2007))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2737))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x035D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2739))) goto done;        	
    if((ret_type = i2c_sensor2m_write(0x3390, 0x2111))) goto done;         
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0024))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x273D))) goto done;        	 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0120))) goto done;         
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2741))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0169))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2745))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0506))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2747))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0824))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x222E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA408))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA409))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x001C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0022))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x2411))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2413))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2415))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2417))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA40D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA410))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done; 
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2751))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2753))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0320))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2755))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2757))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0258))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x275F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2761))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0640))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2763))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2765))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04B0))) goto done;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2703))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0320))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2705))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0258))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2707))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0640))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2709))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04B0))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x270D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x270F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2711))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04BD))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2713))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x064D))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x272F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2731))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2733))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x04BB))) goto done;           
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2735))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x064B))) goto done;           
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA115))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00EF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA116))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0030))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA117))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0055))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA118))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA119))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA11F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0010))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xAB05))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA76F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA770))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0014))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA771))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0022))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA772))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0037))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA773))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0058))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA774))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x006F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA775))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0081))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA776))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA777))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x009D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA778))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A9))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA779))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00B5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00C0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00CA))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D4))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00DD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00E6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA77F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00EF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA780))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F7))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA781))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00FF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA782))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA783))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0014))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA784))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0022))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA785))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0037))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA786))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0058))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA787))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x006F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA788))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0081))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA789))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x008F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x009D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A9))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00B5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78D))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00C0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00CA))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA78F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D4))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA790))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00DD))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA791))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00E6))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA792))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00EF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA793))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00F7))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA794))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00FF))) goto done;
    if((ret_type = i2c_sensor2m_write(0x35A4, 0x0596))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA13E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA13F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA140))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA141))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0004))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA142))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA143))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x000F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA144))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA145))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0032))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA146))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA147))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003A))) goto done;
			
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    value |= 0x0070;
    if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0177))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFD8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x01B1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB9))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBE))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x02A1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D8))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFEE5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFFB))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0051))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFAC))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0086))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFF49))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0018))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0xFFED))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0060))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00A0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA364))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA365))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA366))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA367))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA368))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA369))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0080))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA206))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0040))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x338C, 0xA404))) goto done; 
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    value &= 0xFF7F;
    if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x33F4, 0x031D))) goto done; 

    if((ret_type = i2c_sensor2m_write(0x338C, 0x1078))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0FFF))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C, 0x1070))) goto done;
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;

    if((value&0x001C)==0x04)
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x21A8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6533))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3298))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9564))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x4C26))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2672))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x714B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0x0102))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x012D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00FB))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0AFC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0C87))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0xE438))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x11F5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x234F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x2E69))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x3A77))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x2F69))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x1693))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0xFE87))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x05B1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x0653))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x04FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x06A9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x014D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00CA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0CDA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0D26))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0xF1FE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x06BE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x1A49))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x2B60))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x3362))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x2142))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0x1305))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0xFE1B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x0609))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x05F0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x0614))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x049F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x0127))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x00A8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0B28))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0CA0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0xFD32))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x19FD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x2242))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x3269))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x3165))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x1B46))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0xFCA7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0xE8D9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x0474))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x068C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x062B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0708))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x0125))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x009A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0BC9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0C99))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x0B36))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x13DD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x1D34))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x2C54))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x2B50))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x0F38))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x05FA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0xFA18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x05BF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x04C0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0673))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x06AF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else if ((value&0x001C)==0x00)
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x21A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6532))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3297))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9664))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x4B25))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2670))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x724C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0xFF01))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x0173))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00D9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0BC1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0A5E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0x2A3A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x242E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x2226))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x2126))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x2725))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x272F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x1D1F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0x2DF7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x0077))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x0489))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x01D0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x0415))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x016F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00BF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0D0A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0B21))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0x2409))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x1F1E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x182B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x2520))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x2527))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x221C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0x213B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0x2630))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x0441))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x00AE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x040C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x0214))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x018D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x00AC))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0BC7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0AF0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0x2C2F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x2533))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x1C26))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x2628))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x2123))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x1622))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0x1B29))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0x2115))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x01C6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x0099))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x0040))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0022))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x0147))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x008E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0C9D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0B3A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x3B2A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x191F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x1822))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x1F1C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x1C1A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x101A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x192A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x292D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x019D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x0345))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0024))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x00E5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else if ((value&0x001C)==0x0C)
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x21A0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6432))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3297))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9765))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x4B26))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2671))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x714C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x0103))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00E0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0ED6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0CC0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0xECE7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x09F9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x282D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x322F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x2934))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x3335))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x1AE8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0xF7AF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x05F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x05F6))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x0571))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x05BD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x010F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00AA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0F47))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0CE1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0x08D3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x04F8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x202A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x1920))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x2926))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x251D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0x100B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0xFFFF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x064F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x0533))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x05B3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x00A9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x00E1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x007C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0F45))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0CA1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x17E7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x13F1))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x2621))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x1C1D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x1C20))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x111C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x0903))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x11F5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x054F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x0289))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x0542))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x00F3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x008E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0E47))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0C5D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0x13F8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x160F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x2926))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x2224))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x232A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x131E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0x1200))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0xEFB2))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x007E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x068E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x059A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x0603))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    else
      {
	if((ret_type = i2c_sensor2m_write(0x34CE, 0x01A8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3540, 0x0000))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D0, 0x6633))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D2, 0x3299))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D4, 0x9563))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D6, 0x4B25))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34D8, 0x2670))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DA, 0x724C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DC, 0xFF03))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34DE, 0x0136))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E6, 0x00E4))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EE, 0x0A89))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F6, 0x0BB0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3500, 0x064C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3508, 0x13FF))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3510, 0x2134))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3518, 0x2F6E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3520, 0x3672))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3528, 0x2B4F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3530, 0x09DE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3538, 0x07E9))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354C, 0x0656))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3544, 0x068B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355C, 0x0633))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3554, 0x073A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E0, 0x012E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E8, 0x00A4))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F0, 0x0BC7))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F8, 0x0D12))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3502, 0x0826))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350A, 0x00F5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3512, 0x1925))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351A, 0x265D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3522, 0x2F65))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352A, 0x1F1E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3532, 0xFFFD))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353A, 0xFE3F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354E, 0x0673))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3546, 0x057F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355E, 0x0744))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3556, 0x04C5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E2, 0x010E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EA, 0x009A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F2, 0x0AFE))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FA, 0x0D2D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3504, 0x0151))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350C, 0x0601))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3514, 0x1F22))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351C, 0x2963))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3524, 0x2A63))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352C, 0x181F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3534, 0xF8E3))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353C, 0xFE18))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3550, 0x003F))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3548, 0x06B0))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3560, 0x062B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3558, 0x075E))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34E4, 0x0108))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34EC, 0x007C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34F4, 0x0B71))) goto done;
	if((ret_type = i2c_sensor2m_write(0x34FC, 0x0D17))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3506, 0x164B))) goto done;
	if((ret_type = i2c_sensor2m_write(0x350E, 0x03F8))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3516, 0x1A1D))) goto done;
	if((ret_type = i2c_sensor2m_write(0x351E, 0x234C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3526, 0x2552))) goto done;
	if((ret_type = i2c_sensor2m_write(0x352E, 0x0C17))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3536, 0x00FA))) goto done;
	if((ret_type = i2c_sensor2m_write(0x353E, 0x0C5C))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3552, 0x05C5))) goto done;
	if((ret_type = i2c_sensor2m_write(0x354A, 0x035A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3562, 0x0723))) goto done;
	if((ret_type = i2c_sensor2m_write(0x355A, 0x060A))) goto done;
	if((ret_type = i2c_sensor2m_write(0x3542, 0x0000))) goto done;
      }
    if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
    value |= 0x0004;
    if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);

    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0006))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);

 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: initialize_sensor() failed!\n");

    return  ret_type;
    
}
#endif

int sensor2m_set_mirror(int rows, int columns)
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
    UINT16 value;
#ifndef CONFIG_MACH_ASCENSION
    UINT16 poll_counter;
#endif

    if(rows==0)
        mirror_rows = rotate_mirror_rows;
    else
        mirror_rows = (!rotate_mirror_rows)&0x01;

    if(columns==0)
        mirror_columns = rotate_mirror_columns;
    else
        mirror_columns = (!rotate_mirror_columns)&0x01;

    if(mirror_rows == 0 && mirror_columns == 0)
      {
	if((ret_type = i2c_sensor2m_read(0x3040, &value))) goto done;
	value &= ~0x0003;
	if((ret_type = i2c_sensor2m_write(0x3040, value))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
	value &= ~0x0003;
	if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0003;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0003;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done; 
#endif
      }
    else if(mirror_rows == 1 && mirror_columns == 0)
      {
#ifndef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_read(0x3040, &value))) goto done;
	value &= ~0x0002;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3040, value))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
	value &= ~0x0001;
	value |= 0x0002;
	if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done; 
#endif
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0002;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0002;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done;  
#endif
      }
    else if(mirror_rows == 0 && mirror_columns == 1)
      {
#ifndef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_read(0x3040, &value))) goto done;
	value &= ~0x0001;
	value |= 0x0002;
	if((ret_type = i2c_sensor2m_write(0x3040, value))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
	value &= ~0x0002;
	value |= 0x0001;
	if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done; 
#endif
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0001;
	value |= 0x0002;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value &= ~0x0001;
	value |= 0x0002;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done; 
#endif
      }
    else if(mirror_rows == 1 && mirror_columns == 1)
      {
	if((ret_type = i2c_sensor2m_read(0x3040, &value))) goto done;
	value |= 0x0003;
	if((ret_type = i2c_sensor2m_write(0x3040, value))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3210, &value))) goto done;
	value |= 0x0003;
	if((ret_type = i2c_sensor2m_write(0x3210, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x2719))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value |= 0x0003;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x338C, 0x273B))) goto done; 
	if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	value |= 0x0003;
	if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	if((ret_type = i2c_sensor2m_write(0x3390, 0x0006))) goto done; 
#endif
      }

#ifndef CONFIG_MACH_ASCENSION
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0006))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);
#endif

 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_mirror() failed!\n");

    return ret_type;
}

#ifdef CONFIG_MACH_ASCENSION
SENSOR_FN_RETURN_TYPE_T
sensor2m_set_ambient_lighting_attribute
(
 MMSS_AMBIENT_LIGHTING_T ambience_value
 )
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 value;

    switch (ambience_value)
    {
        case MMSS_AMBIENT_LIGHTING_AUTOMATIC:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0074))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
	  
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
    
	  msleep(200); 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0189))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC7))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF31))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0237))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0045))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x004E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0030))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE4))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFEC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x004D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x009C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF17))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE7))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00c8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00E1))) goto done;
	  break;

        case MMSS_AMBIENT_LIGHTING_SUNNY:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value &= 0xFFDF;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
    
	  msleep(200); 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF68))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x016D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB3))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE6))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCD))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x014D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0038))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x002C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  break;

        case MMSS_AMBIENT_LIGHTING_CLOUDY:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value &= 0xFFDF;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
    
	  msleep(200); 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF68))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x016D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB3))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE6))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCD))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x014D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x002A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;  
	  break;

        case MMSS_AMBIENT_LIGHTING_INDOOR_HOME:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value &= 0xFFDF;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
	  msleep(200); 
        
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0189))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC7))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF31))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0237))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0088))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0098))) goto done;
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  break;

        case MMSS_AMBIENT_LIGHTING_INDOOR_OFFICE:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value &= 0xFFDF;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
	  msleep(200); 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA102))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE6))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB2))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF65))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01E9))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0026))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0080))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  break;
	    
        case MMSS_AMBIENT_LIGHTING_NIGHT:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0010))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0074))) goto done;
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
    
	  msleep(200); 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x017D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFCC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0189))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFC7))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF31))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0237))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0045))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x004E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0030))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE4))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFEC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x004D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x009C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF17))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFE7))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00c8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00E1))) goto done;
	  break;
    
        default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;  
	  break;
    }
    
 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_ambient_lighting() failed\n");

    return ret_type;
}
#else
SENSOR_FN_RETURN_TYPE_T
sensor2m_set_ambient_lighting_attribute
(
 MMSS_AMBIENT_LIGHTING_T ambience_value
 )
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 value;
    UINT16 poll_counter;

    switch (ambience_value)
    {
        case MMSS_AMBIENT_LIGHTING_AUTOMATIC:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
	  
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0177))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFD8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01B1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB9))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBE))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x02A1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEE5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFFB))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0051))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFAC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0086))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF49))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0018))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFED))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0060))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done;
	  break;

        case MMSS_AMBIENT_LIGHTING_SUNNY:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x024F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBD))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0202))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF65))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFEF))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF44))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01EA))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0036))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x002C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0078))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0088))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done; 
	  break;

        case MMSS_AMBIENT_LIGHTING_CLOUDY:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x024F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBD))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x000A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF99))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0202))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF65))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFEF))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF44))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01EA))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x002A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0078))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0088))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done; 
	  break;

        case MMSS_AMBIENT_LIGHTING_INDOOR_HOME:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0177))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFD8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01B1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB9))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBE))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x02A1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x001D))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0078))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0090))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done; 
	  break;

        case MMSS_AMBIENT_LIGHTING_INDOOR_OFFICE:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0007))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01E1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF4A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFDF))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01E1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF8E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFD3))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF0E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0228))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0025))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0039))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0078))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0088))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done; 
	  break;
	    
        case MMSS_AMBIENT_LIGHTING_NIGHT:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	  value |= 0x0070;
	  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;

	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0000))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA20C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0010))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA215))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0008))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA12B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0003))) goto done; 
    
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2306))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0177))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2308))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFD8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF9E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x230E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x01B1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2310))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB9))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2312))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFB0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2314))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEBE))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2316))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x02A1))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2318))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0020))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D8))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x231E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFEE5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2320))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0059))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2322))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFFB))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2324))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0051))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2326))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFAC))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2328))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x003F))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0086))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232C))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFF49))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x232E))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0018))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2330))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0xFFED))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34A))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0060))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA34B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A0))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA361))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00D5))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA362))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00ED))) goto done;
	  break;
    
        default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;  
	  goto done;
	  break;
    }

    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);

 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_ambient_lighting() failed\n");

    return ret_type;
}
#endif

SENSOR_FN_RETURN_TYPE_T
sensor2m_set_style_attribute
(
    MMSS_STYLE_T image_style
)
{   
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS; 
#ifndef CONFIG_MACH_ASCENSION
    UINT16 value;
    UINT16 poll_counter;
#endif

    switch(image_style)
    {
        case MMSS_STYLE_COLOR:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6440))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6440))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x334A, 0xB023))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

            break;
            
        case MMSS_STYLE_BLACK_WHITE:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6441))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6441))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

            break;

        case MMSS_STYLE_ANTIQUE:  
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x334A, 0xB023))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

            break;

        case MMSS_STYLE_NEGATIVE:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6443))) goto done;
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6443))) goto done;
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

	    break;

        case MMSS_STYLE_BLUISH:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x334A, 0x1DE3))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif
     
            break;

        case MMSS_STYLE_REDDISH:   
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x334A, 0xA040))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

            break;

        case MMSS_STYLE_GREENISH:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x2799))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x279B))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x6442))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x334A, 0xDEE3))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#endif

            break;

        default:
	    ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
#ifndef CONFIG_MACH_ASCENSION
	    goto done;
#endif
            break;
    } 

#ifndef CONFIG_MACH_ASCENSION
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    if((ret_type = i2c_sensor2m_write(0x3390,0x0005))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C,0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);
#endif
    
 done:
    if(ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_style() failed!\n");

    return ret_type;
}

SENSOR_FN_RETURN_TYPE_T
sensor2m_set_exposure_attribute
(
    MMSS_EXPOSURE_T exposure_mode
)
{

    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;

    UINT16 exposure_value;
#ifndef CONFIG_MACH_ASCENSION
    UINT16 value;
    UINT16 poll_counter;
#endif
  
    switch (exposure_mode)
    {
        case  MMSS_EXPOSURE_NEG_2:
            exposure_value = 0x0020;
            break ;
            
        case  MMSS_EXPOSURE_NEG_1:
            exposure_value = 0x0030;
            break ;
            
        case  MMSS_EXPOSURE_0:
            exposure_value = 0x0040;
            break ;
            
        case  MMSS_EXPOSURE_POS_1:
            exposure_value = 0x0050;
            break ;
            
        case  MMSS_EXPOSURE_POS_2:
            exposure_value = 0x0060;
            break ;
            
        default :
            ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
            goto done;
            break ;
    }
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA206))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, exposure_value))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done;
#ifndef CONFIG_MACH_ASCENSION
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);
#endif

 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_exposure() failed!\n");

    return ret_type; 
}

SENSOR_FN_RETURN_TYPE_T
sensor2m_set_source_light_frequency 
(
 MMSS_SOURCE_LIGHT_FREQUENCY_T light_freq 
)
{
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;

    UINT16 value = 0;
    
    switch(light_freq)
    {
        case MMSS_SOURCE_LIGHT_FREQUENCY_AUTOMATIC:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA404))) goto done;  
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
#ifdef CONFIG_MACH_ASCENSION
	   value &= 0xFFBF;
#else
	   value &= 0xFF7F;
#endif
           if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;  
           break;

        case MMSS_SOURCE_LIGHT_FREQUENCY_50HZ:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA404))) goto done;  
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	   value |= 0xC0;
           if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;  
#ifndef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x222E))) goto done;  
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A6))) goto done;  
#endif
           break;

        case MMSS_SOURCE_LIGHT_FREQUENCY_60HZ:
	  if((ret_type = i2c_sensor2m_write(0x338C, 0xA404))) goto done;  
	  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
	   value = (value & 0xFF3F) | 0x80;
           if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;  
#ifndef CONFIG_MACH_ASCENSION
	  if((ret_type = i2c_sensor2m_write(0x338C, 0x222E))) goto done;  
	  if((ret_type = i2c_sensor2m_write(0x3390, 0x008A))) goto done;  
#endif
           break;
            
        default:
            ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
            break;
    }

 done:    
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_light_freq() failed!\n");

    return ret_type;
}

SENSOR_FN_RETURN_TYPE_T
sensor2m_set_contextA_windows
(
 UINT16 crop_x0,
 UINT16 crop_x1,
 UINT16 crop_y0,
 UINT16 crop_y1,
 UINT16 width,
 UINT16 height
)
{    
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 poll_counter;
    UINT16 value = 0;

#ifndef CONFIG_MACH_ASCENSION
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    value &= 0xFFFD;
    if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;
#endif
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0001))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA104))) goto done;
    poll_counter = 0;
    do
    {
      msleep(20);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    }  while ( (value&0x000F)!= 0x03 &&
               poll_counter++ < MAX_POLL_TRIES);

    if((ret_type = i2c_sensor2m_write(0x338C, 0x2751))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, crop_x0))) goto done;    
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2753))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, crop_x1))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2755))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, crop_y0))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2757))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, crop_y1))) goto done;
      
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2703))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, width))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0x2705))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, height))) goto done;
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
    msleep(200); 
#else
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
    poll_counter = 0;
    do
    {
      msleep(10);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    } while ( (value&0x000F)!= 0x00 &&
               poll_counter++ < 20);
#endif
      
 done:
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_contextA_windows() failed!\n");

    return ret_type;
}

SENSOR_FN_RETURN_TYPE_T
sensor2m_preload_contextB_windows
(
 UINT16 crop_x0,
 UINT16 crop_x1,
 UINT16 crop_y0,
 UINT16 crop_y1,
 UINT16 width,
 UINT16 height
)
{
  SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
#ifndef CONFIG_MACH_ASCENSION
  UINT16 poll_counter;
  UINT16 value;
#endif

  if((ret_type = i2c_sensor2m_write(0x338C, 0x275F))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, crop_x0))) goto done;
  if((ret_type = i2c_sensor2m_write(0x338C, 0x2761))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, crop_x1))) goto done;
  if((ret_type = i2c_sensor2m_write(0x338C, 0x2763))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, crop_y0))) goto done;
  if((ret_type = i2c_sensor2m_write(0x338C, 0x2765))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, crop_y1))) goto done;
    
  if((ret_type = i2c_sensor2m_write(0x338C, 0x2707))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, width))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x338C, 0x2709))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, height))) goto done;

  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done; 
  if((ret_type = i2c_sensor2m_write(0x3390, 0x0005))) goto done; 
#ifdef CONFIG_MACH_ASCENSION
  msleep(200); 
#else
  if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;
  poll_counter = 0;
  do
  {
    msleep(10);
    if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
  } while ( (value&0x000F)!= 0x00 &&
             poll_counter++ < 20);

  if((ret_type = i2c_sensor2m_write(0x338C, 0xA120))) goto done;
  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
  value |= 0x0002;
  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;
  if((ret_type = i2c_sensor2m_write(0x338C, 0xA136))) goto done;  
  if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
  value |= 0x0040;
  if((ret_type = i2c_sensor2m_write(0x3390, value))) goto done;
  if((ret_type = i2c_sensor2m_write(0x338C, 0xA218))) goto done;  
  if((ret_type = i2c_sensor2m_write(0x3390, 0x00A4))) goto done;
#endif

 done:
  if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
    printk("MXC_CAMERA: preload_contextB_windows() failed!\n");

  return ret_type;
}

SENSOR_FN_RETURN_TYPE_T
sensor2m_set_contextB_windows
(
 UINT16 crop_x0,
 UINT16 crop_x1,
 UINT16 crop_y0,
 UINT16 crop_y1,
 UINT16 width,
 UINT16 height
)
{   
    
    SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
    UINT16 poll_counter = 0;
    UINT16 value = 0;
    
    if((ret_type = i2c_sensor2m_write(0x338C, 0xA103))) goto done;  
    if((ret_type = i2c_sensor2m_write(0x3390, 0x0002))) goto done;

    if((ret_type = i2c_sensor2m_write(0x338C,0xA104))) goto done;
    poll_counter = 0;
    do
    {
      msleep(20);
      if((ret_type = i2c_sensor2m_read(0x3390, &value))) goto done;
    }  while ( (value&0x000F)!= 0x07 &&
               poll_counter++ < MAX_POLL_TRIES);

 done:     
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      printk("MXC_CAMERA: set_contextB_windows() failed!\n");

    return ret_type;
}

u16 sensor2m_reg_read(u16 reg_addr)
{
    u16 value=0;
    i2c_sensor2m_read(reg_addr, &value);
    return value;
}

void sensor2m_reg_write(u16 reg_addr, u16 reg_value)
{
    i2c_sensor2m_write(reg_addr, reg_value);
}


int sensor2m_get_device_id(u16 *id, u16 *rev)
{
    sensor_id = sensor2m_reg_read(0x3000);

    if(id!=NULL)
        *id = sensor_id;

    sensor_revision = sensor2m_reg_read(0x31FA);
    sensor_revision = (sensor_revision & 0x07E0)>>5;

    if(rev!=NULL)
        *rev = sensor_revision;

    return SENSOR_ERR_NONE;
}

int sensor2m_viewfinder_on()
{
  MMSS_VIDEO_RESOLUTION_T video_size;
  MMSS_STILL_IMAGE_RESOLUTION_T image_size;
  MMSS_ZOOM_T zoom;
  int zoom_factor;
  SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;
  int preload_context_b = 0;

  zoom_factor = video_zoom / CAMERA_ZOOM_LEVEL_MULTIPLE;

  if(zoom_factor == 1)
    {
      zoom = MMSS_ZOOM_1X;
    }
  else if(zoom_factor == 2)
    {
      zoom = MMSS_ZOOM_2X;
    }
  else if(zoom_factor == 4)
    {
      zoom = MMSS_ZOOM_4X;
    }
  else if(zoom_factor == 8)
    {
      zoom = MMSS_ZOOM_8X;
    }
  else
    {
      printk("CAMERA: unsupported zoom factor\n");
      return SENSOR_ERR_PARAMETER;
    }

  if (outWidth == 96 && outHeight == 128)
    video_size = MMSS_VIDEO_SQCIF;
  else if(outWidth == 128 && outHeight == 96)
    video_size = MMSS_VIDEO_SQCIF2;
  else if (outWidth == 144 && outHeight == 176)
    video_size = MMSS_VIDEO_QCIF;
  else if (outWidth == 176 && outHeight == 144)
    video_size = MMSS_VIDEO_QCIF2;
  else if (outWidth == 208 && outHeight == 240)
    video_size = MMSS_VIDEO_208_240;
  else if (outWidth == 288 && outHeight == 352)
    video_size = MMSS_VIDEO_CIF;
  else
    video_size = MMSS_VIDEO_QVGA;

  switch(video_size)
    {
    case MMSS_VIDEO_QVGA:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(0,800,0,600, 320, 240);
	  preload_context_b = 1;
	  break;
          
	case MMSS_ZOOM_2X:  
	  ret_type = sensor2m_set_contextA_windows(200,600,150,450, 320, 240);
	  preload_context_b = 1;
	  break;
          
	case MMSS_ZOOM_4X:  
	  ret_type = sensor2m_set_contextA_windows(200,600,150,450, 320, 240);
	  preload_context_b = 1;
	  break;

	case MMSS_ZOOM_8X:
	  ret_type = sensor2m_set_contextA_windows(200,600,150,450,320,240);
	  preload_context_b = 1;
	  break;    
	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;

    case MMSS_VIDEO_CIF:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(155, 645, 0, 600, 288, 352); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(155, 645, 0, 600, 288, 352); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(155, 645, 0, 600, 288, 352); 
	  break; 
          
	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;
      
    case MMSS_VIDEO_208_240:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:
	  ret_type = sensor2m_set_contextA_windows(140, 660, 0, 600, 208, 240); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(270, 530, 150, 450, 208, 240); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(270, 530, 150, 450, 208, 240); 
	  break; 
          
	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;

    case MMSS_VIDEO_QCIF:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(155, 645, 0, 600, 144, 176); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(277, 522, 150, 450, 144, 176); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(277, 522, 150, 450, 144, 176); 
	  break; 
          
	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;

    case MMSS_VIDEO_QCIF2:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(33, 766, 0, 600, 176, 144); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(217, 583, 150, 450, 176, 144); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(308, 491, 225, 375, 176, 144); 
	  break; 

	case MMSS_ZOOM_8X:
	  ret_type = sensor2m_set_contextA_windows(308, 491, 225, 375, 176, 144);
	  break; 
          
	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;

    case MMSS_VIDEO_SQCIF:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(175, 625, 0, 600, 96, 128); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(287, 512, 150, 450, 96, 128); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(344, 456, 225, 375, 96, 128); 
	  break; 
          
	case MMSS_ZOOM_8X:
	  ret_type = sensor2m_set_contextA_windows(344, 456, 225, 375, 96, 128);
	  break; 

	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;
      
    case MMSS_VIDEO_SQCIF2:
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  ret_type = sensor2m_set_contextA_windows(0, 800, 0, 600, 128, 96); 
	  break;
          
	case MMSS_ZOOM_2X: 
	  ret_type = sensor2m_set_contextA_windows(200, 600, 150, 450, 128, 96); 
	  break;
          
	case MMSS_ZOOM_4X: 
	  ret_type = sensor2m_set_contextA_windows(300, 500, 225, 375, 128, 96); 
	  break; 
          
	case MMSS_ZOOM_8X:
	  ret_type = sensor2m_set_contextA_windows(300, 500, 225, 375, 128, 96);
	  break; 

	default:
	  ret_type = SENSOR_FN_RETURN_TYPE_FAIL;
	  goto done;
	}
      break;

    default:
      ret_type   = SENSOR_FN_RETURN_TYPE_FAIL;
      goto done;
    }

  if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
    {
      printk("MXC_CAMERA: viewfinder_on() failed!\n");
      return SENSOR_ERR_TIMEOUT;
    }

  if(preload_context_b)
    {
      zoom_factor = image_zoom / CAMERA_ZOOM_LEVEL_MULTIPLE;
      
      if(zoom_factor == 1)
	{
	  zoom = MMSS_ZOOM_1X;
	}
      else if(zoom_factor == 2)
	{
	  zoom = MMSS_ZOOM_2X;
	}
      else if(zoom_factor == 4)
	{
	  zoom = MMSS_ZOOM_4X;
	}
      else if(zoom_factor == 8)
	{
	  zoom = MMSS_ZOOM_8X;
	}
      else
	{
	  printk("CAMERA: unsupported zoom factor\n");
	  return SENSOR_ERR_PARAMETER;
	}

      if (captureWidth == 160 && captureHeight == 120)
	image_size = MMSS_STILL_IMAGE_QQVGA;
      else if (captureWidth == 320 && captureHeight == 240)
	image_size = MMSS_STILL_IMAGE_QVGA;
      else if (captureWidth == 640 && captureHeight == 480)
	image_size = MMSS_STILL_IMAGE_VGA;
      else if (captureWidth == 1024 && captureHeight == 768)
	image_size = MMSS_STILL_IMAGE_1024_768;
      else if (captureWidth == 1280 && captureHeight == 960)
	image_size = MMSS_STILL_IMAGE_1280_960;
      else if (captureWidth == 1280 && captureHeight == 1024)
	image_size = MMSS_STILL_IMAGE_SXGA;
      else if (captureWidth == 1600 && captureHeight == 1200)
	image_size = MMSS_STILL_IMAGE_1600_1200;
      else
	{
	  printk("CAMERA: unsupported dimension\n");
	  return SENSOR_ERR_PARAMETER;
	}
      
      switch(zoom)
	{
	case MMSS_ZOOM_1X:  
	  switch (image_size)
	    {
	    case MMSS_STILL_IMAGE_1600_1200:
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 1600;
	      image_height = 1200;
	      break;
	    case MMSS_STILL_IMAGE_SXGA:
	      image_cropx0 = 50;
	      image_cropx1 = 1550;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 1280;
	      image_height = 1024;
	      break;
	    case MMSS_STILL_IMAGE_1280_960:
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 1280;
	      image_height = 960;
	      break;
	    case MMSS_STILL_IMAGE_1024_768:
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 1024;
	      image_height = 768;
	      break;
	    case MMSS_STILL_IMAGE_VGA:                 
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 640;
	      image_height = 480;
	      break;
	    case MMSS_STILL_IMAGE_QVGA:
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 320;
	      image_height = 240;
	      break;
	    case MMSS_STILL_IMAGE_QQVGA:
	      image_cropx0 = 0;
	      image_cropx1 = 1600;
	      image_cropy0 = 0;
	      image_cropy1 = 1200;
	      image_width = 160;
	      image_height = 120;
	      break;
	    }            
	  break;
	  
	case MMSS_ZOOM_2X:  
	  switch (image_size)
	    {
	    case MMSS_STILL_IMAGE_1600_1200:
	    case MMSS_STILL_IMAGE_SXGA:
	    case MMSS_STILL_IMAGE_1280_960:
	    case MMSS_STILL_IMAGE_1024_768:
	    case MMSS_STILL_IMAGE_VGA:                 
	      image_cropx0 = 400;
	      image_cropx1 = 1200;
	      image_cropy0 = 300;
	      image_cropy1 = 900;
	      image_width = 640;
	      image_height = 480;
	      break;
	    case MMSS_STILL_IMAGE_QVGA:
	      image_cropx0 = 400;
	      image_cropx1 = 1200;
	      image_cropy0 = 300;
	      image_cropy1 = 900;
	      image_width = 320;
	      image_height = 240;
	      break;
	    case MMSS_STILL_IMAGE_QQVGA:
	      image_cropx0 = 400;
	      image_cropx1 = 1200;
	      image_cropy0 = 300;
	      image_cropy1 = 900;
	      image_width = 160;
	      image_height = 120;
	      break;
	    }		
	  break;
	  
	case MMSS_ZOOM_4X:  
	  switch (image_size)
	    {
	    case MMSS_STILL_IMAGE_1600_1200:
	    case MMSS_STILL_IMAGE_SXGA:
	    case MMSS_STILL_IMAGE_1280_960:
	    case MMSS_STILL_IMAGE_1024_768:
	    case MMSS_STILL_IMAGE_VGA:                 
	    case MMSS_STILL_IMAGE_QVGA:                                                
	      image_cropx0 = 600;
	      image_cropx1 = 1000;
	      image_cropy0 = 450;
	      image_cropy1 = 750;
	      image_width = 320;
	      image_height = 240;
	      break;
	    case MMSS_STILL_IMAGE_QQVGA:
	      image_cropx0 = 600;
	      image_cropx1 = 1000;
	      image_cropy0 = 450;
	      image_cropy1 = 750;
	      image_width = 160;
	      image_height = 120;
	      break;
	    }			
	  break;
	  
	case MMSS_ZOOM_8X:                   
	  switch (image_size)
	    {
	    case MMSS_STILL_IMAGE_1600_1200:
	    case MMSS_STILL_IMAGE_SXGA:
	    case MMSS_STILL_IMAGE_1280_960:
	    case MMSS_STILL_IMAGE_1024_768:
	    case MMSS_STILL_IMAGE_VGA:                 
	    case MMSS_STILL_IMAGE_QVGA:
	    case MMSS_STILL_IMAGE_QQVGA:
	      image_cropx0 = 700;
	      image_cropx1 = 900;
	      image_cropy0 = 525;
	      image_cropy1 = 675;
	      image_width = 160;
	      image_height = 120;
	      break;
	    }		
	  break;    
	}

      ret_type = sensor2m_preload_contextB_windows(image_cropx0, image_cropx1, image_cropy0, image_cropy1, image_width, image_height);
    }

 done:    
    if (ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
      {
	printk("MXC_CAMERA: viewfinder_on() failed!\n");
	return SENSOR_ERR_TIMEOUT;
      }
    else
      {
	return SENSOR_ERR_NONE;
      }
}

int sensor2m_snapshot_trigger()
{
  SENSOR_FN_RETURN_TYPE_T ret_type = SENSOR_FN_RETURN_TYPE_SUCCESS;

  ret_type = sensor2m_set_contextB_windows(image_cropx0, image_cropx1, image_cropy0, image_cropy1, image_width, image_height);

  if(ret_type != SENSOR_FN_RETURN_TYPE_SUCCESS)
    {
      printk("MXC_CAMERA: snapshot_trigger() failed!\n");
      return SENSOR_ERR_TIMEOUT;
    }
  else
    {
      return SENSOR_ERR_NONE;
    }
}

int sensor2m_sensor_size(window_size_t * window)
{
    sensorVfWidth = window->w;
    sensorVfHeight = window->h;

    return SENSOR_ERR_NONE;
}

int sensor2m_capture_sensor_size(window_size_t * window)
{
    sensorWidth = window->w;
    sensorHeight = window->h;

    return SENSOR_ERR_NONE;
}

int sensor2m_output_size(window_size_t * window)
{
    outWidth = window->w;
    outHeight = window->h;

    return SENSOR_ERR_NONE;
}

int sensor2m_capture_size(window_size_t * window)
{
    captureWidth = window->w;
    captureHeight = window->h;

    return SENSOR_ERR_NONE;
}

int sensor2m_set_exposure_mode(int mode, u32 maxexpotime)
{
    if (mode == NM_NIGHT)
        sensor2m_set_ambient_lighting_attribute(MMSS_AMBIENT_LIGHTING_NIGHT);
    else
        sensor2m_set_ambient_lighting_attribute(MMSS_AMBIENT_LIGHTING_AUTOMATIC);

    return SENSOR_ERR_NONE;
}

int sensor2m_set_time(u16 time)
{
    return 0;
}

int sensor2m_set_flicker(int flicker)
{
  MMSS_SOURCE_LIGHT_FREQUENCY_T light_freq;

  if(flicker == 50)
    {
      light_freq = MMSS_SOURCE_LIGHT_FREQUENCY_50HZ;
    }
  else if(flicker == 60)
    {
      light_freq = MMSS_SOURCE_LIGHT_FREQUENCY_60HZ;
    }
  else
    {
      light_freq = MMSS_SOURCE_LIGHT_FREQUENCY_AUTOMATIC;
    }

  sensor2m_set_source_light_frequency(light_freq);

  return SENSOR_ERR_NONE;
}

int sensor2m_set_style(PIC_STYLE_T style)
{
  MMSS_STYLE_T  image_style;

  switch(style)
    {
    case STYLE_BLACK_WHITE:
      image_style = MMSS_STYLE_BLACK_WHITE;
      break;
    case STYLE_SEPIA:
      image_style = MMSS_STYLE_ANTIQUE;
      break;
    case STYLE_SOLARIZE:
      printk("CAMERA: unsupported color style\n");
      return SENSOR_FN_RETURN_TYPE_FAIL;
      break;
    case STYLE_NEG_ART:
      image_style = MMSS_STYLE_NEGATIVE;
      break;
    case STYLE_BLUISH:
      image_style = MMSS_STYLE_BLUISH;
      break ;
    case STYLE_REDDISH:
      image_style = MMSS_STYLE_REDDISH;
      break ;
    case STYLE_GREENISH:
      image_style = MMSS_STYLE_GREENISH;
      break ;
    default:
      image_style = MMSS_STYLE_COLOR;
      break;
    }

    sensor2m_set_style_attribute(image_style);

    return SENSOR_ERR_NONE;
}

        
int sensor2m_set_light(PIC_WB_T light)
{ 
  MMSS_AMBIENT_LIGHTING_T ambience_value;

  switch(light)
    {
    case WB_DIRECT_SUN:
      ambience_value = MMSS_AMBIENT_LIGHTING_SUNNY;
      break;       
      
    case WB_INCANDESCENT:
      ambience_value = MMSS_AMBIENT_LIGHTING_INDOOR_HOME;
      break;       
      
    case WB_FLUORESCENT:
      ambience_value = MMSS_AMBIENT_LIGHTING_INDOOR_OFFICE;
      break;

    case WB_CLOUDY:
      ambience_value = MMSS_AMBIENT_LIGHTING_CLOUDY;
      break;

    default:
      ambience_value = MMSS_AMBIENT_LIGHTING_AUTOMATIC;
      break;      
    }

    sensor2m_set_ambient_lighting_attribute(ambience_value);
    return SENSOR_ERR_NONE;
}

    
int sensor2m_set_bright(int bright)
{
  MMSS_EXPOSURE_T exposure_mode;

  switch(bright)
    {
    case -4:
    case -3:
      exposure_mode = MMSS_EXPOSURE_NEG_2;
      break;
    case -2:
    case -1:
      exposure_mode = MMSS_EXPOSURE_NEG_1;
      break;
    case 0:
      exposure_mode = MMSS_EXPOSURE_0;
      break;
    case 1:
    case 2:
      exposure_mode = MMSS_EXPOSURE_POS_1;
      break;
    case 3:
    case 4:
      exposure_mode = MMSS_EXPOSURE_POS_2;
      break;
    default:
      return SENSOR_ERR_PARAMETER;
    }

  sensor2m_set_exposure_attribute(exposure_mode);

  return SENSOR_ERR_NONE;
}

int sensor2m_default_settings()
{
  int err;
  int count=0;

  image_cropx0 = 0;
  image_cropx1 = MAX_WIDTH;
  image_cropy0 = 0;
  image_cropy1 = MAX_HEIGHT;
  image_width = MAX_WIDTH;
  image_height = MAX_HEIGHT;

  do
    {
      err = sensor2m_initialize_sensor();
    } while((err == SENSOR_RESTART_FAILURE) && count++<2);

  return SENSOR_ERR_NONE;
}

int sensor2m_set_zoom(int azoom, int bzoom)
{
  video_zoom = (u16) azoom;
  image_zoom = (u16) bzoom;
  
  return SENSOR_ERR_NONE;
}
