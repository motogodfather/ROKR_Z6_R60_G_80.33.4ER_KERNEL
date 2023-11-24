/*
 *  sensor2m.c
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
Motorola            06/06/2006     Sensor reset and powerdown sequence
Motorola            12/07/2006     OSS code changes
Motorola            01/04/2007     OSS code changes
Motorola            01/17/2007     Improve zoom quality
Motorola            02/20/2007     Desense improvements

==================================================================================
                                 INCLUDE FILES
==================================================================================*/
#include <linux/types.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>

#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>

#include "sensor2m.h"
#include "sensor2m_hw.h"

#include <asm/arch/gpio.h>

#define LOG_TIME_STAMP   //If defined, the time stamp log will be printed out

#define BUF_SIZE_DEFT     (PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT) + PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT / 2)*2)

#if defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91331)
#define IPU_GPIO_PORT_MCU 	0

#define IPU_GPIO_HIGH 		1

#define CAM_CSI1  		16
#define GPIO_CAM_RST_B 		18
#endif

extern int i2c_sensor2m_cleanup(void);
extern int i2c_sensor2m_init(void);

/***********************************************************************
 *
 * SENSOR2M Functions
 *
 ***********************************************************************/
int camera_func_sensor2m_init(p_camera_context_t);
int camera_func_sensor2m_deinit(p_camera_context_t);
int camera_func_sensor2m_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_sensor2m_set_capture_format(p_camera_context_t);
int camera_func_sensor2m_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_sensor2m_stop_capture(p_camera_context_t);
int camera_func_sensor2m_pm_management(p_camera_context_t, int);


camera_function_t  sensor2m_func = 
{
    init:                camera_func_sensor2m_init,
    deinit:              camera_func_sensor2m_deinit,
    command:             camera_func_sensor2m_docommand,
    set_capture_format:  camera_func_sensor2m_set_capture_format,
    start_capture:       camera_func_sensor2m_start_capture,
    stop_capture:        camera_func_sensor2m_stop_capture,
    pm_management:       camera_func_sensor2m_pm_management
};

static void sensor2m_init(int dma_en)
{
    /* NOTE: configure camera-related pins and pull both
             standby (powerdown) and reset pins low */
    camera_gpio_init();     // only init GPIO mode
    udelay(5);

#ifdef CONFIG_ARCH_MXC91231
    gpio_set_data(GPIO_AP_B_PORT, 24, 0);
    udelay(5);
    /* NOTE: pull reset pin high to bring camera out of reset */
    gpio_set_data(GPIO_AP_B_PORT, 24, 1);
#elif defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91331)
    // pull reset pin high to bring camera out of reset
    gpio_set_data(IPU_GPIO_PORT_MCU, GPIO_CAM_RST_B, IPU_GPIO_HIGH);
#endif

    mdelay(1);
}

static void sensor2m_hw_deinit(void)
{
#ifdef CONFIG_ARCH_MXC91231
    /* NOTE: set STANDBY (POWERDOWN) to high */
    gpio_set_data(GPIO_AP_B_PORT, 25, 1);
#elif defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91331)
    // set POWERDOWN to high
    gpio_set_data(IPU_GPIO_PORT_MCU, CAM_CSI1, IPU_GPIO_HIGH);
#endif

    mdelay(1);

    ci_deinit();
}

static void sensor2m_deinit(void)
{
    sensor2m_enter_9();

    i2c_sensor2m_cleanup();

    sensor2m_hw_deinit();

}

int camera_func_sensor2m_s9(void)
{
    int ret=0;

    u16 device_id = 0 ;
    u16 revision = 0 ;

    /* Turn on power to camera */
    camera_turn_on_vcam();

    /* Enable master clock to sensor */
    ci_init();

    /* Bring sensor out of reset */
    sensor2m_init(1);

    /* Initialize I2C */
    ret = i2c_sensor2m_init();
    if(ret >= 0)
    {
        sensor2m_get_device_id(&device_id, &revision);
        if(device_id != 0x1580 || revision<2) 
        {
	  printk("MXC_CAMERA: unknown device id %04x or unsupported revision %04x!\n", device_id, revision);
	  ret = -ENOTSUPP;
        }
        else
        {
	  printk("MXC_CAMERA: entering standby\n");
	  if((ret = sensor2m_enter_9()))
	    printk("MXC_CAMERA: failed to put sensor in standby mode\n");
        }

	i2c_sensor2m_cleanup();
    }
    else
      printk("MXC_CAMERA: camera i2c addr not found!\n");

    /* Power down the sensor */
    sensor2m_hw_deinit();

    /* Turn off power to camera */
    camera_turn_off_vcam();

    return ret;
}

int camera_func_sensor2m_init(  p_camera_context_t cam_ctx )
{
    u16 device_id = 0 ;
    u16 revision = 0 ;
    int ret;

    cam_ctx->sensor_width  = MAX_WIDTH;
    cam_ctx->sensor_height = MAX_HEIGHT;
    
    cam_ctx->capture_width  = DEFT_VF_WIDTH;
    cam_ctx->capture_height = DEFT_VF_HEIGHT;
    
    cam_ctx->still_width  = MAX_WIDTH;
    cam_ctx->still_height = MAX_HEIGHT;
    
    cam_ctx->frame_rate = cam_ctx->fps = 15;
    cam_ctx->mini_fps = 15;
    
    cam_ctx->buf_size     = BUF_SIZE_DEFT;
    cam_ctx->dma_descriptors_size = (cam_ctx->buf_size/PAGE_SIZE + 10);
    strcpy (cam_ctx->name, "2MP CAMERA");
    cam_ctx->capability.max_width  = MAX_WIDTH;
    cam_ctx->capability.max_height = MAX_HEIGHT;
    cam_ctx->capability.min_width  = MIN_WIDTH; 
    cam_ctx->capability.min_height = MIN_HEIGHT;

    /* TODO: configuration parameters should be specified here and 
             passed into ci_init() */
    ci_init();
 
    sensor2m_init(1);

    ret = i2c_sensor2m_init();
    if(ret < 0)
    {   // i2c failed
        err_print("error: i2c initialize fail!");
        sensor2m_hw_deinit();
        return ret;
    }

    sensor2m_get_device_id(&device_id, &revision);
    if(device_id != 0x1580 || revision<2) 
    {
        err_print("error: unknown device id %04x or unsupported revision %04x!", device_id, revision);
        //camera_gpio_deinit();
        return -ENOTSUPP;
    }

    cam_ctx->sensor_type = CAMERA_TYPE_SENSOR2M;

    sensor2m_default_settings();
    
    return 0;
}
    
int camera_func_sensor2m_deinit(  p_camera_context_t camera_context )
{
    sensor2m_deinit();

    return 0;
}

int camera_func_sensor2m_set_capture_format(  p_camera_context_t camera_context )
{
    window_size_t wsize;
    int aspect, sen_aspect, w, h, swidth, sheight, azoom, bzoom;

    if(camera_context->still_image_mode)
        return 0;

    w = camera_context->capture_width;
    h = camera_context->capture_height;
    if(w < MIN_WIDTH || h < MIN_HEIGHT || w > MAX_VF_WIDTH || h > MAX_VF_HEIGHT)
        return -EINVAL;

    swidth = MAX_VF_WIDTH;
    sheight = MAX_VF_HEIGHT;
    sen_aspect = 10000 * MAX_VF_HEIGHT/MAX_VF_WIDTH;

    aspect = 10000 * h / w;
    if(aspect < sen_aspect)
        sheight = swidth * aspect / 10000;
    else if(aspect > sen_aspect)
        swidth = 10000 * sheight / aspect;
    camera_context->sensor_width = swidth;
    camera_context->sensor_height = sheight;

    azoom = camera_context->capture_digital_zoom;
    bzoom = camera_context->still_digital_zoom;

    if(azoom < CAMERA_ZOOM_LEVEL_MULTIPLE)
        azoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
    if(azoom > CAMERA_ZOOM_LEVEL_MULTIPLE)
    {
        swidth = swidth * CAMERA_ZOOM_LEVEL_MULTIPLE / azoom;
        sheight = sheight * CAMERA_ZOOM_LEVEL_MULTIPLE / azoom;
        if(swidth < camera_context->capture_width)
        {
            swidth = camera_context->capture_width;
            azoom = camera_context->sensor_width * CAMERA_ZOOM_LEVEL_MULTIPLE / swidth;
            camera_context->capture_digital_zoom = azoom;
        }
        if(sheight < camera_context->capture_height)
        {
            sheight = camera_context->capture_height;
        }
	if(azoom < CAMERA_ZOOM_LEVEL_MULTIPLE*2)
	  {
	    azoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
	    swidth = camera_context->sensor_width;
	    sheight = camera_context->sensor_height;
	    camera_context->capture_digital_zoom = azoom;
	  }
	else if(azoom < CAMERA_ZOOM_LEVEL_MULTIPLE*4)
	  {
	    azoom = CAMERA_ZOOM_LEVEL_MULTIPLE*2;
	    swidth = camera_context->sensor_width / 2;
	    sheight = camera_context->sensor_height / 2;
	    camera_context->capture_digital_zoom = azoom;
	  }
	else if(azoom < CAMERA_ZOOM_LEVEL_MULTIPLE*8)
	  {
	    azoom = CAMERA_ZOOM_LEVEL_MULTIPLE*4;
	    swidth = camera_context->sensor_width / 4;
	    sheight = camera_context->sensor_height / 4;
	    camera_context->capture_digital_zoom = azoom;
	  }
	else
	  {
	    azoom = CAMERA_ZOOM_LEVEL_MULTIPLE*8;
	    swidth = camera_context->sensor_width / 8;
	    sheight = camera_context->sensor_height / 8;
	    camera_context->capture_digital_zoom = azoom;
	  }
    }

    wsize.w = swidth;
    wsize.h = sheight;
    sensor2m_sensor_size(&wsize);

    w = camera_context->still_width;
    h = camera_context->still_height;
    if(w < MIN_WIDTH || h < MIN_HEIGHT || w > MAX_WIDTH || h > MAX_HEIGHT)
        return -EINVAL;

    swidth = MAX_WIDTH;
    sheight = MAX_HEIGHT;
    sen_aspect = 10000 * MAX_HEIGHT/MAX_WIDTH;
    aspect = 10000 * h / w;
    if(aspect < sen_aspect)
        sheight = swidth * aspect / 10000;
    else if(aspect > sen_aspect)
        swidth = 10000 * sheight / aspect;
    camera_context->sensor_width = swidth;
    camera_context->sensor_height = sheight;

    if(bzoom < CAMERA_ZOOM_LEVEL_MULTIPLE)
        bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
    if(bzoom > CAMERA_ZOOM_LEVEL_MULTIPLE)
    {
        swidth = swidth * CAMERA_ZOOM_LEVEL_MULTIPLE / bzoom;
        sheight = sheight * CAMERA_ZOOM_LEVEL_MULTIPLE / bzoom;
        if(swidth < camera_context->still_width)
        {
            swidth = camera_context->still_width;
            bzoom = camera_context->sensor_width * CAMERA_ZOOM_LEVEL_MULTIPLE / swidth;
            camera_context->still_digital_zoom = bzoom;
        }
        if(sheight < camera_context->still_height)
        {
            sheight = camera_context->still_height;
        }
	if(bzoom < CAMERA_ZOOM_LEVEL_MULTIPLE*2)
	  {
	    bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
	    swidth = camera_context->sensor_width;
	    sheight = camera_context->sensor_height;
	    camera_context->still_digital_zoom = bzoom;
	  }
	else if(bzoom < CAMERA_ZOOM_LEVEL_MULTIPLE*4)
	  {
	    bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE*2;
	    swidth = camera_context->sensor_width / 2;
	    sheight = camera_context->sensor_height / 2;
	    camera_context->still_digital_zoom = bzoom;
	  }
	else if(bzoom < CAMERA_ZOOM_LEVEL_MULTIPLE*8)
	  {
	    bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE*4;
	    swidth = camera_context->sensor_width / 4;
	    sheight = camera_context->sensor_height / 4;
	    camera_context->still_digital_zoom = bzoom;
	  }
	else
	  {
	    bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE*8;
	    swidth = camera_context->sensor_width / 8;
	    sheight = camera_context->sensor_height / 8;
	    camera_context->still_digital_zoom = bzoom;
	  }
    }

    wsize.w = swidth;
    wsize.h = sheight;
    sensor2m_capture_sensor_size(&wsize);

    wsize.w = camera_context->capture_width;
    wsize.h = camera_context->capture_height;
    sensor2m_output_size(&wsize);

    wsize.w = camera_context->still_width;
    wsize.h = camera_context->still_height;
    sensor2m_capture_size(&wsize);

    sensor2m_set_zoom(azoom, bzoom);

    return 0;
}

int camera_func_sensor2m_start_capture(  p_camera_context_t cam_ctx, unsigned int frames )
{
    int waitingFrame = 0;

    // frames=0 means video mode capture    
    if (frames == 0) 
    {
        sensor2m_viewfinder_on();
    }
    else 
    {
        sensor2m_snapshot_trigger();
    }

    if(frames == 1) //Wait 1 frames to begin capture photo
    {
        waitingFrame = 0;
    } 
    else
    { 
        waitingFrame = 0;
    }
    camera_skip_frame(cam_ctx, waitingFrame);

    return 0;
}

int camera_func_sensor2m_stop_capture(  p_camera_context_t camera_context )
{
    return 0;
}

int camera_func_sensor2m_pm_management(p_camera_context_t cam_ctx, int suspend)
{
    return 0;
}

static int mxc_camera_WCAM_VIDIOCGSNAME(p_camera_context_t cam_ctx, void * param)
{
    if(copy_to_user(param, cam_ctx->name, sizeof(cam_ctx->name))) 
    {
        return -EFAULT;
    } 

    return 0;
}

static int mxc_camera_WCAM_VIDIOCGMINMAX(p_camera_context_t cam_ctx, void * param)
{
    min_max_t cap;

    cap.max_width = cam_ctx->capability.max_width;
    cap.max_height = cam_ctx->capability.max_height;
    cap.min_width = cam_ctx->capability.min_width;
    cap.min_height = cam_ctx->capability.min_height;

    if(copy_to_user(param, &cap, sizeof(min_max_t))) 
    {
        return -EFAULT;
    } 

    return 0;
}

static int mxc_camera_WCAM_VIDIOCGI2CREG(p_camera_context_t cam_ctx, void * param)
{
    camera_i2c_register_t reg;

    if(copy_from_user(&reg, param, sizeof(camera_i2c_register_t))) 
    {
        return -EFAULT;
    }
    reg.value.w = (int)sensor2m_reg_read((u16)reg.addr);

    if(copy_to_user(param, &reg, sizeof(camera_i2c_register_t))) 
    {
        return -EFAULT;
    } 

    return 0;
}

static int mxc_camera_WCAM_VIDIOCSI2CREG(p_camera_context_t cam_ctx, void * param)
{
    camera_i2c_register_t reg;

    if(copy_from_user(&reg, param, sizeof(camera_i2c_register_t))) 
    {
        return  -EFAULT;
    }
    sensor2m_reg_write((u16)reg.addr, (u16)reg.value.w);
    return 0;
} 
 
static int mxc_cam_WCAM_VIDIOCSFPS(p_camera_context_t cam_ctx, void * param)
{
    fps_max_min_t cam_fps;

    if(copy_from_user(&cam_fps, param, sizeof(fps_max_min_t))) 
    {
        return  -EFAULT;
    }
    cam_ctx->fps = cam_fps.maxfps;
    cam_ctx->mini_fps = cam_fps.minfps;
    sensor2m_set_time(cam_ctx->fps);
    return 0;
}


static int mxc_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void * param)
{
  window_size_t size;
  
  if(copy_from_user(&size, param, sizeof(window_size_t))) 
  {
        return  -EFAULT;
  }

  if(size.w > cam_ctx->capability.max_width  ||
     size.h > cam_ctx->capability.max_height || 
     size.w < cam_ctx->capability.min_width   || 
     size.h < cam_ctx->capability.min_height) 
  {
      err_print("WCAM_VIDIOCSSSIZE error parameter!");
      dbg_print("size.w:%d, MAX_WIDTH:%d, MIN_WIDTH:%d", size.w, 
                cam_ctx->capability.max_width, 
                cam_ctx->capability.min_width);
      dbg_print("size.h:%d, MAX_HEIGHT:%d, MIN_HEIGHT:%d", size.h, 
                cam_ctx->capability.max_height, 
                cam_ctx->capability.min_height);
      return  -EFAULT;
  }

  /* NOTE: make dimensions divisible by 8 to accomodate IPU */
  size.w = (size.w+7)/8 * 8;
  size.h = (size.h+7)/8 * 8;
  cam_ctx->sensor_width = size.w;
  cam_ctx->sensor_height = size.h;

  return 0;
}

static int mxc_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{
   window_size_t size;
  
   if(copy_from_user(&size, param, sizeof(window_size_t))) 
   {
        return  -EFAULT;
   }

   /* NOTE: make dimensions divisible by 8 to accomodate IPU */
   size.w = (size.w+7)/8 * 8;
   size.h = (size.h+7)/8 * 8;
   
   cam_ctx->capture_width  = size.w;
   cam_ctx->capture_height = size.h;

   return 0;
}
    
static int mxc_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void * param)
{
  cam_ctx->capture_style = (PIC_STYLE_T)param;
  
  return sensor2m_set_style(cam_ctx->capture_style);
}

static int mxc_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void * param)
{
   cam_ctx->capture_light = (PIC_WB_T)param;

   return  sensor2m_set_light(cam_ctx->capture_light);
}
    
static int mxc_cam_WCAM_VIDIOCSBRIGHT(p_camera_context_t cam_ctx, void * param)
{
   cam_ctx->capture_bright = (int)param;

   return  sensor2m_set_bright((int)param);
}

static int mxc_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
   cam_ctx->flicker_freq = (int)param;

   return  sensor2m_set_flicker(cam_ctx->flicker_freq);
}


static int mxc_cam_WCAM_VIDIOCSNIGHTMODE(p_camera_context_t cam_ctx, void * param)
{
    EXPO_MODE_PARAM_T cam_mode;
    
    if (copy_from_user(&cam_mode, param, sizeof(EXPO_MODE_PARAM_T))) 
    {
        return -EFAULT;
    }

    if(cam_mode.maxexpotime == 0)
    {
        return -EFAULT;
    }

    switch (cam_mode.mode)
    {
        case NM_NIGHT:
        case NM_ACTION:
        case NM_AUTO:
            sensor2m_set_exposure_mode(cam_mode.mode, cam_mode.maxexpotime);
            break;
        default:
            return -EFAULT;
    }

    return 0;
}

static int mxc_cam_WCAM_VIDIOCGEXPOPARA(p_camera_context_t cam_ctx, void * param)
{
    EXPOSURE_PARA_T expo_para;

    if(copy_to_user(param, &expo_para, sizeof(EXPOSURE_PARA_T))) 
    {
        return -EFAULT;
    } 

    return 0;
}

static int mxc_cam_WCAM_VIDIOCSMIRROR(p_camera_context_t cam_ctx, void * param)
{
    int mirror, rows, columns;
    mirror = (int)param;

    if(mirror & CAMERA_MIRROR_VERTICALLY)
        rows = 1;
    else
        rows = 0;

    if(mirror & CAMERA_MIRROR_HORIZONTALLY)
        columns = 1;
    else
        columns = 0;

    return  sensor2m_set_mirror(rows, columns);
}


int camera_func_sensor2m_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
    case WCAM_VIDIOCSSSIZE:
         return mxc_cam_WCAM_VIDIOCSSSIZE(cam_ctx, param);

    case WCAM_VIDIOCSOSIZE:
         return mxc_cam_WCAM_VIDIOCSOSIZE(cam_ctx, param);
         
    case WCAM_VIDIOCSFPS:
         return mxc_cam_WCAM_VIDIOCSFPS(cam_ctx, param);
            
    case WCAM_VIDIOCSSTYLE:
         return mxc_cam_WCAM_VIDIOCSSTYLE(cam_ctx, param);
         
    case WCAM_VIDIOCSLIGHT:
         return mxc_cam_WCAM_VIDIOCSLIGHT(cam_ctx, param);
    
    case WCAM_VIDIOCSBRIGHT:
         return mxc_cam_WCAM_VIDIOCSBRIGHT(cam_ctx, param);
    
    case WCAM_VIDIOCSFLICKER:
         return mxc_cam_WCAM_VIDIOCSFLICKER(cam_ctx, param);

    case WCAM_VIDIOCSNIGHTMODE:
         return mxc_cam_WCAM_VIDIOCSNIGHTMODE(cam_ctx, param);

    case WCAM_VIDIOCGEXPOPARA:
         return mxc_cam_WCAM_VIDIOCGEXPOPARA(cam_ctx, param);

    case WCAM_VIDIOCSMIRROR:
         return mxc_cam_WCAM_VIDIOCSMIRROR(cam_ctx, param);

    case WCAM_VIDIOCGI2CREG:
         return mxc_camera_WCAM_VIDIOCGI2CREG(cam_ctx, param);

    case WCAM_VIDIOCSI2CREG:
          return mxc_camera_WCAM_VIDIOCSI2CREG(cam_ctx, param);
        
    case WCAM_VIDIOCGSNAME:
         return mxc_camera_WCAM_VIDIOCGSNAME(cam_ctx, param);

    case WCAM_VIDIOCGMINMAX:
         return mxc_camera_WCAM_VIDIOCGMINMAX(cam_ctx, param);

    default:
         {
           err_print("Error cmd=%d", cmd);
           return -1;
         }
    }
    return 0;
 
}
