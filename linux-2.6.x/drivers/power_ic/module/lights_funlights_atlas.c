/*
 * Copyright (C) 2005-2007 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Motorola 2007-Mar-30 - Support BT CIT test.
 * Motorola 2007-Feb-23 - Finalize Bluetooth LED behavior.
 * Motorola 2007-Feb-09 - Control Bluetooth LED blinking
 * Motorola 2006-Dec-19 - Add more steps in main backlight
 * Motorola 2006-Oct-10 - Update File
 * Motorola 2006-Sep-07 - Add supoort for Saipan keypad
 * Motorola 2006-Sep-01 - Correct back light brightness in Lido
 * Motorola 2006-Jul-31 - Update comments
 * Motorola 2006-Jul-26 - Support ambient light sensor
 * Motorola 2006-Jul-14 - Add Lido & Saipan Support
 * Motorola 2006-Jun-28 - Fix camera flash issue
 * Motorola 2006-May-22 - Add LED enable control
 * Motorola 2006-May-15 - Fix typecasting warning.
 * Motorola 2006-Apr-12 - Added new interface functions to control backlight brightness.
 * Motorola 2006-Apr-07 - Add new region for EL NAV Keypad
 * Motorola 2006-Apr-07 - Change camera torch GPIO for Ascension P2
 * Motorola 2006-Apr-04 - Move all GPIO functionality to gpio.c
 * Motorola 2006-Jan-12 - Add in Main Display, EL Keypad, and Camera functionality 
 *                        for Ascension
 * Motorola 2005-Nov-15 - Rewrote the software.
 * Motorola 2005-Jun-16 - Updated the region table for new hardware.
 */

/*!
 * @file lights_funlights_atlas.c
 *
 * @ingroup poweric_lights
 *
 * @brief Funlight module
 *
 *  In this file, there are interface functions between the funlights driver and outside world.
 *  These functions implement a cached priority based method of controlling fun lights regions.
 *  Upon powerup the regions are assigned to the default app, which can be looked at as the old functionality.
 *  This allows the keypad and display backlights to operate as they do today, when they are not in use by fun
 *  lights or KJAVA.  If a new application needs to be added it must be done in the corresponding
 *  header file.

 */
#include <stdbool.h>
#include <linux/kernel.h>
#include <linux/power_ic.h>
#include <linux/power_ic_kernel.h>
#include <linux/lights_funlights.h>
#include <linux/lights_backlight.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#include "emu.h"
#include "../core/gpio.h"
#include "../core/os_independent.h"

/*******************************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************************/
static bool lights_fl_region_gpio(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nStep);
static bool lights_fl_region_cli_display(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nStep);
static bool lights_fl_region_tri_color(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_sol_led(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nColor); 
static bool lights_fl_region_main_display(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nStep);
#if defined(CONFIG_MACH_LIDO)
static bool lights_fl_region_main_cli_display(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nStep);
#endif
static bool lights_fl_region_keypad(const LIGHTS_FL_LED_CTL_T *pCtlData, LIGHTS_FL_COLOR_T nStep);

/*******************************************************************************************
 * LOCAL CONSTANTS
 ******************************************************************************************/
/*!@cond INTERNAL */
/*!
 * @name Number of Backlight Steps
 *
 * @brief Define the number of backlight steps which are supported for the different
 * backlights.  This value includes off, so the minimum value is 2, 1 for on 1 for off.
 */
/*@{*/
# define LIGHTS_NUM_KEYPAD_STEPS      2
# define LIGHTS_NUM_DISPLAY_STEPS     8
# define LIGHTS_NUM_CLI_DISPLAY_STEPS 2
# define LIGHTS_NUM_NAV_KEYPAD_STEPS  2
/*@}*/
/*!@endcond */

/* ATLAS Registers */

/*!
 * @name Register LED Control 0
 */
/*@{*/
#define LIGHTS_FL_CHRG_LED_EN                18
#define LIGHTS_FL_LEDEN                      0
/*@}*/

/*!
 * @name Register LED Control 2
 */
/*@{*/
#define LIGHTS_FL_MAIN_DISP_DC_MASK          0x001E00
#define LIGHTS_FL_MAIN_DISP_CUR_MASK         0x000007
#define LIGHTS_FL_CLI_DISP_DC_MASK           0x01E000
#define LIGHTS_FL_CLI_DISP_CUR_MASK          0x000038
#define LIGHTS_FL_KEYPAD_DC_MASK             0x1E01C0
/*@}*/

/*!
 * @name Register LED Control 3
 */
/*@{*/
#define LIGHTS_FL_TRI_COLOR_RED_DC_INDEX     6
#define LIGHTS_FL_TRI_COLOR_RED_DC_MASK      0x0007C0
#define LIGHTS_FL_TRI_COLOR_GREEN_DC_INDEX   11
#define LIGHTS_FL_TRI_COLOR_GREEN_DC_MASK    0x00F800
#define LIGHTS_FL_TRI_COLOR_BLUE_DC_INDEX    16
#define LIGHTS_FL_TRI_COLOR_BLUE_DC_MASK     0x1F0000
#define LIGHTS_FL_TRI_COLOR_BT_DC_MASK       0x020000
/*@}*/

/*!
 * @name Register LED Control 4
 */
#define LIGHTS_FL_TRI_COLOR_DC_MASK          0x1FFFC0

/*!
 * @name Register LED Control 5
 */
/*@{*/
#define LIGHTS_FL_DUTY_CYCLE_SHIFT           3
/*@}*/

/*!
 * @bits GPO1EN and GPO1STBY in register PWR MISC 
 */
/*@{*/
#define LIGHTS_SENSOR_enable_MASK            0x0000C0
/*@}*/

#define LIGHTS_FL_CLI_DISP_CUR_SHFT          3

/* Defines the brightness of main and cli display.*/
const static uint8_t atlas_brightness_tb[LIGHTS_NUM_DISPLAY_STEPS] =
{
    0x00, /* Off   */
    0x01, /* 3 mA  */
    0x02, /* 6 mA  */
    0x03, /* 9 mA  */
    0x04, /* 12 mA */
    0x05, /* 15 mA */
    0x06, /* 18 mA */
    0x07  /* 21 mA */
 };

/*******************************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************************/
/*
 * Table to determine the number of brightness steps supported by hardware.  See
 * lights_backlights.h for more details.
 */
const uint8_t lights_bl_num_steps_tb[LIGHTS_BACKLIGHT_ALL] =
{
    LIGHTS_NUM_KEYPAD_STEPS,      /* LIGHTS_BACKLIGHT_KEYPAD      */
    LIGHTS_NUM_DISPLAY_STEPS,     /* LIGHTS_BACKLIGHT_DISPLAY     */
    LIGHTS_NUM_CLI_DISPLAY_STEPS, /* LIGHTS_BACKLIGHT_CLI_DISPLAY */
    LIGHTS_NUM_NAV_KEYPAD_STEPS   /* LIGHTS_BACKLIGHT_NAV         */
};

/*
 * Table to determine the percentage step size for backlight brightness. See lights_backlights.h
 * for more details.
 */
const uint8_t lights_bl_percentage_steps[LIGHTS_BACKLIGHT_ALL] =
{
    100/(LIGHTS_NUM_KEYPAD_STEPS-1),      /* LIGHTS_BACKLIGHT_KEYPAD      */
    100/(LIGHTS_NUM_DISPLAY_STEPS-1),     /* LIGHTS_BACKLIGHT_DISPLAY     */
    100/(LIGHTS_NUM_CLI_DISPLAY_STEPS-1), /* LIGHTS_BACKLIGHT_CLI_DISPLAY */
    100/(LIGHTS_NUM_NAV_KEYPAD_STEPS-1)   /* LIGHTS_BACKLIGHT_NAV         */
};

/*
 * Table of control functions for region LED's.  See lights_funlights.h for more details.
 */
const LIGHTS_FL_REGION_CFG_T LIGHTS_FL_region_ctl_tb[LIGHTS_FL_MAX_REGIONS] =
{
#if defined(CONFIG_MACH_ASCENSION) 
    {(void*)lights_fl_region_main_display,      {0, NULL}}, /* Display Backlight */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_keypad_base}}, /* Keypad Backlight */
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Tri Color LED #1 */
    {(void*)lights_fl_region_tri_color,         {2, NULL}}, /* Tri Color LED #2 */
    {(void*)lights_fl_region_cli_display,       {0, NULL}}, /* CLI Display Backlight */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_camera_flash}}, /* Camera Flash */
    {(void*)NULL,                               {0, NULL}}, /* WiFi Status LED */
    {(void*)lights_fl_region_sol_led,           {0, NULL}}, /* SOL */ 
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Bluetooth Status LED */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_keypad_slider}}  /* Navigation Keypad Backlight */
#elif defined(CONFIG_MACH_LIDO)
    {(void*)lights_fl_region_main_cli_display,  {0, NULL}}, /* Display Backlight */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_keypad_base}}, /* Keypad Backlight */
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Tri Color LED #1 */
    {(void*)lights_fl_region_tri_color,         {2, NULL}}, /* Tri Color LED #2 */
    {(void*)lights_fl_region_main_cli_display,  {0, NULL}}, /* CLI Display Backlight */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_camera_flash}}, /* Camera Flash */
    {(void*)NULL,                               {0, NULL}}, /* WiFi Status LED */
    {(void*)lights_fl_region_sol_led,           {0, NULL}}, /* SOL */ 
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Bluetooth Status LED */
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_keypad_slider}}  /* Navigation Keypad Backlight */
#elif  defined(CONFIG_MACH_SAIPAN)
    {(void*)lights_fl_region_main_display,      {0, NULL}}, /* Display Backlight */
    {(void*)lights_fl_region_keypad,            {0, NULL}}, /* Keypad Backlight */
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Tri Color LED #1 */
    {(void*)lights_fl_region_tri_color,         {2, NULL}}, /* Tri Color LED #2 */
    {(void*)lights_fl_region_cli_display,       {0, NULL}}, /* CLI Display Backlight */
    {(void*)NULL,                               {0, NULL}}, /* Camera Flash */
    {(void*)NULL,                               {0, NULL}}, /* WiFi Status LED */
    {(void*)lights_fl_region_sol_led,           {0, NULL}}, /* SOL */ 
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Bluetooth Status LED */
    {(void*)NULL,                               {0, NULL}}  /* Navigation Keypad Backlight */
#else
    {(void*)lights_fl_region_gpio,              {0, power_ic_gpio_lights_set_main_display}}, /* Display Backlight */
    {(void*)lights_fl_region_keypad,            {0, NULL}}, /* Keypad Backlight */
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Tri Color LED #1 */
    {(void*)lights_fl_region_tri_color,         {2, NULL}}, /* Tri Color LED #2 */
    {(void*)lights_fl_region_cli_display,       {0, NULL}}, /* CLI Display Backlight */
    {(void*)NULL,                               {0, NULL}}, /* Camera Flash */
    {(void*)NULL,                               {0, NULL}}, /* WiFi Status LED */
    {(void*)lights_fl_region_sol_led,           {0, NULL}}, /* SOL */ 
    {(void*)lights_fl_region_tri_color,         {1, NULL}}, /* Bluetooth Status LED */
    {(void*)lights_fl_region_keypad,            {0, NULL}}  /* Navigation Keypad Backlight */
#endif
};

/*******************************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************************/
/*!
 * @brief Update an LED connected to a gpio
 *
 * This function will turn on or off an LED which is connected to gpio.  The control data
 * must include the gpio function to call.
 *
 * @param  pCtlData     Pointer to the control data for the region. 
 * @param  nColor       The color to set the region to.  In this case 0 is off and non
 *                      0 is on.
 *
 * @return Always returns 0.
 */
static bool lights_fl_region_gpio
(
    const LIGHTS_FL_LED_CTL_T *pCtlData, 
    POWER_IC_UNUSED LIGHTS_FL_COLOR_T nColor
)    
{
    tracemsg(_k_d("=>Update gpio light to color %d"), nColor);

    if (pCtlData->pGeneric != NULL)
    {
        (*(void (*)(LIGHTS_FL_COLOR_T))(pCtlData->pGeneric))(nColor);
    }
    return true;
}

/*!
 * @brief Set main display
 *
 * Function to handle a request for the main display backlight on devices which support variable
 * backlight intensity and have display backlights controlled by either a GPIO or a power ic.
 * The backlight intensity is set in lights_led_backlights.c and converted to a hardware
 * value in this routine.
 *
 * @param   pCtlData   Pointer to the control data for the region.
 * @param   nStep      The brightness step to set the region to.
 *
 * @return     true  region updated
 *             false  region not updated
 */
static bool lights_fl_region_main_display
(
    POWER_IC_UNUSED const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nStep
)
{
    int error = 0;
  
    tracemsg(_k_d("=>Update the main display with step %d"), nStep);
    
    if (nStep >= LIGHTS_NUM_DISPLAY_STEPS)
    {
        nStep = LIGHTS_NUM_DISPLAY_STEPS-1;
    }

    if(nStep)
    {

        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_CLI_DISP_DC_MASK | LIGHTS_FL_CLI_DISP_CUR_MASK,
                                      0);    
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                       LIGHTS_FL_MAIN_DISP_DC_MASK | LIGHTS_FL_MAIN_DISP_CUR_MASK,
                                       LIGHTS_FL_MAIN_DISP_DC_MASK | atlas_brightness_tb[nStep]);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_MAIN_DISP_DC_MASK | LIGHTS_FL_MAIN_DISP_CUR_MASK,
                                      0);

    } 
    return (error != 0);
}

/*!
 * @brief Set CLI display
 *
 * Function to handle a request for the cli display backlight on devices which support variable
 * backlight intensity and have CLI display backlights controlled by either a GPIO port or a power
 * IC.
 *
 * @param  pCtlData     Pointer to the control data for the region. 
 * @param  nStep        The brightness step to set the region to. 
 *
 * @return     true  region updated
 *             false region not updated
 */
static bool lights_fl_region_cli_display
(
    const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nStep
)   
{   
    int error = 0;
    
    tracemsg(_k_d("=>Update the cli display with step %d"), nStep);
    if (nStep >= LIGHTS_NUM_CLI_DISPLAY_STEPS)
    {
        nStep = LIGHTS_NUM_CLI_DISPLAY_STEPS-1;
    }
    if(nStep)
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_MAIN_DISP_DC_MASK|LIGHTS_FL_MAIN_DISP_CUR_MASK,
                                      0);
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                       LIGHTS_FL_CLI_DISP_DC_MASK | LIGHTS_FL_CLI_DISP_CUR_MASK,
                                       LIGHTS_FL_CLI_DISP_DC_MASK | LIGHTS_FL_CLI_DISP_CUR_MASK);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_CLI_DISP_DC_MASK | LIGHTS_FL_CLI_DISP_CUR_MASK,
                                      0);
    }
    return (error != 0);
}

/*!
 * @brief Set main display and cli display
 *
 * Function to handle a request for main display or CLI display backlight on devices which support
 * variable backlight intensity and have main display and CLI display backlights controlled by either
 * a GPIO or a power ic.
 *
 * @param   pCtlData   Pointer to the control data for the region.
 * @param   nStep      The brightness step to set the region to.
 *
 * @return     true  region updated
 *             false  region not updated
 */
static bool lights_fl_region_main_cli_display
(
    POWER_IC_UNUSED const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nStep
)
{
    int error = 0;
    
   
    tracemsg(_k_d("=>Update the main and cli display with step %d"), nStep);
    
    if (nStep >= LIGHTS_NUM_DISPLAY_STEPS)
    {
        nStep = LIGHTS_NUM_DISPLAY_STEPS-1;
    }

    if(nStep)
    {
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                       LIGHTS_FL_MAIN_DISP_DC_MASK |
                                       LIGHTS_FL_CLI_DISP_DC_MASK |
                                       LIGHTS_FL_MAIN_DISP_CUR_MASK|
                                       LIGHTS_FL_CLI_DISP_CUR_MASK,
                                       LIGHTS_FL_MAIN_DISP_DC_MASK |
                                       LIGHTS_FL_CLI_DISP_DC_MASK |
                                       atlas_brightness_tb[nStep]|
                                       (atlas_brightness_tb[nStep] << LIGHTS_FL_CLI_DISP_CUR_SHFT));
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_MAIN_DISP_DC_MASK |
                                      LIGHTS_FL_MAIN_DISP_CUR_MASK |
                                      LIGHTS_FL_CLI_DISP_DC_MASK |
                                      LIGHTS_FL_CLI_DISP_CUR_MASK,
                                      0);

    } 
    return (error != 0);
}

/*!
 * @brief Update keypad backlight
 *
 * Turn on or off the keypad backlight when it is connected to the power IC.
 *
 * @param  pCtlData    Pointer to the control data for the region. 
 * @param  nStep       The brightness step to set the region to. 
 *
 * @return     true  region updated
 *             false region not updated
 */
static bool lights_fl_region_keypad
(
    const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nStep
)    
{
    int error = 0;
    
    tracemsg(_k_d("=>Update the keypad backlight with step %d"), nStep);
    if (nStep >= LIGHTS_NUM_KEYPAD_STEPS)
    {
        nStep = LIGHTS_NUM_KEYPAD_STEPS-1;
    }

    if(nStep)
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_KEYPAD_DC_MASK,
                                      LIGHTS_FL_KEYPAD_DC_MASK);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_KEYPAD_DC_MASK,
                                      0);
    }
    return (error != 0);
}

/*!
 * @brief Update a tri colored LED
 *
 * Function to handle the enabling tri color leds which have the regions combined.
 *
 * @param  pCtlData     Pointer to the control data for the region. 
 * @param  nColor       The color to set the region to. 
 *
 * @return     true  region updated
 *             false region not updated
 */
static bool lights_fl_region_tri_color 
(
    const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)
{
    unsigned int color_mask;
    int error;
    
    tracemsg(_k_d("=>Update the tricolor display %d"), nColor);

    color_mask = (((nColor & LIGHTS_FL_COLOR_BLUE) >> LIGHTS_FL_COLOR_BLUE_SFT)
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT)  << LIGHTS_FL_TRI_COLOR_BLUE_DC_INDEX;
    color_mask |= (((nColor & LIGHTS_FL_COLOR_GREEN) >> LIGHTS_FL_COLOR_GREEN_SFT) 
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT) << LIGHTS_FL_TRI_COLOR_GREEN_DC_INDEX;
    color_mask |= (((nColor & LIGHTS_FL_COLOR_RED) >> LIGHTS_FL_COLOR_RED_SFT) 
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT) << LIGHTS_FL_TRI_COLOR_RED_DC_INDEX;
                                    
    if(pCtlData->nData == 1)
    {
        if (LIGHTS_FL_calling_app == LIGHTS_FL_APP_CTL_TST_CMDS)
        {
            error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_3, LIGHTS_FL_TRI_COLOR_BLUE_DC_MASK, color_mask);
        }
        else
        {
            error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_3, LIGHTS_FL_TRI_COLOR_BLUE_DC_MASK, (LIGHTS_FL_TRI_COLOR_BT_DC_MASK & color_mask));
        }
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_4, LIGHTS_FL_TRI_COLOR_DC_MASK, color_mask);
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_5, LIGHTS_FL_TRI_COLOR_DC_MASK, color_mask);
    }
 
    return (error != 0);
}

/*!
 * @brief Enable or disable the sol LED.
 *
 * Enables or disables the sol LED based on the nColor parameter.  When nColor is 0
 * the led is turned off when it is non 0 it is turned on.
 *
 * @param  pCtlData     Pointer to the control data for the region. 
 * @param  nColor       The color to set the region to.
 *
 * @return true  When the region is update.
 *         false When the region was not updated due to a communication or hardware error.
 */
static bool lights_fl_region_sol_led
(
    const LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)
{
    int error;
    tracemsg(_k_d("=>Update the sol led with color %d"), nColor);
    
    error = power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGER_0,LIGHTS_FL_CHRG_LED_EN,(nColor != 0));
        
    return (error != 0);
}

/*******************************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************************/
/*!
 * @brief Enable or disable the LED EN bit.
 *
 *  The LEDEN bit in Atlas is the master control bit for all the LED's. 
 *  leden will be set to FALSE if none of the LED's are enabled. Otherwise, 
 *  leden will be set to TRUE if one of more of the LED's are enabled.
 * @param  none
 *
 * @return none
 */
void lights_funlights_enable_led(void)
{
    unsigned int reg_value2;
    unsigned int reg_value3;
    unsigned int reg_value4;
    unsigned int reg_value5;
    int value;
    bool leden;
    /* Read out LED CONTROL 2 ,3,4,5 register value. */
    if((power_ic_read_reg(POWER_IC_REG_ATLAS_LED_CONTROL_2,&reg_value2))||
       (power_ic_read_reg(POWER_IC_REG_ATLAS_LED_CONTROL_3,&reg_value3))||
       (power_ic_read_reg(POWER_IC_REG_ATLAS_LED_CONTROL_4,&reg_value4))||
       (power_ic_read_reg(POWER_IC_REG_ATLAS_LED_CONTROL_5,&reg_value5)))
    {
        return;
    }
    leden = ((reg_value2 & (LIGHTS_FL_MAIN_DISP_DC_MASK |
                            LIGHTS_FL_MAIN_DISP_CUR_MASK |
                            LIGHTS_FL_CLI_DISP_DC_MASK |
                            LIGHTS_FL_KEYPAD_DC_MASK)) ||
             (reg_value3 & LIGHTS_FL_TRI_COLOR_DC_MASK)||
             (reg_value4 & LIGHTS_FL_TRI_COLOR_DC_MASK)||
             (reg_value5 & LIGHTS_FL_TRI_COLOR_DC_MASK));

    if(power_ic_get_reg_value(POWER_IC_REG_ATLAS_LED_CONTROL_0,LIGHTS_FL_LEDEN,&value,1))
    {
        return;
    }
    tracemsg(_k_d("The funlight register0 value is: %d"), value);
    /* If the state of the LEDs/Funlights is not equal to the state of the LEDEN bit
       in Atlas, then set the LEDEN bit to the state of the LEDs/Funlights */
    if(leden != value)
    {
        tracemsg(_k_d("=>Update the leden bit,leden= %d,"),leden);
        power_ic_set_reg_bit(POWER_IC_REG_ATLAS_LED_CONTROL_0,LIGHTS_FL_LEDEN, leden ? 1 : 0);
    }
   
}

/*!
 * @brief Enable or disable the ambinet light sensor.
 *
 * The GPO1EN and GPO1STBY bits in register PWR MISC control the ambient light sensor
 * circuit. Both bits set enables the light sensor. Both bits cleared disables the sensor.
 *
 * @param  0 is disable the ambient light circuit
 *         Non 0 is enable the ambient light circuit
 *
 * @return 0 for normal case
 */
int lights_funlights_enable_light_sensor(int light_enable)
{
    tracemsg(_k_d("Start to enable the light sensor circuit"));
    return(power_ic_set_reg_mask(POWER_IC_REG_ATLAS_PWR_MISC,
                                  LIGHTS_SENSOR_enable_MASK,
                                 (light_enable ? LIGHTS_SENSOR_enable_MASK : 0)));
           
}

/*!
 * @brief Read ambient light sensor Atod value
 *
 * This function will read and return the light sensor atod value
 *
 * @return light sensor atod value
 * @note The circuit must be enabled by calling lights_funlights_enable_light_sensor
 *       before the value can be read.
 */       
int lights_funlights_light_sensor_atod(void)
{
    int light_sensor;
   
    if(power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_AMBIENT_LIGHT,&light_sensor))
    {
        light_sensor = 0;
    }
    tracemsg(_k_d("light sensor atod reading value is %d"), light_sensor );
    return light_sensor;
}
