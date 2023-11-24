/*
 * GPIO and IOMUX Configuration for ArgonLV-based Bute
 * 
 * linux/arch/arm/mach-mxc91321/argonlvref_gpio.c
 *
 * Copyright 2006 Motorola, Inc.
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Date         Author          Comment
 * ===========  ==============  ==============================================
 * 04-Oct-2006  Motorola        Initial revision.
 * 06-Dec-2006  Motorola        Added etm_enable_trigger_clock.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/delay.h>
#include <asm/arch/gpio.h>
#include <asm/mot-gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <linux/errno.h>

#ifdef CONFIG_MOT_FEAT_BRDREV
#include <asm/boardrev.h>
#endif /* CONFIG_MOT_FEAT_BRDREV */

#include "iomux.h"

/*!
 * @file argonlvref_gpio.c
 * 
 * @brief This file contains all the GPIO setup functions for the board. 
 * 
 * @ingroup GPIO
 */


/*
 * Local functions.
 */
#ifdef CONFIG_MOT_FEAT_BRDREV
static void gpio_setting_fixup_p3_brassboard(void);
static void gpio_setting_fixup_p4(void);
#endif /* CONFIG_MOT_FEAT_BRDREV */


/**
 * Initial GPIO register settings.
 */
struct gpio_signal_settings initial_gpio_settings[MAX_GPIO_SIGNAL] = {
    /*
     * MCU GPIO Port Pin 1 -- Secondary Display Reset (active low)
     * ButeP3 (brass) Signal: GPIO_CLI_RST_B
     * ButeP4 (close) Signal: GPIO_CLI_RST_B
     * ButeP4 (wing)  Signal: GPIO_CLI_RST_B/GPS_RESET_B
     * ButeP5 (close) Signal: BT_RX
     * Pin: USB1_XRXD Mux Setting: GPIO
     *
     * Array index:  0  GPIO_SIGNAL_CLI_RST_B
     */
    { GPIO_INVALID_PORT,     1, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 8 -- Ethernet Interrupt
     * ButeP3 (brass) Signal: ENET_INT_B
     * ButeP4 (close) Signal: GPIO_CAM_Flash_T_F
     * ButeP4 (wing)  Signal: GPIO_CAM_Flash_T_F
     * ButeP5 (close) Signal: GPU_VCORE2_EN/PWGT2EN
     * Pin: GPIO8 Mux Setting: Func
     *
     * GPIO_MCU_8 also on pin IPU_CSI_DATA_6 mux setting GPIO and
     * pin GPIO33 mux setting Mux2.
     *
     * Array index:  1  GPIO_SIGNAL_ENET_INT_B
     */
    { GPIO_INVALID_PORT,     8, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 14 -- IRDA Shutdown
     * ButeP3 (brass) Signal: IRDA_SD
     * ButeP4 (close) Signal: LCD_SD
     * ButeP4 (wing)  Signal: LCD_SD
     * ButeP5 (close) Signal: LCD_SD
     * Pin: GPIO14 Mux Setting: Func
     *
     * GPIO_MCU_14 also on pin GPIO19 mux setting GPIO.
     *
     * Array index:  2  GPIO_SIGNAL_IRDA_SD
     */
    { GPIO_INVALID_PORT,    14, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 15 -- 2MP Imager Power Down (set high to disable)
     * ButeP3 (brass) Signal: CSI_CS0
     * ButeP4 (close) Signal: GPIO_CAM_EXT_PWRDN
     * ButeP4 (wing)  Signal: GPIO_CAM_EXT_PWRDN
     * ButeP5 (close) Signal: GPIO_CAM_EXT_PWRDN
     * Pin: GPIO15 Mux Setting: Func
     *
     * GPIO_MCU_15 also on pin GPIO22 mux setting GPIO.
     *
     * Array index:  3  GPIO_SIGNAL_CAM_EXT_PWRDN
     */
    { GPIO_MCU_PORT,        15, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * MCU GPIO Port Pin 16 -- 1.3MP Imager Power Down (set high to disable)
     * ButeP3 (brass) Signal: CSI_CS1
     * ButeP4 (close) Signal: GPIO_CAM_INT_PWRDN
     * ButeP4 (wing)  Signal: GPIO_CAM_INT_PWRDN
     * ButeP5 (close) Signal: GPIO_CAM_INT_PWRDN
     * Pin: GPIO16 Mux Setting: Func
     *
     * Array index:  4  GPIO_SIGNAL_CAM_INT_PWRDN
     */
    { GPIO_MCU_PORT,        16, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },


    /*
     * MCU GPIO Port Pin 17 -- SDHC Port 1 Card Detect
     * ButeP3 (brass) Signal: SD1_DET_B
     * ButeP4 (close) Signal: SD1_DET_B
     * ButeP4 (wing)  Signal: SD1_DET_B
     * ButeP5 (close) Signal: SD1_DET_B
     * Pin: GPIO17 Mux Setting: Func
     *
     * GPIO_MCU_17 also on pin GPIO22 mux setting GPIO.
     *
     * Array index:  5  GPIO_SIGNAL_SD1_DET_B
     */
    { GPIO_MCU_PORT,        17, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 24 -- Flip Detect
     * ButeP3 (brass) Signal: FLIP_DETECT
     * ButeP4 (close) Signal: FLIP_DETECT
     * ButeP4 (wing)  Signal: FLIP_DETECT
     * ButeP5 (close) Signal: FLIP_DETECT
     * Pin: GPIO37 Mux Setting: Func
     *
     * GPIO_MCU_24 also on pin SIM1_RST0 mux setting GPIO.
     *
     * Array index:  6  GPIO_SIGNAL_FLIP_DETECT
     */
    { GPIO_MCU_PORT,        24, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 28 -- Main Display Reset
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     *
     * Array index:  7  GPIO_SIGNAL_DISP_RST_B
     *
     * This line must toggle from high to low to enable the display.
     */
    { GPIO_INVALID_PORT,    28, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 29 -- Main Display Color Mode (low=high color mode)
     * ButeP3 (brass) Signal: GPIO_DISP_CM
     * ButeP4 (close) Signal: STBY_B
     * ButeP4 (wing)  Signal: STBY_B
     * ButeP5 (close) Signal: STBY_B/GPU_INT_B
     * Pin: USB1_TXENB Mux Setting: GPIO
     *
     * Array index:  8  GPIO_SIGNAL_DISP_CM
     */
    { GPIO_INVALID_PORT,    29, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 30 -- Main Display Backlight Enable
     * ButeP3 (brass) Signal: IPU_MAIN_BKLT_EN_B
     * ButeP4 (close) Signal: LCD_Backlight
     * ButeP4 (wing)  Signal: LCD_Backlight
     * ButeP5 (brass) Signal: LCD_Backlight/GPU_ADDR_LATCH
     * Pin: USB1_VPIN Mux Setting: GPIO
     *
     * Array index:  9  GPIO_SIGNAL_LCD_BACKLIGHT
     */
    { GPIO_INVALID_PORT,    30, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * Shared GPIO Port Pin 1 -- Bluetooth Host Wake
     * ButeP3 (brass) Signal: BT_HOST_WAKE_B
     * ButeP4 (close) Signal: BT_HOST_WAKE_B
     * ButeP4 (wing)  Signal: BT_HOST_WAKE_B
     * ButeP5 (close) Signal: BT_HOST_WAKE_B
     * Pin: GPIO21 Mux Setting: Func
     *
     * GPIO_Shared_1 also on pin IPU_CSI_MCLK mux setting GPIO.
     *
     * Array index: 10  GPIO_SIGNAL_BT_HOST_WAKE_B
     */
    { GPIO_SHARED_PORT,      1, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * Shared GPIO Port Pin 3 -- Camera Reset
     * ButeP3 (brass) Signal: GPIO_CAM_RST_B
     * ButeP4 (close) Signal: GPIO_CAM_RST_B
     * ButeP4 (wing)  Signal: GPIO_CAM_RST_B
     * ButeP5 (close) Signal: GPIO_CAM_RST_B
     * Pin: GPIO23 Mux Setting: Func
     *
     * GPIO_Shared_3 also on pin IPU_CSI_VSYNC mux setting GPIO.
     *
     * Array index: 11  GPIO_SIGNAL_CAM_RST_B
     */
    { GPIO_SHARED_PORT,      3, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * Shared GPIO Port Pin 6 -- Bluetooth Controller Wake
     * ButeP3 (brass) Signal: BT_WAKE_B
     * ButeP4 (close) Signal: BT_WAKE_B
     * ButeP4 (wing)  Signal: BT_WAKE_B
     * ButeP5 (close) Signal: BT_WAKE_B
     * Pin: GPIO26 Mux Setting: Func
     *
     * GPIO_Shared_6 also on pin IPU_CSI_DATA_2 mux setting GPIO.
     *
     * Array index: 12  GPIO_SIGNAL_BT_WAKE_B
     */
    { GPIO_SHARED_PORT,      6, GPIO_GDIR_OUTPUT, GPIO_DATA_LOW },

    /*
     * MCU GPIO Port Pin 13 -- Bluetooth Reset
     * ButeP3 (brass) Signal: WLAN_CLIENT_WAKE_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: BT_RESET_B
     * Pin: GPIO13 Mux Setting: Func
     *
     * GPIO_MCU_13 also on pin GPIO18 mux setting GPIO.
     *
     * Array index: 13  GPIO_SIGNAL_BT_POWER
     */
    { GPIO_MCU_PORT,        13, GPIO_GDIR_OUTPUT, GPIO_DATA_LOW },

    /*
     * MCU GPIO Port Pin 14 -- Main Display Shut Down
     * ButeP3 (brass) Signal: IRDA_SD
     * ButeP4 (close) Signal: LCD_SD
     * ButeP4 (wing)  Signal: LCD_SD
     * ButeP5 (close) Signal: LCD_SD
     * Pin: GPIO14 Mux Setting: Func
     *
     * GPIO_MCU_14 also on pin GPIO19 mux setting GPIO.
     *
     * Array index: 14  GPIO_SIGNAL_LCD_SD
     */
    { GPIO_MCU_PORT,        14, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 28 -- Main Display Serializer/Deserializer Enable
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B 
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     *
     * Array index: 15  GPIO_SIGNAL_SERDES_RESET_B
     *
     * Set high to enable serializer at boot.
     */
    { GPIO_INVALID_PORT,    28, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 29 -- Serializer Standby (in low-power mode when low)
     * ButeP3 (brass) Signal: GPIO_DISP_CM
     * ButeP4 (close) Signal: STBY_B
     * ButeP4 (wing)  Signal: STBY_B
     * ButeP5 (close) Signal: STBY_B/GPU_INT_B
     * Pin: USB1_TXENB Mux Setting: GPIO
     *
     * Array index: 16  GPIO_SIGNAL_STBY
     *
     * Set low to put serializer into standby mode at boot.
     */
    { GPIO_INVALID_PORT,    29, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 23 -- GPU Deep-Power-Down
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_DPD_B
     * Pin: GPIO36 Mux Setting: Func
     *
     * GPIO_MCU_23 is also on pin SIM1_CLK0 mux setting GPIO.
     *
     * Array index: 17  GPIO_SIGNAL_GPU_DPD_B
     */
    { GPIO_MCU_PORT,            23, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 28 -- GPU Reset
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B 
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     *
     * Array index: 18  GPIO_SIGNAL_GPU_RESET_B
     */
    { GPIO_MCU_PORT,            28, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 31 -- APPS_CLK_EN_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: APPS_CLK_EN_B
     * Pin: USB1_VPOUT Mux Setting: GPIO
     *
     * Array index: 19  GPIO_SIGNAL_APP_CLK_EN_B
     */
    { GPIO_MCU_PORT,            31, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 12 -- GPS Reset
     * ButeP4 (close) Signal: KPD_EL_EN2
     * ButeP4 (wing)  Signal: KPD_EL_EN2
     * ButeP5 (close) Signal: GPS_RESET_B
     * Pin: GPIO12 Mux Setting: Func
     *
     * GPIO_MCU_12 is also available pin SDMA_EVNT1 mux setting Alt2 and
     * pin IPU_CSI_PIXCLK mux setting GPIO.
     *
     * Array index: 20  GPIO_SIGNAL_GPS_RESET
     */
    { GPIO_MCU_PORT,            12, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * MCU GPIO Port Pin 7 -- SW_GPU_CORE1
     * ButeP3 (brass) Signal: GPS_RESET_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_VCORE1_EN/PWGT1EN
     * Pin: GPIO7 Mux Setting: Func
     *
     * Array index: 21  GPIO_SIGNAL_GPU_VCORE1_EN
     */
    { GPIO_MCU_PORT,             7, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * MCU GPIO Port Pin 8 -- SW_GPU_CORE2
     * ButeP3 (brass) Signal: ENET_INT_B
     * ButeP4 (close) Signal: GPIO_CAM_Flash_T_F
     * ButeP4 (wing)  Signal: GPIO_CAM_Flash_T_F
     * ButeP5 (close) Signal: GPU_VCORE2_EN/PWGT2EN
     * Pin: GPIO8 Mux Setting: Func
     *
     * Array index: 22  GPIO_SIGNAL_GPU_VCORE2_EN
     */
    { GPIO_MCU_PORT,             8, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },
};



/*
 * PWM Registers for backlight brightness control.
 */
#define PWMCR               IO_ADDRESS(PWM_BASE_ADDR + 0x00)
#define PWMSAR              IO_ADDRESS(PWM_BASE_ADDR + 0x0c)
#define PWMPR               IO_ADDRESS(PWM_BASE_ADDR + 0x10)

#define DUTY_CYCLE_0   0x00000000
#define DUTY_CYCLE_50  0x00000007
#define DUTY_CYCLE_100 0x0000000F

/*
 * Function Prototypes
 */
void gpio_lcd_backlight_enable(bool enable);
int  gpio_get_lcd_backlight(void);
void pwm_set_lcd_bkl_brightness(int value);
int  pwm_get_lcd_bkl_brightness(void);

void gpio_dai_enable(void);
void gpio_dai_disable(void);

/**
 * Initial IOMUX settings.
 */
struct iomux_initialization {
    enum iomux_pins             pin;
    enum iomux_output_config    out;
    enum iomux_input_config     in;
};


/**
 * CSI Camera Sensor Interface IOMUX settings
 */
struct iomux_initialization csi_iomux_settings[] __initdata = {
    /*
     * Pin Name: IPU_CSI_DATA_0
     * Bute3A Signal Name: IPU_CSI_DATA[0]
     * Bute4A Signal Name: CSI_D[0]
     * Functional Mode: ipp_ind_sensb_data[6] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_1
     * Bute3A Signal Name: IPU_CSI_DATA[1]
     * Bute4A Signal Name: CSI_D[1]
     * Functional Mode: ipp_ind_sensb_data[7] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_2
     * Bute3A Signal Name: IPU_CSI_DATA[2]
     * Bute4A Signal Name: CSI_D[2]
     * Functional Mode: ipp_ind_sensb_data[8] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_3
     * Bute3A Signal Name: IPU_CSI_DATA[3]
     * Bute4A Signal Name: CSI_D[3]
     * Functional Mode: ipp_ind_sensb_data[9] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_4
     * Bute3A Signal Name: IPU_CSI_DATA[4]
     * Bute4A Signal Name: CSI_D[4]
     * Functional Mode: ipp_ind_sensb_data[10] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_5
     * Bute3A Signal Name: IPU_CSI_DATA[5]
     * Bute4A Signal Name: CSI_D[5]
     * Functional Mode: ipp_ind_sensb_data[11] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_6
     * Bute3A Signal Name: IPU_CSI_DATA[6]
     * Bute4A Signal Name: CSI_D[6]
     * Functional Mode: ipp_ind_sensb_data[12] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_7
     * Bute3A Signal Name: IPU_CSI_DATA[7]
     * Bute4A Signal Name: CSI_D[7]
     * Functional Mode: ipp_ind_sensb_data[13] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_8
     * Bute3A Signal Name: IPU_CSI_DATA[8]
     * Bute4A Signal Name: CSI_D[8]
     * Functional Mode: ipp_ind_sensb_data[14] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_DATA_9
     * Bute3A Signal Name: IPU_CSI_DATA[9]
     * Bute4A Signal Name: CSI_D[9]
     * Functional Mode: ipp_ind_sensb_data[15] (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_DATA_9, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_MCLK
     * Bute3A Signal Name: IPU_CSI_MCLK
     * Bute4A Signal Name: CSI_MCLK
     * Functional Mode: ipp_do_sensb_mstr_clk (output)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_MCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_VSYNC
     * Bute3A Signal Name: IPU_CSI_VSYNC
     * Bute4A Signal Name: CSI_VSYNC
     * Functional Mode: ipp_ind_sensb_vsync (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_VSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_HSYNC
     * Bute3A Signal Name: IPU_CSI_HSYNC
     * Bute4A Signal Name: CSI_HSYNC
     * Functional Mode: ipp_ind_sensb_hsync (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_HSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: IPU_CSI_PIXCLK
     * Bute3A Signal Name: IPU_CSI_PIXCLK
     * Bute4A Signal Name: CSI_PIXCLK
     * Functional Mode: ipp_ind_sensb_pix_clk (input)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_IPU_CSI_PIXCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /* List Terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC }
};

/**
 *  IPU Image Processing Unit IOMUX settings
 */
struct iomux_initialization ipu_iomux_settings[] __initdata = {
    /* List Terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC }
};

/**
 *  GPIO General Purpose Input/Output IOMUX settings
 */
struct iomux_initialization gpio_iomux_settings[] __initdata = {
    /*
     * Pin Name: GPIO7
     * ButeP3 (brass) Signal: GPS_RESET_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_VCORE1_EN/PWGT1EN
     * Functional Mode: GPIO MCU Port Pin 7 (Output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO12
     * ButeP4 (close) Signal: KPD_EL_EN2
     * ButeP4 (wing)  Signal: KPD_EL_EN2
     * ButeP5 (close) Signal: GPS_RESET_B
     * Functional Mode: GPIO MCU Port Pin 12 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO12, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO36
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_DPD_B
     * Functional Mode: GPIO MCU Port Pin 23 (output) (_NOT_ present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO36, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: USB1_VPOUT
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: APPS_CLK_EN_B
     * GPIO Mode: MCU Port Pin 31 (output) (_NOT_ present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_USB1_VPOUT, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO },
    /*
     * Pin Name: USB1_VMOUT
     * Bute3A Signal Name: GPU_RESET_B / GPIO_DISP_RST_B
     * Bute4A Signal Name: SERDES_RESET_B
     * GPIO Mode: MCU Port Pin 28 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_USB1_VMOUT, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO },
    /*
     * Pin Name: USB1_TXENB
     * Bute3A Signal Name: GPU_INT_B / GPIO_DISP_CM
     * Bute4A Signal Name: STBY
     * GPIO Mode: MCU Port Pin 29 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_USB1_TXENB, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO },
    /*
     * Pin Name: USB1_VPIN
     * Bute3A Signal Name: IPU_MAIN_BKLT_EN_B / GPU_A2
     * Bute4A Signal Name: LCD_Backlight
     * GPIO Mode: MCU Port Pin 30 (output) (not present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_USB1_VPIN, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO },
    /*
     * Pin Name: USB1_XRXD
     * ButeP3 (brass) Signal: GPIO_CLI_RST_B
     * ButeP4 (close) Signal: GPIO_CLI_RST_B
     * ButeP4 (wing)  Signal: GPIO_CLI_RST_B/GPS_RESET_B
     * ButeP5 (close) Signal: BT_RX
     * GPIO Mode: MCU Port Pin 1 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_USB1_XRXD, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO },
    /*
     * Pin Name: GPIO23
     * Bute3A Signal Name: GPIO_CAM_RST_B
     * Bute4A Signal Name: GPIO_CAM_RST_B
     * Functional Mode: GPIO Shared Port Pin 3 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO23, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO15
     * Bute3A Signal Name: CSI_CS0
     * Bute4A Signal Name: GPIO_CAM_EXT_PWRDN
     * Functional Mode: GPIO MCU Port Pin 15 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO15, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO16
     * Bute3A Signal Name: CSI_CS1
     * Bute4A Signal Name: GPIO_CAM_INT_PWRDN
     * Functional Mode: GPIO MCU Port Pin 16 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO16, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO17
     * Bute3A Signal Name: SD1_DET_B
     * Bute4A Signal Name: SD1_DET_B
     * Functional Mode: GPIO MCU Port Pin 17 (input) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     *
     */
    { PIN_GPIO17, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO14
     * Bute3A Signal: IRDA_SD
     * Bute4A Signal: LCD_SD
     * Functional Mode: GPIO MCU Port Pin 14 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO14, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /* 
     * Pin Name: GPIO8
     * Bute3A Signal: DCM_HS_DET_B/ENET_INT_B
     * Bute4A Signal: GPIO_CAM_Flash_T_F
     * Functional Mode: GPIO MCU Port Pin 8 (input) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO37
     * Bute3A Signal: FLIP_DETECT
     * Bute4A Signal: FLIP_DETECT
     * Functional Mode: GPIO MCU Port Pin 24 (input) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO37, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /* List Terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC }
};

/**
 * EDIO External Interrupt IOMUX settings
 */
struct iomux_initialization edio_iomux_settings[] __initdata = {
    /*
     * Pin Name: PM_INT
     * Bute3A Signal: PM_INT
     * Bute4A Signal: PM_INT
     * Mux2 Mode: ED_INT1 -- ipp_ind_extintr[1]/ipp_obe_extintr[1]
     * Out of Reset Setting: 0x00 (OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE)
     */
    { PIN_PM_INT, OUTPUTCONFIG_ALT2, INPUTCONFIG_ALT2 },
    /* List Terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC }
};

/**
 * Bluetooth IOMUX settings
 */
struct iomux_initialization bluetooth_iomux_settings[] __initdata = {
    /*
     * Pin Name: GPIO21
     * Bute3A Signal: BT_HOST_WAKE_B
     * Bute4A Signal: BT_HOST_WAKE_B
     * Functional Mode: GPIO Shared Port Pin 1 (input) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO21, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO26
     * Bute3A Signal: BT_WAKE_B
     * Bute4A Signal: BT_WAKE_B
     * Functional Mode: GPIO Shared Port Pin 6 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO26, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO27
     * Bute3A Signal: BT_RESET_B
     * Bute4A Signal: BT_RESET_B
     * Functional Mode: GPIO Shared Port Pin 7 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO27, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /*
     * Pin Name: GPIO13
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: BT_RESET_B
     * Functional Mode: GPIO MCU Port Pin 13 (output) (present on LVLT)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    { PIN_GPIO13, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC },
    /* List Terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC }
};

/**
 * Global IOMUX settings.
 */
struct iomux_initialization *initial_iomux_settings[] __initdata = {
    csi_iomux_settings,
    ipu_iomux_settings,
    gpio_iomux_settings,
    edio_iomux_settings,
    bluetooth_iomux_settings,
    /* End of List */
    NULL
};


/*!
 * This system-wise GPIO function initializes the pins during system startup. 
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by
 * fixup_argonlvref() during system startup. This function is board
 * specific. 
 */
void __init argonlvref_gpio_init(void)
{
    unsigned i, j;

#if defined(CONFIG_MOT_FEAT_BRDREV)
    if((boardrev() < BOARDREV_P4A) || (boardrev() == BOARDREV_UNKNOWN)) {
        gpio_setting_fixup_p3_brassboard();
    } else if(boardrev() < BOARDREV_P5A) {
        gpio_setting_fixup_p4();
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /* configure GPIO registers to desired initial state */
    for(i = 0; i < MAX_GPIO_SIGNAL; i++) {
        if(initial_gpio_settings[i].port != GPIO_INVALID_PORT) {
            gpio_tracemsg("GPIO port: 0x%08x signal: 0x%08x output: 0x%08x "
                    "data: 0x%08x",
                    initial_gpio_settings[i].port,
                    initial_gpio_settings[i].sig_no,
                    initial_gpio_settings[i].out,
                    initial_gpio_settings[i].data);

            /* set data */
            if(initial_gpio_settings[i].data != GPIO_DATA_INVALID) {
                gpio_set_data(initial_gpio_settings[i].port,
                        initial_gpio_settings[i].sig_no,
                        initial_gpio_settings[i].data);
            }

            /* set direction */
        gpio_config(initial_gpio_settings[i].port,
                initial_gpio_settings[i].sig_no,
                    initial_gpio_settings[i].out,
                GPIO_INT_NONE); /* setup interrupts later */
        }
    }

    /* configure IOMUX settings to their prescribed initial states */
    for(i = 0; initial_iomux_settings[i] != NULL; i++) {
        for(j = 0; initial_iomux_settings[i][j].pin != IOMUX_INVALID_PIN; j++) {
            gpio_tracemsg("IOMUX pin: 0x%08x output: 0x%02x input: 0x%02x",
                    initial_iomux_settings[i][j].pin,
                    initial_iomux_settings[i][j].output_config,
                    initial_iomux_settings[i][j].input_config);

            iomux_config_mux(initial_iomux_settings[i][j].pin,
                    initial_iomux_settings[i][j].out,
                    initial_iomux_settings[i][j].in);
        }
    }
}


#if defined(CONFIG_MOT_FEAT_BRDREV)
/**
 * Adjust initial_iomux_settings array to reflect P3 brassboard configuration.
 */
void __init gpio_setting_fixup_p3_brassboard(void)
{
    /* first go into P4 state */
    gpio_setting_fixup_p4();

    /*
     * MCU GPIO Port Pin 7 -- GPS Reset
     * ButeP3 (brass) Signal: GPS_RESET_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_VCORE1_EN/PWGT1EN
     * Pin: GPIO7 Mux Setting: Func
     *
     * GPIO_MCU_7 is also on pin USB_XRXD mux setting GPIO.
     */
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].port   = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].sig_no = 7;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].data   = GPIO_DATA_LOW;

    /*
     * MCU GPIO Port Pin 8 -- Ethernet Interrupt
     * ButeP3 (brass) Signal: ENET_INT_B
     * ButeP4 (close) Signal: GPIO_CAM_Flash_T_F
     * ButeP4 (wing)  Signal: GPIO_CAM_Flash_T_F
     * ButeP5 (close) Signal: GPU_VCORE2_EN/PWGT2EN
     * Pin: GPIO8 Mux Setting: Func
     */
    initial_gpio_settings[GPIO_SIGNAL_ENET_INT_B].port      = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_ENET_INT_B].sig_no    = 8;
    initial_gpio_settings[GPIO_SIGNAL_ENET_INT_B].out       = GPIO_GDIR_INPUT;
    initial_gpio_settings[GPIO_SIGNAL_ENET_INT_B].data      = GPIO_DATA_INVALID;

    /*
     * MCU GPIO Port Pin 14 -- IRDA Shutdown
     * ButeP3 (brass) Signal: IRDA_SD
     * ButeP4 (close) Signal: LCD_SD
     * ButeP4 (wing)  Signal: LCD_SD
     * ButeP5 (close) Signal: LCD_SD
     * Pin: GPIO14 Mux Setting: Func
     */
    initial_gpio_settings[GPIO_SIGNAL_IRDA_SD].port     = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_IRDA_SD].sig_no   = 14;
    initial_gpio_settings[GPIO_SIGNAL_IRDA_SD].out      = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_IRDA_SD].data     = GPIO_DATA_HIGH;

    /*
     * MCU GPIO Port Pin 28 -- Main Display Reset
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     *
     * This line must toggle from high to low to enable the display.
     */
    initial_gpio_settings[GPIO_SIGNAL_DISP_RST_B].port      = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_RST_B].sig_no    = 28;
    initial_gpio_settings[GPIO_SIGNAL_DISP_RST_B].out       = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_RST_B].data      = GPIO_DATA_HIGH;

    /*
     * MCU GPIO Port Pin 29 -- Main Display Color Mode (low=high color mode)
     * ButeP3 (brass) Signal: GPIO_DISP_CM
     * ButeP4 (close) Signal: STBY_B
     * ButeP4 (wing)  Signal: STBY_B
     * ButeP5 (close) Signal: STBY_B/GPU_INT_B
     * Pin: USB1_TXENB Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_DISP_CM].port     = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_CM].sig_no   = 29;
    initial_gpio_settings[GPIO_SIGNAL_DISP_CM].out      = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_CM].data     = GPIO_DATA_LOW;

    /*
     * MCU GPIO Port Pin 14 -- Main Display Shut Down
     * ButeP3 (brass) Signal: IRDA_SD
     * ButeP4 (close) Signal: LCD_SD
     * ButeP4 (wing)  Signal: LCD_SD
     * ButeP5 (close) Signal: LCD_SD
     * Pin: GPIO14 Mux Setting: Func
     *
     * GPIO_MCU_14 also on pin GPIO19 mux setting GPIO.
     */
    initial_gpio_settings[GPIO_SIGNAL_LCD_SD].port  = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 28 -- Main Display Serializer/Deserializer Enable
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B 
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_SERDES_RESET_B].port  = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 29 -- Serializer Standby (in low-power mode when low)
     * ButeP3 (brass) Signal: GPIO_DISP_CM
     * ButeP4 (close) Signal: STBY_B
     * ButeP4 (wing)  Signal: STBY_B
     * ButeP5 (close) Signal: STBY_B/GPU_INT_B
     * Pin: USB1_TXENB Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_STBY].port    = GPIO_INVALID_PORT;
}


/**
 * Adjust initial_gpio_settings array to reflect P4 closed-phone and
 * wingboard.
 */
void __init gpio_setting_fixup_p4(void)
{
    /*
     * MCU GPIO Port Pin 1 -- Secondary Display Reset (active low)
     * ButeP3 (brass) Signal: GPIO_CLI_RST_B
     * ButeP4 (close) Signal: GPIO_CLI_RST_B
     * ButeP4 (wing)  Signal: GPIO_CLI_RST_B/GPS_RESET_B
     * ButeP5 (close) Signal: BT_RX
     * Pin: USB1_XRXD Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_CLI_RST_B].port   = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_CLI_RST_B].sig_no = 1;
    initial_gpio_settings[GPIO_SIGNAL_CLI_RST_B].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_CLI_RST_B].data   = GPIO_DATA_LOW;

    /* Pin USB1_XRXD is also used to control GPS_RESET_B on some modified
     * P4A Wingboards. This means there is a conflict between the ability
     * to control the secondary display and the GPS module. */
    if( (boardrev() >= BOARDREV_P4AW) && (boardrev() < BOARDREV_P5A) ) {
        /*
         * MCU GPIO Port Pin 1 -- Secondary Display Reset (active low)
         * ButeP3 (brass) Signal: GPIO_CLI_RST_B
         * ButeP4 (close) Signal: GPIO_CLI_RST_B
         * ButeP4 (wing)  Signal: GPIO_CLI_RST_B/GPS_RESET_B
         * ButeP5 (close) Signal: BT_RX
         * Pin: USB1_XRXD Mux Setting: GPIO
         */
        initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].port   = GPIO_MCU_PORT;
        initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].sig_no = 1;
        initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].out    = GPIO_GDIR_OUTPUT;
        initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].data   = GPIO_DATA_INVALID;
    } else {
        /*
         * MCU GPIO Port Pin 12 -- GPS Reset
         * ButeP4 (close) Signal: KPD_EL_EN2
         * ButeP4 (wing)  Signal: KPD_EL_EN2
         * ButeP5 (close) Signal: GPS_RESET_B
         * Pin: GPIO12 Mux Setting: Func
         */
        initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].port   = GPIO_INVALID_PORT;
    }

    /*
     * MCU GPIO Port Pin 30 -- Main Display Backlight Enable
     * ButeP3 (brass) Signal: IPU_MAIN_BKLT_EN_B
     * ButeP4 (close) Signal: LCD_Backlight
     * ButeP4 (wing)  Signal: LCD_Backlight
     * ButeP5 (brass) Signal: LCD_Backlight/GPU_ADDR_LATCH
     * Pin: USB1_VPIN Mux Setting: GPIO
     *
     * Configured by MBM at boot time.
     */
    initial_gpio_settings[GPIO_SIGNAL_LCD_BACKLIGHT].port   = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_LCD_BACKLIGHT].sig_no = 30;
    initial_gpio_settings[GPIO_SIGNAL_LCD_BACKLIGHT].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_LCD_BACKLIGHT].data   = GPIO_DATA_INVALID;

    /*
     * Shared GPIO Port Pin 7 -- Bluetooth Reset
     * ButeP3 (brass) Signal: BT_RESET_B
     * ButeP4 (close) Signal: BT_RESET_B
     * ButeP4 (wing)  Signal: BT_RESET_B
     * ButeP5 (close) Signal: NC
     * Pin: GPIO27 Mux Setting: Func
     *
     * GPIO_Shared_7 also on pin IPU_CSI_DATA_3 mux setting GPIO.
     */
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].port    = GPIO_SHARED_PORT;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].sig_no  = 7;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].out     = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].data    = GPIO_DATA_LOW;
    
    /*
     * MCU GPIO Port Pin 28 -- Main Display Serializer/Deserializer Enable
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B 
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_SERDES_RESET_B].port   = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_SERDES_RESET_B].sig_no = 28;
    initial_gpio_settings[GPIO_SIGNAL_SERDES_RESET_B].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_SERDES_RESET_B].data   = GPIO_DATA_HIGH;

    /*
     * MCU GPIO Port Pin 29 -- Serializer Standby (in low-power mode when low)
     * ButeP3 (brass) Signal: GPIO_DISP_CM
     * ButeP4 (close) Signal: STBY_B
     * ButeP4 (wing)  Signal: STBY_B
     * ButeP5 (close) Signal: STBY_B/GPU_INT_B
     * Pin: USB1_TXENB Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_STBY].port    = GPIO_MCU_PORT;
    initial_gpio_settings[GPIO_SIGNAL_STBY].sig_no  = 29;
    initial_gpio_settings[GPIO_SIGNAL_STBY].out     = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_STBY].data    = GPIO_DATA_LOW;

    /*
     * MCU GPIO Port Pin 7 -- SW_GPU_CORE1
     * ButeP3 (brass) Signal: GPS_RESET_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_VCORE1_EN/PWGT1EN
     * Pin: GPIO7 Mux Setting: Func
     */
    initial_gpio_settings[GPIO_SIGNAL_GPU_VCORE1_EN].port   = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 8 -- SW_GPU_CORE2
     * ButeP3 (brass) Signal: ENET_INT_B
     * ButeP4 (close) Signal: GPIO_CAM_Flash_T_F
     * ButeP4 (wing)  Signal: GPIO_CAM_Flash_T_F
     * ButeP5 (close) Signal: GPU_VCORE2_EN/PWGT2EN
     * Pin: GPIO8 Mux Setting: Func
     */
    initial_gpio_settings[GPIO_SIGNAL_GPU_VCORE2_EN].port   = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 23 -- GPU Deep-Power-Down
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: GPU_DPD_B
     * Pin: GPIO36 Mux Setting: Func
     */
    initial_gpio_settings[GPIO_SIGNAL_GPU_DPD_B].port   = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 28 -- GPU Reset
     * ButeP3 (brass) Signal: GPIO_DISP_RST_B
     * ButeP4 (close) Signal: SERDES_RESET_B 
     * ButeP4 (wing)  Signal: SERDES_RESET_B
     * ButeP5 (close) Signal: SERDES_RESET_B/GPU_RESET_B
     * Pin: USB1_VMOUT Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_GPU_RESET_B].port = GPIO_INVALID_PORT;

    /*
     * MCU GPIO Port Pin 31 -- APPS_CLK_EN_B
     * ButeP4 (close) Signal: NC
     * ButeP4 (wing)  Signal: NC
     * ButeP5 (close) Signal: APPS_CLK_EN_B
     * Pin: USB1_VPOUT Mux Setting: GPIO
     */
    initial_gpio_settings[GPIO_SIGNAL_APP_CLK_EN_B].port    = GPIO_INVALID_PORT;
}
#endif /* CONFIG_MOT_FEAT_BRDREV */


#if defined(CONFIG_MOT_FEAT_GPIO_API)
/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
    /*
     * Configure the IOMUX control registers for the UART signals 
     */
    switch (port) {
        /* UART 1 IOMUX Configs */
        case 0:
            if (no_irda == 1) {
                /* 
                 * Pin Name: UART_TXD1
                 * Bute3A Signal: BT_TX
                 * Bute4A Signal: BT_TX
                 * Functional Mode: ipp_uart1_txd (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_TXD1, OUTPUTCONFIG_FUNC,
                        INPUTCONFIG_FUNC);

                /*
                 * Pin Name: UART_RXD1
                 * Bute3A Signal: BT_RX
                 * Bute4A Signal: BT_RX
                 * Functional Mode: ipp_uart1_rxd (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_RXD1, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);

                /*
                 * Pin Name: UART_RTS1_B
                 * Bute3A Signal: BT_RTS_B
                 * Bute4A Signal: BT_RTS_B
                 * Functional Mode: ipp_uart1_rts_b (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_RTS1_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);

                /*
                 * Pin name: UART_CTS1_B
                 * Bute3A Signal: BT_CTS_B
                 * Bute4A Signal: BT_CTS_B
                 * Functional Mode: ipp_uart1_cts_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_CTS1_B, OUTPUTCONFIG_FUNC,
                        INPUTCONFIG_FUNC);
            }
            break; /* case 0 */
            
        /* UART 2 IOMUX Configs (UART2 on USB pins) */
        case 1: 
            if (no_irda == 1) {
                /*
                 * Pin name: USB_VMOUT
                 * Bute3A Signal: USB_VMOUT
                 * Bute4A Signal: USB_VMOUT
                 * Mux2 Mode: UART2_TXD, ipp_uart2_txd (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_USB_VMOUT, OUTPUTCONFIG_ALT2,
                        INPUTCONFIG_ALT2);

                /*
                 * Pin name: USB_VPOUT
                 * Bute3A Signal: USB_VPOUT
                 * Bute4A Signal: USB_VPOUT
                 * Mux2 Mode: UART2_RXD, ipp_uart2_rxd (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_USB_VPOUT, OUTPUTCONFIG_ALT2, 
                        INPUTCONFIG_ALT2); 

                /*
                 * Pin name: USB_XRXD
                 * Bute3A Signal: USB_XRXD
                 * Bute4A Signal: USB_XRXD
                 * Mux1 Mode: UART_RTS2_B, ipp_uart2_rts_b (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_USB_XRXD, OUTPUTCONFIG_ALT1, 
                        INPUTCONFIG_ALT1);

                /* 
                 * Pin name: UART_CTS2_B
                 * Bute3A Signal: CTS2_B
                 * Bute4A Signal: CTS2_B
                 * Functional Mode: ipp_uart2_cts_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_CTS2_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);
                
                /* 
                 * Pin name: UART_DSR2_B
                 * Bute3A Signal: DSR2_B
                 * Bute4A Signal: DAI_STDA
                 * Functional Mode: ipp_uart2_dsr_dce_o_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_DSR2_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);
                
                /*
                 * Pin name: UART_DTR2_B
                 * Bute3A Signal: DTR2_B
                 * Bute3A Signal: DAI_SRDA
                 * Functional Mode: ipp_uart2_dtr_dce_i_b (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_DTR2_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);
                
                /*
                 * Pin name: UART_RI2_B
                 * Bute3A Signal: RI2_B
                 * Bute4A Signal: DAI_SCKA
                 * Functional Mode: ipp_uart2_ri_dce_o_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_RI2_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);

                /*
                 * Pin name: UART_DCD2_B
                 * Bute3A Signal: DCD2_B
                 * Bute4A Signal: DAI_FS
                 * Functional Mode: ipp_uart2_dcd_dce_o_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_DCD2_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);
            }
            break; /* case 1 */

        /* UART 3 IOMUX Configs */
        case 2:
            if (no_irda == 1) {
                /*
                 * Pin name: UART_TXD3
                 * Bute3A Signal: GPS_TXD
                 * Bute4A Signal: TXD3
                 * Functional Mode: ipp_uart3_txd (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_TXD3, OUTPUTCONFIG_FUNC,
                        INPUTCONFIG_FUNC);

                /*
                 * Pin name: UART_RXD3
                 * Bute3A Signal: GPS_RXD
                 * Bute4A Signal: RXD3
                 * Functional Mode: ipp_uart3_rxd (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_RXD3, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);

                /*
                 * Pin name: UART_RTS3_B
                 * Bute3A Signal: GPS_RTS_B
                 * Bute4A Signal: RTS3_B
                 * Functional Mode: ipp_uart3_rts_b (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_RTS3_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);

                /*
                 * Pin name: UART_CTS3_B
                 * Bute3A Signal: GPS_CTS_B
                 * Bute4A Signal: CTS3_B
                 * Functional Mode: ipp_uart3_cts_b (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_UART_CTS3_B, OUTPUTCONFIG_FUNC, 
                        INPUTCONFIG_FUNC);
            }
            break; /* case 2 */

        /* UART4 IOMUX Configuration - IRDA */
        case 3:
            if (no_irda == 0) {
                /*
                 * IOMUX configs for Irda pins
                 */
                /*
                 * Pin Name: IRDA_TX4
                 * Bute3A Signal: IRDA_TX
                 * Bute4A Signal: IRDA_TX
                 * Functional Mode: ipp_uart4_txd_ir (output)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, 
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_IRDA_TX4, OUTPUTCONFIG_FUNC,
                        INPUTCONFIG_NONE);

                /*
                 * Pin Name: IRDA_RX4
                 * Bute3A Signal: IRDA_RX
                 * Bute4A Signal: IRDA_RX
                 * Functional Mode: ipp_uart4_rxd_ir (input)
                 * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC,
                 *      INPUTCONFIG_FUNC)
                 */
                iomux_config_mux(PIN_IRDA_RX4, OUTPUTCONFIG_FUNC,
                        INPUTCONFIG_FUNC);

                /* Pin GPIO14 muxed to MCU GPIO Port Pin 14 at boot. */
                gpio_signal_set_data(GPIO_SIGNAL_IRDA_SD, GPIO_DATA_LOW);
            } 
            break; /* case 3 */

        default:
            break;
    }
}


/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
    /*
    * Disable the UART Transceiver by configuring the GPIO pin
    */
    switch (port) {
        case 0:
            /*
             * Disable the UART 1 Transceiver
             */
            break;

        case 1:
            /*
             * Disable the UART 2 Transceiver
             */
            break;

        case 2:
            /*
             * Disable the UART 3 Transceiver
             */
            break;

        case 3:
            /*
             * Disable the Irda Transmitter
             */
            gpio_signal_set_data(GPIO_SIGNAL_IRDA_SD, GPIO_DATA_HIGH);
            break;

        default:
            break;
    }
}


/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port) 
{
    switch (port) {
        case 3:
            /* Configure to receive UART 4 SDMA events. */
            /* See Table 4-47 (GEN_P_REG1 register connection
             *   description in ArgonLV DTS 1.2. */
            iomux_config_gpr(MUX_PGP_FIRI, 0);
            break;
            
        default:
            break;
    }
}


/*!
 * Setup GPIO for a Keypad to be active
 */
void gpio_keypad_active(void)
{
    iomux_config_mux(PIN_KEY_COL0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_COL7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    iomux_config_mux(PIN_KEY_ROW7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
} 


/*!
 * Setup GPIO for a keypad to be inactive
 */
void gpio_keypad_inactive(void)
{
    iomux_config_mux(PIN_KEY_COL0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL2, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL4, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_COL7, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW2, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW4, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
    iomux_config_mux(PIN_KEY_ROW7, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE); 
}


/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
    switch (cspi_mod) {
        case 0:
            /* CSPI1 is not connected on ButeP3A and ButeP4A */
            break; /* case 0 */

        case 1:
            /*
             * Pin Name: CSPI2_CS_0 (CSPI2_CS0)
             * Bute3A Signal: PM_SPI_CS
             * Bute4A Signal: PM_SPI_CS
             * Functional mode: SPI2_SS0
             *   ipp_ind_cspi2_ss0_b in; ipp_do_cspi2_ss0_b out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_CSPI2_CS_0, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);

            /*
             * Pin Name: CSPI2_DI
             * Bute3A Signal: BB_SPI_MISO
             * Bute4A Signal: BB_SPI_MISO
             * Functional mode: SPI2_MISO
             *   ipp_cspi2_ind_miso in; ipp_cspi2_do_miso out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_CSPI2_DI, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
        
            /*
             * Pin Name: CSPI2_DO
             * Bute3A Signal: BB_SPI_MOSI
             * Bute4A Signal: BB_SPI_MOSI
             * Functional mode: SPI2_MOSI
             *   ipp_cspi2_ind_mosi in; ipp_cspi2_do_mosi out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_CSPI2_DO, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            
            /*
             * Pin Name: CSPI2_CS_1 (CSPI2_CS1)
             * Bute3A Signal: (testpoint 1106)
             * Bute4A Signal: (no connection)
             * Functional mode: SPI2_SS1
             *   ipp_ind_cspi2_ss1_b in; ipp_do_cspi2_ss1_b out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_CSPI2_CS_1, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            
            /*
             * Pin Name: CSPI2_CK
             * Bute3A Signal: BB_SPI_CLK
             * Bute4A Signal: BB_SPI_CLK
             * Functional Mode: SPI2_CLK
             *   ipp_cspi2_clk_in in; ipp_cspi2_clk_out out 
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_CSPI2_CK, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);

            break; /* case 1 */

        default:
            break;
    }
}


/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
    switch (cspi_mod) {
        case 0:
            /* CSPI1 is not connected on this board */
            break;

        case 1:
            /* SPI2_SS0 */
            iomux_config_mux(PIN_CSPI2_CS_0, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_GPIO);
            /* SPI2_MISO */
            iomux_config_mux(PIN_CSPI2_DI, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
            /* SPI2_MOSI */
            iomux_config_mux(PIN_CSPI2_DO, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
            /* SPI2_SS1 */
            iomux_config_mux(PIN_CSPI2_CS_1, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_GPIO);
            /* SPI2_CLK */
            iomux_config_mux(PIN_CSPI2_CK, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);

            break;

        default:
            break;
    }
}


/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num) 
{
    switch (i2c_num) {
        case 0:
            /*
             * Pin Name: I2C_CLK
             * Bute3A Signal: I2C_CLK
             * Bute4A Signal: I2C_CLK
             * Functional Mode: ipp_i2c_scl_in/ipp_i2c_scl_out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_I2C_CLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            /*
             * Pin Name: I2C_DAT
             * Bute3A Signal: I2C_DATA / I2C_DAT
             * Bute4A Signal: I2C_DAT
             * Functional Mode: ipp_i2c_sda_in/ipp_i2c_sda_out
             * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_I2C_DAT, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            break;

        default:
            break;
    }
}


/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num) 
{
    switch (i2c_num) {
        case 0:
            iomux_config_mux(PIN_I2C_CLK, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
            iomux_config_mux(PIN_I2C_DAT, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
            break;

        default:
            break;
    }
}


/*!
 * Setup GPIO for FIRI port to be active
 */
void gpio_firi_active(void)
{
    /* Pin Name: IRDA_TX4
     * Bute3A Signal: IRDA_TX
     * Bute4A Signal: IRDA_TX
     * Mux2 Mode: ipp_do_firi_txd -- IR_TXD (FIRI) (output)
     * Out of Reset Setting: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    iomux_config_mux(PIN_IRDA_TX4, OUTPUTCONFIG_ALT2, INPUTCONFIG_ALT2);

    /* Pin NAme: IRDA_RX4
     * Bute3A Signal: IRDA_RX
     * Bute4A Signal: IRDA_RX
     * Mux2 Mode: ipp_ind_firi_rxd -- IR_RXD (FIRI) (input)
     * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
     */
    iomux_config_mux(PIN_IRDA_RX4, OUTPUTCONFIG_ALT2, INPUTCONFIG_ALT2);

    /* Pin GPIO14 muxed to MCU GPIO Port Pin 14 at boot. */
    gpio_signal_set_data(GPIO_SIGNAL_IRDA_SD, GPIO_DATA_LOW);
}


/*!
 * Configure the IOMUX GPR register to receive shared SDMA FIRI events
 *
 * @param  port         a UART port
 */
void config_firidma_event(void) 
{
    /* Configure to receive FIRI SDMA events */
    /* See Table 4-47 (GEN_P_REG1 register connection description)
     *   in ArgonLV DTS 1.2. */
    iomux_config_gpr(MUX_PGP_FIRI, 1);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API */


#ifdef CONFIG_MOT_FEAT_GPIO_API_MC13783
/*!
 * This function configures the Atlas interrupt operations.
 *
 */
void gpio_mc13783_active(void) 
{
}


/*!
 * This function clears the Atlas interrupt.
 *
 */
void gpio_mc13783_clear_int(void) 
{
}


/*!
 * This function return the SPI connected to Atlas.
 *
 */
int gpio_mc13783_get_spi(void) 
{
    /* Atlas uses CSPI2 (CSPI index is 0 based, so return 1.) */
    return 1;
}
        

/*!
 * This function return the SPI smave select for Atlas.
 *
 */
int gpio_mc13783_get_ss(void) 
{
    /* ARGONBUTE -> SS = 0 */
    return 0;
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_MC13783 */


#if defined(CONFIG_MOT_FEAT_GPIO_API_SDHC)
/*!
 * Setup IOMUX for Secure Digital Host Controller to be active.
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module) 
{
    switch(module) {
        case 0:
            /*
             * Pin: MMC1_CLK
             * Bute3A Signal Name: MMC1_CLK
             * Bute4A Signal Name: SD1_CLK
             * Functional Mode: ipp_do_sdhc1_mmc_clk (output)
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_CLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            /*
             * Pin: MMC1_CMD
             * Bute3A Signal Name: MMC1_CMD
             * Bute4A Signal Name: SD1_CMD
             * Functional Mode: ipp_ind_sdhc1_cmd/ipp_do_sdhc1_cmd
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_CMD, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            /*
             * Pin: MMC1_DATA_0
             * Bute3A Signal Name: MMC1_DATA[0]
             * Bute4A Signal Name: SD1_DATA[0]
             * Functional Mode: ipp_ind_sdhc1_data0/ipp_do_sdhc1_data0
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_DATA_0, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);

            /*
             * Pin: MMC1_DATA_1
             * Bute3A Signal Name: MMC1_DATA[1]
             * Bute4A Signal Name: SD1_DATA[1]
             * Functional Mode: ipp_ind_sdhc1_data1/ipp_do_sdhc1_data1
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_DATA_1, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);

            /*
             * Pin: MMC1_DATA_2
             * Bute3A Signal Name: MMC1_DATA[2]
             * Bute4A Signal Name: SD1_DATA[2]
             * Functional Mode: ipp_ind_sdhc1_data2/ipp_do_sdhc1_data2
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_DATA_2, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);

            /*
             * Pin: MMC1_DATA_3
             * Bute3A Signal Name: MMC1_DATA[3]
             * Bute4A Signal Name: SD1_DATA[3]
             * Functional Mode: ipp_ind_sdhc1_data3/ipp_do_sdhc1_data3
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC1_DATA_3, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            break;

        case 1:
            /*
             * Pin: MMC2_CLK
             * Bute3A Signal Name: MMC2_CLK
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_do_sdhc2_mmc_clk (output)
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_CLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            /*
             * Pin: MMC2_CMD
             * Bute3A Signal Name: MMC2_CMD
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_ind_sdhc2_cmd/ipp_do_sdhc2_cmd
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_CMD, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

            /*
             * Pin: MMC2_DATA_0
             * Bute3A Signal Name: MMC2_DATA[0]
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_ind_sdhc2_data0/ipp_do_sdhc2_data0
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_DATA_0, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            
            /*
             * Pin: MMC2_DATA_1
             * Bute3A Signal Name: MMC2_DATA[1]
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_ind_sdhc2_data1/ipp_do_sdhc2_data1
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_DATA_1, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            
            /*
             * Pin: MMC2_DATA_2
             * Bute3A Signal Name: MMC2_DATA[2]
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_ind_sdhc2_data2/ipp_do_sdhc2_data2
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_DATA_2, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            
            /*
             * Pin: MMC2_DATA_3
             * Bute3A Signal Name: MMC2_DATA[3]
             * Bute4A Signal Name: (no connection)
             * Functional Mode: ipp_ind_sdhc2_data3/ipp_do_sdhc2_data3
             * Out of Reset Setting:
             */
            iomux_config_mux(PIN_MMC2_DATA_3, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            break;

        default:
            break;
    }
}


/*!
 * Setup GPIO for Secure Digital Host Controller to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module) 
{
    switch(module) {
        case 0:
            iomux_config_mux(PIN_MMC1_CLK, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);      
            iomux_config_mux(PIN_MMC1_CMD, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);      
            iomux_config_mux(PIN_MMC1_DATA_0, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);
            iomux_config_mux(PIN_MMC1_DATA_1, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE); 
            iomux_config_mux(PIN_MMC1_DATA_2, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);
            iomux_config_mux(PIN_MMC1_DATA_3, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);      

            break;

        case 1:
            iomux_config_mux(PIN_MMC2_CLK, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);      
            iomux_config_mux(PIN_MMC2_CMD, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);      
            iomux_config_mux(PIN_MMC2_DATA_0, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);
            iomux_config_mux(PIN_MMC2_DATA_1, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE); 
            iomux_config_mux(PIN_MMC2_DATA_2, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);
            iomux_config_mux(PIN_MMC2_DATA_3, OUTPUTCONFIG_GPIO,
                    INPUTCONFIG_NONE);      
            break;

        default:
            break;
        }
}


/*!
 * Setup the IOMUX/GPIO for SDHC1 SD1_DET.
 * 
 * @param  host Pointer to MMC/SD host structure.
 * @param  handler      GPIO ISR function pointer for the GPIO signal.
 * @return The function returns 0 on success and -1 on failure.
 */
int sdhc_intr_setup(void *host, gpio_irq_handler handler) 
{
    /*
     * SD Card Detect (Host 1); Low=Card Present
     *
     * Pin GPIO17 muxed to MCU GPIO Port Pin 17 at boot.
     */
    gpio_signal_config(GPIO_SIGNAL_SD1_DET_B, GPIO_GDIR_INPUT,
           GPIO_INT_FALL_EDGE);
    return gpio_signal_request_irq(GPIO_SIGNAL_SD1_DET_B,
            GPIO_HIGH_PRIO, handler, 0, "MXCMMC", host);
}


/*!
 * Free the interrupt request for SD1_DET.
 *
 * @param  host Pointer to MMC/SD host structure.
 */
void sdhc_intr_destroy(void *host)
{
    gpio_signal_free_irq(GPIO_SIGNAL_SD1_DET_B, GPIO_HIGH_PRIO);
}


/*!
 * Clear the GPIO interrupt for SDHC1 SD1_DET.
 *
 * @param flag Flag represents whether the card is inserted/removed.
 *             Using this sensitive level of GPIO signal is changed.
 */
void sdhc_intr_clear(int *flag) 
{
    /* Actual interrupt is cleared by MXC interrupt handler. */
    
    if(*flag) {
        gpio_signal_config(GPIO_SIGNAL_SD1_DET_B, GPIO_GDIR_INPUT,
                GPIO_INT_FALL_EDGE);
        *flag = 0;
    } else {
        gpio_signal_config(GPIO_SIGNAL_SD1_DET_B, GPIO_GDIR_INPUT,
                GPIO_INT_RISE_EDGE);
        *flag = 1;
    }
}


/**
 * Find the minimum clock for SDHC.
 *
 * @param clk SDHC module number.
 * @return Returns the minimum SDHC clock.
 */
unsigned int sdhc_get_min_clock(enum mxc_clocks clk)
{
    return (mxc_get_clocks(SDHC1_CLK) / 8) / 32;
}


/**
 * Find the maximum clock for SDHC.
 * @param clk SDHC module number.
 * @return Returns the maximum SDHC clock.
 */
unsigned int sdhc_get_max_clock(enum mxc_clocks clk)
{
    return mxc_get_clocks(SDHC1_CLK) / 4;
}


/**
 * Probe for the card. Low=card present; high=card absent.
 */
int sdhc_find_card(int id)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_SD1_DET_B);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_SDHC */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LCD)
/*!
 * Setup GPIO for LCD to be active
 *
 */
void gpio_lcd_active(void)
{
    /* Main Display Shut Down; set low to enable display */
#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() >= BOARDREV_P4A) && (boardrev() < BOARDREV_P5A)
            && (boardrev() != BOARDREV_UNKNOWN)) {
        /* Pin GPIO14 muxed to MCU GPIO Port Pin 14 at boot. */
        gpio_signal_set_data(GPIO_SIGNAL_LCD_SD, GPIO_DATA_LOW);

        /*
         * Serializer Enable; set high to enable serializer.
         *
         * Pin USB1_VMOUT muxed to MCU GPIO Port Pin 28 at boot.
         */
        gpio_signal_set_data(GPIO_SIGNAL_SERDES_RESET_B, GPIO_DATA_HIGH);

        /* must wait 20us before waking up serializer */
        udelay(20);

        /*
         * Serializer Standby; set high to take wakeup serializer.
         * 
         * Pin USB1_TXENB muxed to MCU GPIO Port Pin 29 at boot.
         */
        gpio_signal_set_data(GPIO_SIGNAL_STBY, GPIO_DATA_HIGH);
    } else {
#endif /* CONFIG_MOT_FEAT_BRDREV */
        /* Pin USB1_VMOUT muxed to MCU GPIO Port Pin 28 at boot. */
        gpio_signal_set_data(GPIO_SIGNAL_DISP_RST_B, GPIO_DATA_LOW);
#ifdef CONFIG_MOT_FEAT_BRDREV
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /*
     * Magic numbers from the table "PWM settings" in the IPU chapter
     * of the Bute ICD.
     */
    /* set pwm duty cycle to 100% (controls brightness) */
    writel(0x0000000F, PWMSAR);
    /* pwm period is 16 clock cycles */
    writel(0x0000000D, PWMPR);
    /* PWM enablabed, PWM clock is 32k, Clock source is 32K, active in debug */
    writel(0x00430001, PWMCR);
}


/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
    /* Main Display Shut Down; set high to disable display */
#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() >= BOARDREV_P4A) && (boardrev() < BOARDREV_P5A)
            && (boardrev() != BOARDREV_UNKNOWN)) {
        /* Pin GPIO14 muxed to MCU GPIO Port Pin 14 at boot. */
        gpio_signal_set_data(GPIO_SIGNAL_LCD_SD, GPIO_DATA_HIGH);

        /*
         * Serializer Standby; set low to put serializer asleep.
         * 
         * Pin USB1_TXENB muxed to MCU GPIO Port Pin 29 at boot.
         */
        gpio_signal_set_data(GPIO_SIGNAL_STBY, GPIO_DATA_LOW);

        /* turn off serializer */
        gpio_signal_set_data(GPIO_SIGNAL_SERDES_RESET_B, GPIO_DATA_LOW);
    } else {
#endif /* CONFIG_MOT_FEAT_BRDREV */
        /* Pin USB1_VMOUT muxed to MCU GPIO Port Pin 28 at boot. */
    gpio_signal_set_data(GPIO_SIGNAL_DISP_RST_B, GPIO_DATA_HIGH);
#ifdef CONFIG_MOT_FEAT_BRDREV
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
/**
 * Turn on or off the main display backlight.
 *
 * @param enable True to turn on the backlight; false to turn it off.
 */
void gpio_lcd_backlight_enable(bool enable)
{
    /* 
     * Main Display Backlight -- set high to enable backlight.
     * 
     * Pin USB1_VPIN muxed to MCU GPIO Port Pin 30 at boot.
     */
    gpio_signal_set_data(GPIO_SIGNAL_LCD_BACKLIGHT,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);    
}


/**
 * Get status of lcd backlight gpio signals.
 *
 * @return  Status of the LCD_Backlight signal.
 */
int gpio_get_lcd_backlight(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_LCD_BACKLIGHT);
}


/**
 *  Set the display backlight intensity.
 *
 *  @param  value   Intensity level; range 0-100
 */
void pwm_set_lcd_bkl_brightness(int value)
{
    int dutycycle = DUTY_CYCLE_0;
    if (value > 0)
    {
        dutycycle = (value <= 50) ? DUTY_CYCLE_50 : DUTY_CYCLE_100;
    }
    /* set pwm duty cycle (controls brightness) */
    writel(dutycycle, PWMSAR);
}


/**
 * Get the display backlight intensity. 
 *
 * @return Backlight intensity, a value between 0 and 100. 
 */
int pwm_get_lcd_bkl_brightness(void)
{
    int dutycycle, value = 0;
    dutycycle = readl(PWMSAR);
    if (dutycycle > DUTY_CYCLE_0)
    {
        value = (dutycycle <=  DUTY_CYCLE_50) ? 50 : 100;
    }
    return value;
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_CSI)
/*!
 * Setup GPIO for Camera sensor to be active
 */
void gpio_sensor_active(void)
{
    /*
     * Camera Reset, Active Low
     *
     * Pin GPIO23 muxed to MCU GPIO Port Pin 18 at boot.
     */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_RST_B, GPIO_DATA_HIGH);

#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() < BOARDREV_P4A) || (boardrev() == BOARDREV_UNKNOWN)) {
#endif /* CONFIG_MOT_FEAT_BRDREV */

#if defined(CONFIG_MXC_IPU_CAMERA_SENSOR2M)
    /*
     * Camera Power Down (1.3MP Imager), Active High
     *
     * Pin GPIO16 muxed to MCU GPIO Port Pin 16 at boot.
     */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_INT_PWRDN, GPIO_DATA_LOW);
#endif

#ifdef CONFIG_MOT_FEAT_BRDREV
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}


/*!
 * Setup GPIO for camera sensor to be inactive
 */
void gpio_sensor_inactive(void)
{
    /* GPIO_CAM_RST_B -- GPIO_Shared_3 -- Camera Reset, Active Low */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_RST_B, GPIO_DATA_LOW);

#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() < BOARDREV_P4A) || (boardrev() == BOARDREV_UNKNOWN)) {
#endif /* CONFIG_MOT_FEAT_BRDREV */

#if defined(CONFIG_MXC_IPU_CAMERA_SENSOR2M)
    /* CSI_CS1 -- Camera Power Down (1.3MP Imager), Active High */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_INT_PWRDN, GPIO_DATA_HIGH);
#endif

#ifdef CONFIG_MOT_FEAT_BRDREV
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_CSI */


#if defined(CONFIG_MOT_FEAT_GPIO_API_ETHERNET)
/**
 * Configure GPIO to receive ethernet interrupts.
 *
 * @param handler The function that will handle the Ethernet interrupt.
 * @param irq_flags Interrupt options. (Usually 0.)
 * @param devname Name of the device requesting the interrupt.
 * @param dev_id Handle to the device driver (or null).
 *
 * @return Zero on success; non-zero on failure.
 */
int enet_request_irq(
        irqreturn_t (*handler)(int, void *, struct pt_regs *),
        unsigned long irq_flags, const char * devname, void *dev_id)
{
#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() >= BOARDREV_P4A) && (boardrev() != BOARDREV_UNKNOWN)) {
        return -EBUSY;
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /* Pin GPIO8 muxed to MCU GPIO Port Pin 8 at boot. */
    gpio_signal_config(GPIO_SIGNAL_ENET_INT_B, GPIO_GDIR_INPUT,
           GPIO_INT_HIGH_LEV);
    return gpio_signal_request_irq(GPIO_SIGNAL_ENET_INT_B,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);
}


/**
 * Configure GPIO to no longer receive ethernet interrupts.
 *
 * @param dev_id Handle to the device driver (or null).
 *
 * @return Zero on success; non-zero on failure.
 */
int enet_free_irq(void *dev_id)
{
#ifdef CONFIG_MOT_FEAT_BRDREV
    if((boardrev() >= BOARDREV_P4A) && (boardrev() != BOARDREV_UNKNOWN)) {
        return -EBUSY;
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /* ENET_INT_B -- GPIO_MCU_8 -- ethernet interrupt */
    gpio_signal_config(GPIO_SIGNAL_ENET_INT_B, GPIO_GDIR_INPUT,
            GPIO_INT_NONE);
    gpio_signal_free_irq(GPIO_SIGNAL_ENET_INT_B,
            GPIO_HIGH_PRIO);
    return 0;
}


/**
 * Clear any pending Ethernet interrupt.
 */
void enet_clear_int(void)
{
    /* NO OP */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_ETHERNET */


#if defined(CONFIG_MOT_FEAT_GPIO_API_BTPOWER)
/**
 * Install handler for BT_HOST_WAKE_B interrupt.
 *
 * @param   handler     Function to be called when interrupt arives.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Name of device driver.
 * @param   dev_id      Device identifier to pass to request_irq.
 *
 * @return  Zero on success; non-zero on failure.
 */
int gpio_bluetooth_hostwake_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id)
{
    int retval;

    retval = gpio_signal_request_irq(GPIO_SIGNAL_BT_HOST_WAKE_B,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);

    /* if request_irq was successful, configure the signal for interrupts  */
    if(retval == 0) {
        /* the clear_int function sets the proper trigger edge */
        gpio_bluetooth_hostwake_clear_int();
    }

    return retval;
}


/**
 * Remove handler for BT_HOST_WAKE_B interrupt.
 *
 * @param   dev_id      Device identifier to pass to free_irq.
 */
void gpio_bluetooth_hostwake_free_irq(void *dev_id)
{
    gpio_signal_config(GPIO_SIGNAL_BT_HOST_WAKE_B, GPIO_GDIR_INPUT,
            GPIO_INT_NONE);
    gpio_signal_free_irq(GPIO_SIGNAL_BT_HOST_WAKE_B, GPIO_HIGH_PRIO);
}


/**
 * Clear the BT_HOST_WAKE_B interrupt.
 *
 * ArgonLV GPIO doesn't support both edge triggered interrupts. So this
 * function adjusts the edge trigger of the interrupt based on its current
 * state.
 */
void gpio_bluetooth_hostwake_clear_int(void)
{
    gpio_signal_clear_int(GPIO_SIGNAL_BT_HOST_WAKE_B);
    
    /* Adjust the edge trigger for the interrupt. */
    gpio_signal_config(GPIO_SIGNAL_BT_HOST_WAKE_B, GPIO_GDIR_INPUT,
            gpio_bluetooth_hostwake_get_data() ? GPIO_INT_FALL_EDGE
            : GPIO_INT_RISE_EDGE);
}


/**
 * Get the current status of BT_HOST_WAKE_B.
 *
 * @return  Zero if signal is low; non-zero if signal is high.
 */
__u32 gpio_bluetooth_hostwake_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_BT_HOST_WAKE_B);
}


/**
 * Set the state of the BT_WAKE_B signal.
 *
 * @param   data    Desired state for the signal.
 */
void gpio_bluetooth_wake_set_data(__u32 data)
{
    gpio_signal_set_data(GPIO_SIGNAL_BT_WAKE_B, data);
}


/**
 * Get the current status of the USB_HS_DMA_REQ signal.
 */
__u32 gpio_bluetooth_wake_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_BT_WAKE_B);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_BTPOWER */


#ifdef CONFIG_MOT_FEAT_GPIO_API_ETM
/**
 * Setup IOMUX for ETM.
 *
 * @param   alternative Choices of pins onto which to place ETM functions.
 */
void etm_iomux_config(enum etm_iomux alternative)
{
    switch(alternative) {
        case ETM_MUX_IPU: /* ETM on IPU port (WJ1500) */
            /*
             * Pin Name: IPU_VSYNC3
             * Signal Name: IPU_VSYNC3
             * Function: TRCLK (ipp_do_etm_traceclk)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_VSYNC3, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * WJ1500 Pin 8 is connected to IPU_DBACK/IPU_HSYNC/KEY_COL(1) which
             * are all marked as DNP.
             */
            /*
             * Pin Name: IPU_VSYNC0
             * Signal Name: IPU_VSYNC0
             * Function: EXTRIG (ipp_ind_arm11p_ext_etm_extin3)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_VSYNC0, OUTPUTCONFIG_ALT1, 
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_7
             * Signal Name: IPU_FDAT(7)
             * Function: TRACE7 (ipp_do_etm_tracedata[7])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_7, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_6
             * Signal Name: IPU_FDAT(6)
             * Function: TRACE6 (ipp_do_etm_tracedata[6])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_6, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_5
             * Signal Name: IPU_FDAT(5)
             * Function: TRACE5 (ipp_do_etm_tracedata[5])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_5, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_4
             * Signal Name: IPU_FDAT(4)
             * Function: TRACE4 (ipp_do_etm_tracedata[4])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_4, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_3
             * Signal Name: IPU_FDAT(3)
             * Function: TRACE3 (ipp_do_etm_tracedata[3])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_3, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_2
             * Signal Name: IPU_FDAT(2)
             * Function: TRACE2 (ipp_do_etm_tracedata[2])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_2, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_1
             * Signal Name: IPU_FDAT(1)
             * Function: TRACE1 (ipp_do_etm_tracedata[1])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_1, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * WJ1500 Pin 30 is connected to KEY_ROW(1) and GND_BB both of
             * which have resistors marked DNP.
             */
            /*
             * WJ1500 Pin 32 is connected to KEY_ROW(0) and GND_BB both of
             * which have resistors marked DNP.
             */
            /*
             * Pin Name: IPU_FDAT_17
             * Signal Name: IPU_FDAT(17)
             * Function: TRCTL (ipp_do_etm_tracectl)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_17, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_0
             * Signal Name: IPU_FDAT(0)
             * Function: TRACE0 (ipp_do_etm_tracedata[0])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_0, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_8
             * Signal Name: IPU_FDAT(8)
             * Function: TRACE8 (ipp_do_etm_tracedata[8])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_8, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_9
             * Signal Name: IPU_FDAT(9)
             * Function: TRACE9 (ipp_do_etm_tracedata[9])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_9, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_10
             * Signal Name: IPU_FDAT(10)
             * Function: TRACE10 (ipp_do_etm_tracedata[10])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_10, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_11
             * Signal Name: IPU_FDAT(11)
             * Function: TRACE11 (ipp_do_etm_tracedata[11])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_11, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_12
             * Signal Name: IPU_FDAT(12)
             * Function: TRACE12 (ipp_do_etm_tracedata[12])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_12, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_13
             * Signal Name: IPU_FDAT(13)
             * Function: TRACE13 (ipp_do_etm_tracedata[13])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_13, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_14
             * Signal Name: IPU_FDAT(14)
             * Function: TRACE14 (ipp_do_etm_tracedata[14])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_14, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_FDAT_15
             * Signal Name: IPU_FDAT(15)
             * Function: TRACE15 (ipp_do_etm_tracedata[15])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_FDAT_15, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            break; /* ETM on IPU port (WJ1500) */

        case ETM_MUX_CSI_KPP: /* ETM on CSI/KPP port (WJ1502) */
            /*
             * Pin Name: KEY_COL1
             * Signal Name: KEY_COL(1)
             * Function: TRCLK (ipp_do_etm_traceclk)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL1, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * WJ1502 Pin 8 is connected to CSI_DBACK/IPU_CSI_VSYNC/KEY_COL(1)
             * all of which are connected via resistors marked DNP.
             */
            /*
             * Pin Name: EVTI_B
             * Signal Name: EVTI_B
             * Function: ETM_EXTRIG (ipp_ind_arm11p_ext_etm_extin3)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_EVTI_B, OUTPUTCONFIG_ALT2, INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_COL7
             * Signal Name: KEY_COL(7)
             * Function: TRACE7 (ipp_do_etm_tracedata[7])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL7, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_COL6
             * Signal Name: KEY_COL(6)
             * Function: TRACE6 (ipp_do_etm_tracedata[6])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL6, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_COL5
             * Signal Name: KEY_COL(5)
             * Function: TRACE5 (ipp_do_etm_tracedata[5])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL5, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_COL4
             * Signal Name: KEY_COL(4)
             * Function: TRACE4 (ipp_do_etm_tracedata[4])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL4, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_ROW7
             * Signal Name: KEY_ROW(7)
             * Function: TRACE3 (ipp_do_etm_tracedata[3])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_ROW7, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_ROW6
             * Signal Name: KEY_ROW(6)
             * Function: TRACE2 (ipp_do_etm_tracedata[2])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_ROW6, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_ROW5
             * Signal Name: KEY_ROW(5)
             * Function: TRACE1 (ipp_do_etm_tracedata[1])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_ROW5, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * WJ1502 Pin 30 is connected to KEY_ROW(1) and GND_BB both of
             * which have resistors marked DNP.
             */
            /*
             * WJ1502 Pin 32 is connected to KEY_ROW(0) and GND_BB both of
             * which have resistors marked DNP.
             */
            /*
             * Pin Name: KEY_COL3
             * Signal Name: KEY_COL(3)
             * Function: TRCTL (ipp_do_etm_tracectl)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_COL3, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: KEY_ROW4
             * Signal Name: KEY_ROW(4)
             * Function: TRACE0 (ipp_do_etm_tracedata[0])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_KEY_ROW4, OUTPUTCONFIG_ALT3,
                    INPUTCONFIG_ALT2);
            /*
             * Pin Name: IPU_CSI_DATA_0
             * Signal Name: IPU_CSI_DATA(0)
             * Function: TRACE8 (ipp_do_etm_tracedata[8])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_0, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_1
             * Signal Name: IPU_CSI_DATA(1)
             * Function: TRACE9 (ipp_do_etm_tracedata[9])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_1, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_2
             * Signal Name: IPU_CSI_DATA(2)
             * Function: TRACE10 (ipp_do_etm_tracedata[10])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_2, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_3
             * Signal Name: IPU_CSI_DATA(3)
             * Function: TRACE11 (ipp_do_etm_tracedata[11])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_3, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_4
             * Signal Name: IPU_CSI_DATA(4)
             * Function: TRACE12 (ipp_do_etm_tracedata[12])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_4, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_5
             * Signal Name: IPU_CSI_DATA(5)
             * Function: TRACE13 (ipp_do_etm_tracedata[13])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_5, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_6
             * Signal Name: IPU_CSI_DATA(6)
             * Function: TRACE14 (ipp_do_etm_tracedata[14])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_6, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            /*
             * Pin Name: IPU_CSI_DATA_7
             * Signal Name: IPU_CSI_DATA(7)
             * Function: TRACE15 (ipp_do_etm_tracedata[15])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_IPU_CSI_DATA_7, OUTPUTCONFIG_ALT1,
                    INPUTCONFIG_ALT1);
            break; /* ETM on CSI/KPP (WJ1502) */

        case ETM_MUX_DEFAULT: /* ETM on ETM port */
        default:
            /*
             * Pin Name: TRACE0
             * Signal Name: TRACE(0)
             * Function: TRACE0 (ipp_do_etm_tracedata[0])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE0, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE1
             * Signal Name: TRACE(1)
             * Function: TRACE1 (ipp_do_etm_tracedata[1])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE1, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE2
             * Signal Name: TRACE(2)
             * Function: TRACE2 (ipp_do_etm_tracedata[2])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE2, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE3
             * Signal Name: TRACE(3)
             * Function: TRACE3 (ipp_do_etm_tracedata[3])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE3, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE4
             * Signal Name: TRACE(4)
             * Function: TRACE4 (ipp_do_etm_tracedata[4])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE4, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE5
             * Signal Name: TRACE(5)
             * Function: TRACE5 (ipp_do_etm_tracedata[5])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE5, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE6
             * Signal Name: TRACE(6)
             * Function: TRACE6 (ipp_do_etm_tracedata[6])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE6, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE7
             * Signal Name: TRACE(7)
             * Function: TRACE7 (ipp_do_etm_tracedata[7])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE7, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE8
             * Signal Name: TRACE(8)
             * Function: TRACE8 (ipp_do_etm_tracedata[8])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE8, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE9
             * Signal Name: TRACE(9)
             * Function: TRACE9 (ipp_do_etm_tracedata[9])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE9, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE10
             * Signal Name: TRACE(10)
             * Function: TRACE10 (ipp_do_etm_tracedata[10])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE10, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE11
             * Signal Name: TRACE(11)
             * Function: TRACE11 (ipp_do_etm_tracedata[11])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE11, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE12
             * Signal Name: TRACE(12)
             * Function: TRACE12 (ipp_do_etm_tracedata[12])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE12, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE13
             * Signal Name: TRACE(13)
             * Function: TRACE13 (ipp_do_etm_tracedata[13])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE13, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE14
             * Signal Name: TRACE(14)
             * Function: TRACE14 (ipp_do_etm_tracedata[14])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE14, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE15
             * Signal Name: TRACE(15)
             * Function: TRACE15 (ipp_do_etm_tracedata[15])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE15, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE16
             * Signal Name: TRACE(16)
             * Function: TRACE16 (ipp_do_etm_tracedata[16])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE16, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE17
             * Signal Name: TRACE(17)
             * Function: TRACE17 (ipp_do_etm_tracedata[17])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE17, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE18
             * Signal Name: TRACE(18)
             * Function: TRACE18 (ipp_do_etm_tracedata[18])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE18, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE19
             * Signal Name: TRACE(19)
             * Function: TRACE19 (ipp_do_etm_tracedata[19])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE19, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE20
             * Signal Name: TRACE(20)
             * Function: TRACE20 (ipp_do_etm_tracedata[20])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE20, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE21
             * Signal Name: TRACE(21)
             * Function: TRACE21 (ipp_do_etm_tracedata[21])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE21, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE22
             * Signal Name: TRACE(22)
             * Function: TRACE22 (ipp_do_etm_tracedata[22])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE22, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE23
             * Signal Name: TRACE(23)
             * Function: TRACE23 (ipp_do_etm_tracedata[23])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE23, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE24
             * Signal Name: TRACE(24)
             * Function: TRACE24 (ipp_do_etm_tracedata[24])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE24, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE25
             * Signal Name: TRACE(25)
             * Function: TRACE25 (ipp_do_etm_tracedata[25])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE25, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE26
             * Signal Name: TRACE(26)
             * Function: TRACE26 (ipp_do_etm_tracedata[26])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE26, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE27
             * Signal Name: TRACE(27)
             * Function: TRACE27 (ipp_do_etm_tracedata[27])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE27, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE28
             * Signal Name: TRACE(28)
             * Function: TRACE28 (ipp_do_etm_tracedata[28])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE28, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE29
             * Signal Name: TRACE(29)
             * Function: TRACE29 (ipp_do_etm_tracedata[29])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE29, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE30
             * Signal Name: TRACE(30)
             * Function: TRACE30 (ipp_do_etm_tracedata[30])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE30, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRACE31
             * Signal Name: TRACE(31)
             * Function: TRACE31 (ipp_do_etm_tracedata[31])
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRACE31, OUTPUTCONFIG_FUNC,
                    INPUTCONFIG_FUNC);
            /*
             * Pin Name: DBGACK
             * Signal Name: DBGACK
             * Function: DBGACK (arm_dbgack)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_DBGACK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
            /*
             * Pin Name: TRCLK
             * Signal Name: TRCLK
             * Function: TRCLK (ipp_do_etm_traceclk)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_TRCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
            /*
             * Pin Name: SRST
             * Signal Name: TRCTL
             * Function: TRCTL (ipp_do_etm_tracectl)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_SRST, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
            /*
             * Pin Name: EXTRIG
             * Signal Name: EXTRIG
             * Function: EXTRIG (ipp_ind_arm11p_ext_etm_extin3)
             * Reset Default: 0x12 (OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)
             */
            iomux_config_mux(PIN_EXTRIG, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
            break; /* ETM on ETM port */
    }

}


/**
 * Adjust clock settings so that ETM trigger clock behaves properly.
 */
void etm_enable_trigger_clock(void)
{
    /* stub */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_ETM */


#ifdef CONFIG_MOT_FEAT_GPIO_API_DAI
void gpio_dai_enable(void)
{
    /* Stub function to work around error in APAL driver. */
    return;
}


void gpio_dai_disable(void)
{
    /* Stub function to work around error in APAL driver. */
    return;
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_DAI */


#if defined(CONFIG_MOT_FEAT_FLIP)
/**
 * Register a handler for the flip event.
 *
 * @param   handler     Handler for the interrupt.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Unique name for the interrupt.
 * @param   dev_id      Parameter to pass to the interrupt handler.
 *
 * @return  Zero on success; non-zero on error.
 */
int  gpio_flip_request_irq(gpio_irq_handler handler, unsigned long irq_flags,
        const char *devname, void *dev_id)
{
    /* set the proper triggering for the interrupt in addition to
     * clearing any pending interrupts */
    gpio_flip_clear_int();

    return gpio_signal_request_irq(GPIO_SIGNAL_FLIP_DETECT,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);
}


/**
 * Unregister a previously requested flip interrupt handler.
 *
 * @param   dev_id      Paramter to pass to free_irq.
 *
 * @return  Zero on success; non-zero on error.
 */
int  gpio_flip_free_irq(void *dev_id)
{
    return gpio_signal_free_irq(GPIO_SIGNAL_FLIP_DETECT, GPIO_HIGH_PRIO);
}


/**
 * Clear any pending interrupts. This function will also adjust the edge
 * triggering of the interrupt based on the current state of FLIP_DETECT
 * signal. Since GPIO interrupts don't support both-edge interrupt triggering
 * we better hope we don't miss an interrupt.
 */
void gpio_flip_clear_int(void)
{
    gpio_signal_clear_int(GPIO_SIGNAL_FLIP_DETECT);
    
    gpio_signal_config(GPIO_SIGNAL_FLIP_DETECT, GPIO_GDIR_INPUT,
            gpio_flip_open() ? GPIO_INT_FALL_EDGE : GPIO_INT_RISE_EDGE);
}


/**
 * Determine the status of the flip.
 *
 * @return  True if the flip is flopped.
 */
int  gpio_flip_open(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_FLIP_DETECT);
}
#endif /* CONFIG_MOT_FEAT_FLIP */
