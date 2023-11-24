/*
 * linux/arch/arm/mach-mxc91231/scma11ref_gpio.c
 *
 * Copyright 2006-2007 Motorola, Inc.
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
 * 10-Nov-2006  Motorola        Update high speed USB API.
 * 13-Nov-2006  Motorola        Change pad group 25 settings.
 * 06-Dec-2006  Motorola        Moved etm function to independent file.
 * 26-Jan-2007  Motorola        Bluetooth current drain improvements.
 * 29-Jan-2007  Motorola        Added support for P3C wingboards.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <asm/arch/gpio.h>
#include <asm/mot-gpio.h>
#include <asm/io.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <linux/delay.h>
#include <linux/errno.h>

#ifdef CONFIG_MOT_FEAT_BRDREV
#include <asm/boardrev.h>
#endif /* CONFIG_MOT_FEAT_BRDREV */

#include "iomux.h"

/*!
 * @file scma11ref_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */


#ifdef CONFIG_MOT_FEAT_BRDREV
/* Adjust GPIO and IOMUX settings for P0C plus and previous Wing Boards. */
static void gpio_signal_fixup_p0c(void);
static void iomux_setting_fixup_p0c(void);

static void gpio_signal_fixup_p1a(void);

static void gpio_signal_fixup_p2aw(void);

static void gpio_signal_fixup_p2bw(void);

static void gpio_signal_fixup_p3aw(void);
#endif /* CONFIG_MOT_FEAT_BRDREV */


/**
 * Initial GPIO register settings.
 */
struct gpio_signal_settings initial_gpio_settings[MAX_GPIO_SIGNAL] = {
    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P3C Wingboard Signal: GPS_RESET (GPS)/LIN_VIB_AMP_EN
     *                                           (Linear Vibrator)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Array index: 0   GPIO_SIGNAL_GPS_RESET
     *
     * Disable GPS at reset to avoid interference with UART3.
     */
    { GPIO_SP_A_PORT,    2, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C14
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CLI_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CLI_BKL (Backlight)
     * Selected Primary Function: GP_AP_C14 (Output)
     *
     * Array index: 1   GPIO_SIGNAL_CLI_BKL
     *
     * Disable secondary display backlight at boot to prevent interference
     * with main display backlight.
     */
    { GPIO_AP_C_PORT,   14, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },
    
    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * SCM-A11 Reference P1A Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
     *
     * Array index: 2   GPIO_SIGNAL_BT_POWER
     *
     * Power off Bluetooth at boot time. (Signal is connected to Bluetooth's
     * VREG_CTL on P2A wingboard and later.)
     */
    { GPIO_AP_C_PORT,    8, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * SCM-A11 Reference P1A Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
     *
     * Array index: 3   GPIO_SIGNAL_BT_RESET_B
     *
     * On P1 wingboards, the BT controller is brought out of reset
     * at boot by setting GP_AP_C8 high and then powered down
     * by setting GP_AP_C12 low.
     */
    { GPIO_INVALID_PORT,     8, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: SPI1_MOSI
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: SPI1_MOSI (Wing Connector)
     * SCM-A11 Reference P2A Wingboard Signal: USB_HS_RESET (Wing Connector)
     * Selected Primary Function: GP_SP_A28 (Output)
     *
     * Array index: 4   GPIO_SIGNAL_USB_HS_RESET
     *
     * Put the high speed USB controller into reset at boot.
     */
    { GPIO_SP_A_PORT,   28, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_B22
     * SCM-A11 Reference P1A Wingboard Signal: GP_AP_B22 (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: GP_AP_B22 (Wing Connector)
     * Selected Primary Function: GP_AP_B22 (Input)
     *
     * Array index: 5   GPIO_SIGNAL_USB_HS_DMA_REQ
     */
    { GPIO_AP_B_PORT,   22, GPIO_GDIR_INPUT,  GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C9
     * SCM-A11 Reference P1A Wingboard Signal: BOOT_RS232_USB (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: 10K Pulldown (Misc)
     * Selected Primary Function: GP_AP_C9 (Output)
     *
     * Array index: 6   GPIO_SIGNAL_USB_HS_WAKEUP
     *
     * Put the high speed USB controller into sleep mode.
     */
    { GPIO_AP_C_PORT,    9, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },
    
    /*
     * SCM-A11 Package Pin Name: GP_AP_C10
     * SCM-A11 Reference P1A Wingboard Signal: USB_HS_DMA_ACK (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: USB_HS_DMA_ACK (Wing Connector)
     * Selected Primary Function: GP_AP_C10 (Input)
     *
     * Array index: 7   GPIO_SIGNAL_USB_HS_FLAGC
     */
    { GPIO_AP_C_PORT,   10, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: ED_INT6
     * SCM-A11 Reference P1A Wingboard Signal: USB_HS_INT (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: USB_HS_INT (Wing Connector)
     * Selected Primary Function: GP_AP_C24 (Input)
     * Selected Secondary Function: ED_INT6 (Input)
     *
     * Array index: 8   GPIO_SIGNAL_USB_HS_INT
     *
     * High speed USB interrupt; setup to receive interrupts later.
     */
    { GPIO_AP_C_PORT,   24, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: UH2_TXDP
     * SCM-A11 Reference P1A Wingboard Signal: IO_LED_DBG (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: IO_LED_DBG (Misc)
     * SCM-A11 Reference P2B Wingboard Signal: USB_HS_SWITCH (Wing Connector)
     * Selected Primary Function: GP_SP_A11 (Output)
     *
     * Array index: 9   GPIO_SIGNAL_USB_HS_SWITCH
     *
     * USB connected to Atlas when low.
     */
    { GPIO_SP_A_PORT,   11, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: SD2_DAT1
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(1) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(1) (WLAN)
     * Selected Primary Function: SD2_DAT1 (Input/Output)
     *
     * Array index: 10  GPIO_SIGNAL_SD1_DET
     *
     * sdhc_intr_setup puts SD2_DAT1 into GPIO mode as GP_SP_A23.
     */
    { GPIO_SP_A_PORT,   23, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: U2_RI_B
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_CM/DAI_CLK (Display)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_CM/DAI_CLK (Display)
     * Selected Primary Function: GP_AP_C27 (Output)
     *
     * Array index: 11  GPIO_SIGNAL_DISP_CM
     *
     * Main display color mode; set low for high color mode.
     */
    { GPIO_AP_C_PORT,   27, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: IPU_D0_CS
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_SD (Display)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_SD (Display)
     * Selected Primary Function: GP_AP_A26 (Output)
     *
     * Array index: 12  GPIO_SIGNAL_DISP_SD
     *
     * Set low to enable main display; high to shut down. Set by MBM at boot.
     */
    { GPIO_AP_A_PORT,   26, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_B17
     * SCM-A11 Reference P1A Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: PWM_BKL (Backlight)
     * Selected Primary Function: GP_AP_B17 (Output)
     *
     * Array index: 13  GPIO_SIGNAL_PWM_BKL
     *
     * This signal is no longer connected to the backlight driver for
     * P1A and later wingboards.
     */
    { GPIO_INVALID_PORT,    17, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: IPU_D3_SPL
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_MAIN_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_MAIN_BKL (Backlight)
     * Selected Primary Function: GP_AP_A30 (Output)
     *
     * Array index: 14  GPIO_SIGNAL_MAIN_BKL
     *
     * Set high to turn on the main display backlight. Set by MBM at boot.
     */
    { GPIO_AP_A_PORT,   30, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: CSI_D0
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CAM_RST_B (camera)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CAM_RST_B (camera)
     * Selected Primary Function: GP_AP_B24 (Output)
     *
     * Array index: 15  GPIO_SIGNAL_CAM_RST_B
     *
     * Take camera out of reset at boot.
     */
    { GPIO_AP_B_PORT,   24, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: CSI_D1
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CAM_PD (camera)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CAM_PD (camera)
     * Selected Primary Function: GP_AP_B25 (Output)
     *
     * Array index: 16  GPIO_SIGNAL_CAM_PD
     *
     * Power down camera at boot.
     */
    { GPIO_AP_B_PORT,   25, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: ED_INT3
     * SCM-A11 Reference P1A Wingboard Signal: CPLD_IO_ENET_INT (Ethernet)
     * SCM-A11 Reference P1D Wingboard Signal: CPLD_IO_ENET_INT/(hall effect)
     *                                          (Ethernet)
     * Selected Primary Function: GP_AP_C21 (Input)
     * Selected Secondary Function: ED_INT3 (Input)
     *
     * Array index: 17  GPIO_SIGNAL_ENET_INT
     */
    { GPIO_AP_B_PORT,   21, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_B18
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_RST_B (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_RST_B (CLI)
     * Selected Primary Function: GP_AP_B18 (Output)
     *
     * Array index: 18  GPIO_SIGNAL_DISP_RST_B
     *
     * This is described as the secondary display reset in the ICD; however, for
     * P2A closedphone this line is used as the LCD clock select.
     */
    { GPIO_AP_B_PORT,   18, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: IPU_PAR_RS
     * SCM-A11 Reference P1A Wingboard Signal: IPU_PAR_RS (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_PAR_RS (Display)
     * SCM-A11 Reference P2B Wingboard Signal: SER_EN (Display)
     * Selected Primary Function: GP_AP_A29 (Output)
     *
     * Array index: 19  GPIO_SIGNAL_SER_EN
     *
     * Serializer enable; set high to enable. (Configured at boot by MBM.)
     */
    { GPIO_AP_A_PORT,   29, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: U3_CTS_B
     * SCM-A11 Reference P1A Wingboard Signal: VVIB_EN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: VVIB_EN (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P3C Wingboard Signal: WLAN_CLIENT_WAKE_B
     *                                          (WLAN)/UI_IC_DBG (Morphing)
     * Selected Primary Function: GP_SP_A3 (Output)
     *
     * Array index: 20  GPIO_SIGNAL_WLAN_CLIENT_WAKE_B
     *
     * Put WLAN to sleep at boot.
     */
    { GPIO_SP_A_PORT,    3, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: UH2_RXDM
     * SCM-A11 Reference P1A Wingboard Signal: TF_ENABLE (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: TF_ENABLE (MMC)
     * SCM-A11 Reference P2B Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * Selected Primary Function: GP_SP_A13 (Output)
     *
     * Array index: 21  GPIO_SIGNAL_CAM_TORCH_EN
     */
    { GPIO_SP_A_PORT,   13, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_SP_A26
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLK_EN_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLK_EN_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * Selected Primary Function: GP_SP_A26 (Input)
     *
     * Array index: 22  GPIO_SIGNAL_WLAN_PWR_DWN_B
     */
    { GPIO_SP_A_PORT,       26, GPIO_GDIR_OUTPUT,    GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: SPI1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: EL_NUM_EN (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     *
     * Array index: 23  GPIO_SIGNAL_EL_NUM_EN
     */
    { GPIO_INVALID_PORT,    27, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: SPI1_MISO (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     *
     * Array index: 24  GPIO_SIGNAL_EL_NAV_EN
     */
    { GPIO_INVALID_PORT,    29, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C12
     * SCM-A11 Reference P1A Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P2A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_RESET (WLAN)
     * Selected Primary Function: GP_AP_C12 (Output)
     *
     * Array index: 25   GPIO_SIGNAL_WLAN_RESET
     *
     * Take WLAN out of reset. (Put to sleep by WLAN_CLIENT_WAKE_B.)
     */
    { GPIO_AP_C_PORT,   12, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: ED_INT4
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DET_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DET_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: TF_DET (MMC)
     * Selected Primary Function: GP_AP_C22 (Input)
     * Selected Secondary Function: ED_INT4 (Input)
     *
     * Array index: 26  GPIO_SIGNAL_TF_DET
     */
    { GPIO_AP_C_PORT,   22, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C13
     * SCM-A11 Reference P1A Wingboard Signal: BT_WAKE_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_WAKE_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: BT_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C13 (Output)
     *
     * Array index: 27  GPIO_SIGNAL_BT_WAKE_B
     *
     * Set low to wakeup blueooth.
     */
    { GPIO_AP_C_PORT,   13, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: ED_INT1
     * SCM-A11 Reference P1A Wingboard Signal: BT_HOST_WAKE_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_HOST_WAKE_B (Bluetooth)
     * SCM-A11 Reference P2A Wingboard Signal: BT_HOST_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C19 (Input)
     * Selected Secondary Function: ED_INT1 (Input)
     *
     * Array index: 28  GPIO_SIGNAL_BT_HOST_WAKE_B
     *
     * Host wake interrupt from bluetooth controller.
     */
    { GPIO_AP_C_PORT,   19, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P3C Wingboard Signal: GPS_RESET (GPS)/LIN_VIB_AMP_EN
     *                                           (Linear Vibrator)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Array index: 29  GPIO_SIGNAL_LIN_VIB_AMP_EN
     *
     * Set high to enable vibrator.
     */
    { GPIO_INVALID_PORT,     2, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: ED_INT0
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_HOST_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_HOST_WAKE_B (WLAN)
     * Selected Primary Function: GP_AP_C18 (Input)
     * Selected Secondary Function: ED_INT0 (Input)
     *
     * Array index: 30  GPIO_SIGNAL_WLAN_HOST_WAKE_B
     */
    { GPIO_AP_C_PORT,   18, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: SPI1_MISO (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     *
     * Array index: 31  GPIO_SIGNAL_EL_EN
     */
    { GPIO_SP_A_PORT,       29, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: VFUSE_SELECT (Misc)
     * SCM-A11 Reference P3A Wingboard Signal: FM_RESET (FM Radio)
     * Selected Primary Function: GP_SP_A9 (Output)
     *
     * Array index: 32  GPIO_SIGNAL_FM_RESET
     */
    { GPIO_SP_A_PORT,        9, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C11
     * SCM-A11 Reference P1A Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: FM_INTERRUPT (FM Radio)
     * Selected Primary Function: GP_AP_C11 (Input)
     *
     * Array index: 33  GPIO_SIGNAL_FM_INTERRUPT
     */
    { GPIO_AP_C_PORT,       11, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_RXDP (Saipan Connector)
     * SCM-A11 Reference P3A Wingboard Signal: UH2_RXDP (INTERRUPT) (Morphing)
     * SCM-A11 Reference P3C Wingboard Signal: TNLC_KCHG_INT (Morphing)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Array index: 34  GPIO_SIGNAL_TNLC_KCHG_INT
     */
    { GPIO_SP_A_PORT,       12, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: SPI1_SS1
     * SCM-A11 Reference P1A Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: SPI1_SS1 (2nd MORPH RESET)
     *                                                  (Morphing)
     * Selected Primary Function: GP_SP_A31 (Output)
     *
     * Array index: 35  GPIO_SIGNAL_TNLC_RESET
     */
    { GPIO_INVALID_PORT,    31, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_PWR
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_PWR (Saipan Connector)
     * SCM-A11 Reference P3A Wingboard Signal: UH2_RXDM (RESET) (Morphing)
     * Selected Primary Function: GP_SP_A15 (Output)
     * 
     * Array index: 36  GPIO_SIGNAL_CAP_RESET
     */
    { GPIO_SP_A_PORT,       15, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: OMEGA_INTERRUPT (Omega Wheel)
     * SCM-A11 Reference P3C Wingboard Signal: TNLC_RCHG (Morphing)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Array index: 37  GPIO_SIGNAL_TNLC_RCHG
     */
    { GPIO_SP_A_PORT,        8, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: U1_TXD
     * SCM-A11 Reference P3A Signal: BT_RX_AP_TX (Bluetooth)
     * Selected Primary Function: U1_TXD (Output)
     * Mux0 Function: GP_AP_A7
     *
     * SCM-A11 Reference Design uses Broadcom Bluetooth controller with
     * internal pull downs, so drive BT pins low when inactive to decrease
     * current drain.
     *
     * Array index: 38  GPIO_SIGNAL_U1_TXD
     */
    { GPIO_AP_A_PORT,        7, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: U1_CTS_B
     * SCM-A11 Reference P3A Signal: BT_CTS_B (Bluetooth)
     * Selected Primary Function: U1_CTS_B (Output)
     * Mux0 Function: GP_AP_A10
     *
     * SCM-A11 Reference Design uses Broadcom Bluetooth controller with
     * internal pull downs, so drive BT pins low when inactive to decrease
     * current drain.
     *
     * Array index: 39  GPIO_SIGNAL_U1_CTS_B
     */
    { GPIO_AP_A_PORT,       10, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: U3_CTS_B
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P3A Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P3C Wingboard Signal: WLAN_CLIENT_WAKE_B
     *                                          (WLAN)/UI_IC_DBG (Morphing)
     * Selected Primary Function: GP_SP_A3 (Output)
     *
     * Array index: 40  GPIO_SIGNAL_UI_IC_DBG
     */
    { GPIO_INVALID_PORT,     3, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },
    
    /*
     * SCM-A11 Package Pin Name: SPI1_SS1
     * SCM-A11 Reference P1A Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: SPI1_SS1 (2nd MORPH RESET)
     *                                                  (Morphing)
     * SCM-A11 Reference P3C Wingboard Signal: FSS_HYST (Morphing)
     * Selected Primary Function: GP_SP_A31 (Output)
     *
     * Array index: 41  GPIO_SIGNAL_FSS_HYST
     */
    { GPIO_SP_A_PORT,   31, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: IPU_WR
     * SCM-A11 Reference P3A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3C Signal: GPIO_SER_RST_B (Display)
     * Selected Primary Function: GP_AP_B1 (Output)
     *
     * Array index: 42  GPIO_SIGNAL_SER_RST_B
     *
     * Low means serializer is in reset.
     */
    { GPIO_AP_B_PORT,    1, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },
};


/**
 * Default IOMUX pad register settings.
 */
static struct {
    enum iopad_group grp;
    __u32 config;
} iomux_pad_register_settings[] __initdata = {
    /* SDRAM -- DSE_HIGH | PKE_ENABLE */
    { IOPAD_GROUP1, 0x0082 },

    /* SDRAM -- DSE_HIGH */
    { IOPAD_GROUP2, 0x0002 },

    /* SDRAM -- 0 */
    { IOPAD_GROUP3, 0x0000 },

    /* SDRAM -- DSE_HIGH | PKE_ENABLE */
    { IOPAD_GROUP4, 0x0082 },

    /* SDRAM -- DSE_HIGH */
    { IOPAD_GROUP5, 0x0002 },

    /* SDRAM -- SRE_FAST | PUS_100K_PULLUP | PKE_ENABLE */
    { IOPAD_GROUP6, 0x00C1 },

    /* SDRAM -- SRE_FAST | DSE_MIN | PKE_ENABLE */
    { IOPAD_GROUP7, 0x0087 },

    /* SDRAM -- SRE_FAST */
    { IOPAD_GROUP8, 0x0001 },

    /* SDRAM -- SRE_FAST | PKE_ENABLE */
    { IOPAD_GROUP9, 0x0081 },

    /* Audio & AP GPIOs -- PUS_22K_PULLUP */
    { IOPAD_GROUP10, 0x0060 },

    /* Keypad -- PUS_47K_PULLUP */
    { IOPAD_GROUP11, 0x0020 },

    /* Keypad -- DDR_MODE_DDR | PUS_47K_PULLUP */
    { IOPAD_GROUP12, 0x0220 },

    /* I2C -- DDR_MODE_DDR | PUS_100K_PULLUP */
    { IOPAD_GROUP13, 0x0240 },

    /* CSPI1 -- 0 */
    { IOPAD_GROUP14, 0x0000 },

    /* CKO & CKOH -- 0 */
    { IOPAD_GROUP15, 0x0000 },

    /* MQSPI -- 0 */
    { IOPAD_GROUP16, 0x0000 },

    /* EL1T -- 0 */
    { IOPAD_GROUP17, 0x0000 },

    /* ETM/NEXUS -- 0 */
    { IOPAD_GROUP18, 0x0000 },

    /* JTAG -- 0 */
    { IOPAD_GROUP19, 0x0000 },

    /* DIGRF -- 0 */
    { IOPAD_GROUP20, 0x0000 },

    /* ETM/NEXUS -- 0 */
    { IOPAD_GROUP21, 0x0000 },

    /* CSI (Camera) -- SRE_FAST | DSE_MIN */
    { IOPAD_GROUP22, 0x0007 },

    /* EDIO -- PUS_100K_PULLUP */
    { IOPAD_GROUP23, 0x0040 },

    /* EDIO -- PUS_100K_PULLUP */
    { IOPAD_GROUP24, 0x0040 },

    /* DIG --  HYS_SCHMITZ | PKE_ENABLE | PUS_22K_PULLUP | SRE_FAST*/
    { IOPAD_GROUP25, 0x01E1 },

    /* SDHC -- PUS_100K_PULLUP */
    { IOPAD_GROUP26, 0x0040 },

    /* SDHC -- PUS_100K_PULLUP */
    { IOPAD_GROUP27, 0x0040 },

    /* Not Used -- 0 */
    { IOPAD_GROUP28, 0x0000 },
};

#define IOMUX_PAD_SETTING_START  9 /* MBM initializes SDRAM pad registers */
#define IOMUX_PAD_SETTING_STOP  27


/**
 * Initial IOMUX settings.
 */
struct iomux_initialization {
    enum iomux_pins             pin;    
    enum iomux_output_config    output_config;
    enum iomux_input_config     input_config;
};


/**
 * Atlas IOMUX settings.
 */
static struct iomux_initialization atlas_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: AD1_TXD
     * SCM-A11 Reference P1A Wingboard Signal: ASAP_TX (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: ASAP_TX (Atlas)
     * Selected Primary Function: AD1_TXD (Output)
     *
     * Primary function out of reset: AD1_TXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A11
     */
    { AP_AD1_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD1_RXD
     * SCM-A11 Reference P1A Wingboard Signal: ASAP_RX (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: ASAP_RX (Atlas)
     * Selected Primary Function: AD1_RXD (Input)
     *
     * Primary function out of reset: AD1_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A12
     */
    { AP_AD1_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD1_TXC
     * SCM-A11 Reference P1A Wingboard Signal: ASAP_CLK (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: ASAP_CLK (Atlas)
     * Selected Primary Function: AD1_TXC (Input/Output)
     *
     * Primary function out of reset: AD1_TXC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A13
     */
    { AP_AD1_TXC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD1_TXFS
     * SCM-A11 Reference P1A Wingboard Signal: ASAP_FS (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: ASAP_FS (Atlas)
     * Selected Primary Function: AD1_TXFS (Input/Output)
     *
     * Primary function out of reset: AD1_TXFS
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A14
     */
    { AP_AD1_TXFS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD2_TXD
     * SCM-A11 Reference P1A Wingboard Signal: MMSAP_TX (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: MMSAP_TX (Atlas)
     * Selected Primary Function: AD2_TXD (Output)
     *
     * Primary function out of reset: AD2_TXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A15
     */
    { AP_AD2_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD2_RXD
     * SCM-A11 Reference P1A Wingboard Signal: MMSAP_RX (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: MMSAP_RX (Atlas)
     * Selected Primary Function: AD2_RXD (Input)
     *
     * Primary function out of reset: AD2_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A16
     */
    { AP_AD2_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD2_TXC
     * SCM-A11 Reference P1A Wingboard Signal: MMSAP_CLK (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: MMSAP_CLK (Atlas)
     * Selected Primary Function: AD2_TXC (Input/Output)
     *
     * Primary function out of reset: AD2_TXC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A17
     */
    { AP_AD2_TXC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD2_TXFS
     * SCM-A11 Reference P1A Wingboard Signal: MMSAP_FS (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: MMSAP_FS (Atlas)
     * Selected Primary Function: AD2_TXFS (Input/Output)
     *
     * Primary function out of reset: AD2_TXFS
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A18
     */
    { AP_AD2_TXFS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW5
     * SCM-A11 Reference P1A Wingboard Signal: LOBAT_B (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: LOBAT_B (Atlas)
     * Selected Primary Function: GP_AP_B16 (Input)
     *
     * Primary function out of reset: KPROW5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B16
     */
    { AP_KPROW5, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_B23
     * SCM-A11 Reference P1A Wingboard Signal: POWER_RDY (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: POWER_RDY (Atlas)
     * Selected Primary Function: GP_AP_B23 (Input)
     *
     * Primary function out of reset: GP_AP_B23
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B23
     */
    { AP_GPIO_AP_B23, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C15
     * SCM-A11 Reference P1A Wingboard Signal: WDOG_AP (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: WDOG_AP (Atlas)
     * Selected Primary Function: WDOG_AP (Output)
     *
     * Primary function out of reset: GP_AP_C15
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C15
     */
    { AP_GPIO_AP_C15, OUTPUTCONFIG_FUNC4, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C16
     * SCM-A11 Reference P1A Wingboard Signal: USB_VPIN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_VPIN (Atlas)
     * Selected Primary Function: USB_VP1 (Input)
     * Selected Secondary Function: GP_AP_C16 (Input)
     *
     * Primary function out of reset: GP_AP_C16
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C16
     */
    { AP_GPIO_AP_C16, OUTPUTCONFIG_FUNC3, INPUTCONFIG_DEFAULT },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C17
     * SCM-A11 Reference P1A Wingboard Signal: USB_VMIN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_VMIN (Atlas)
     * Selected Primary Function: USB_VM1 (Input)
     * Selected Secondary Function: GP_AP_C17 (Input)
     *
     * Primary function out of reset: GP_AP_C17
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C17
     */
    { AP_GPIO_AP_C17, OUTPUTCONFIG_FUNC3, INPUTCONFIG_DEFAULT },
    /*
     * SCM-A11 Package Pin Name: ED_INT5
     * SCM-A11 Reference P1A Wingboard Signal: PM_INT (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: PM_INT (Atlas)
     * Selected Primary Function: GP_AP_C23 (Input)
     * Selected Secondary Function: ED_INT5 (Input)
     *
     * Primary function out of reset: ED_INT5
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C23
     */
    { AP_ED_INT5, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: ED_INT7
     * SCM-A11 Reference P1A Wingboard Signal: POWER_FAIL (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: POWER_FAIL (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_AP_C25 (Input)
     * Selected Secondary Function: ED_INT7 (Input)
     *
     * Primary function out of reset: ED_INT7
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C25
     */
    { AP_ED_INT7, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: U3_CTS_B
     * SCM-A11 Reference P1A Wingboard Signal: VVIB_EN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: VVIB_EN (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P3C Wingboard Signal: WLAN_CLIENT_WAKE_B
     *                                          (WLAN)/UI_IC_DBG (Morphing)
     * Selected Primary Function: GP_SP_A3 (Output)
     *
     * Primary function out of reset: U3_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A3
     */
    { SP_U3_CTS_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: USB_TXOE_B
     * SCM-A11 Reference P1A Wingboard Signal: USB_TXEN_B (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_TXEN_B (Atlas)
     * Selected Primary Function: USB_TXOE_B (Output)
     *
     * Primary function out of reset: USB_TXOE_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A4
     */
    { SP_USB_TXOE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: USB_DAT_VP
     * SCM-A11 Reference P1A Wingboard Signal: USB_VPOUT (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_VPOUT (Atlas)
     * Selected Primary Function: USB_DAT_VP (Input/Output)
     *
     * Primary function out of reset: USB_DAT_VP
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A5
     */
    { SP_USB_DAT_VP, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: USB_SE0_VM
     * SCM-A11 Reference P1A Wingboard Signal: USB_VMOUT (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_VMOUT (Atlas)
     * Selected Primary Function: USB_SE0_VM (Input/Output)
     *
     * Primary function out of reset: USB_SE0_VM
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A6
     */
    { SP_USB_SE0_VM, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: USB_RXD
     * SCM-A11 Reference P1A Wingboard Signal: USB_XRXD (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_XRXD (Atlas)
     * Selected Primary Function: USB_RXD (Input)
     *
     * Primary function out of reset: USB_RXD
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A7
     */
    { SP_USB_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_OVR
     * SCM-A11 Reference P1A Wingboard Signal: USB_XCVR_EN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USB_XCVR_EN (Atlas)
     * Selected Primary Function: GP_SP_A14 (Output)
     *
     * Primary function out of reset: UH2_OVR
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A14
     */
    { SP_UH2_OVR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_SVEN
     * SCM-A11 Reference P1A Wingboard Signal: VSIM_EN (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: VSIM_EN (Atlas)
     * Selected Primary Function: SIM1_SVEN (Output)
     *
     * Primary function out of reset: SIM1_SVEN
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SIM1_SVEN, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_PD
     * SCM-A11 Reference P1A Wingboard Signal: BAT_DETB (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: BAT_DETB (Atlas)
     * Selected Primary Function: SIM1_PD (Input)
     *
     * Primary function out of reset: SIM1_PD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C31
     */
    { SP_SIM1_PD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Main and secondary display backlight IOMUX settings.
 */
static struct iomux_initialization backlight_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: IPU_D3_SPL
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_MAIN_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_MAIN_BKL (Backlight)
     * Selected Primary Function: GP_AP_A30 (Output)
     *
     * Primary function out of reset: IPU_D3_SPL
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A30
     */
    { AP_IPU_D3_PS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_B17
     * SCM-A11 Reference P1A Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P2A Wingboard Signal: PWM_BKL (Linear Vibrator)
     * SCM-A11 Reference P2B Wingboard Signal: PWM_BKL (Linear Vibrator)
     * Selected Primary Function: AP_PWM (Output)
     *
     * Primary function out of reset: GP_AP_B17
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B17
     */
    { AP_GPIO_AP_B17, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C14
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CLI_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CLI_BKL (Backlight)
     * Selected Primary Function: GP_AP_C14 (Output)
         *
         * Primary function out of reset: GP_AP_C14
         * Out of Reset State: Input
         * Mux0 Function: GP_AP_C14
     */
    { AP_GPIO_AP_C14, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: SPI1_MISO (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     *
     * Primary function out of reset: SPI1_MISO
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A29
     */
    { SP_SPI1_MISO, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Bluetooth IOMUX settings.
 */
static struct iomux_initialization bluetooth_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: U1_TXD
     * SCM-A11 Reference P1A Wingboard Signal: BT_TX (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_TX (Bluetooth)
     * Selected Primary Function: U1_TXD (Output)
     *
     * Primary function out of reset: U1_TXD
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A7
     */
    { AP_U1_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_RXD
     * SCM-A11 Reference P1A Wingboard Signal: BT_RX (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RX (Bluetooth)
     * Selected Primary Function: U1_RXD (Input)
     *
     * Primary function out of reset: U1_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A8
     */
    { AP_U1_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_RTS_B
     * SCM-A11 Reference P1A Wingboard Signal: BT_RTS_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RTS_B (Bluetooth)
     * Selected Primary Function: U1_RTS_B (Input)
     *
     * Primary function out of reset: U1_RTS_B
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A9
     */
    { AP_U1_RTS_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_CTS_B
     * SCM-A11 Reference P1A Wingboard Signal: BT_CTS_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_CTS_B (Bluetooth)
     * Selected Primary Function: U1_CTS_B (Output)
     *
     * Primary function out of reset: U1_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A10
     */
    { AP_U1_CTS_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * SCM-A11 Reference P1A Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
         *
         * Primary function out of reset: GP_AP_C8
         * Out of Reset State: Input
         * Mux0 Function: GP_AP_C8
     */
    { AP_GPIO_AP_C8, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C12
     * SCM-A11 Reference P1A Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P2A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_RESET (WLAN)
     * Selected Primary Function: GP_AP_C12 (Output)
         *
         * Primary function out of reset: GP_AP_C12
         * Out of Reset State: Input
         * Mux0 Function: GP_AP_C12
     */
    { AP_GPIO_AP_C12, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C13
     * SCM-A11 Reference P1A Wingboard Signal: BT_WAKE_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_WAKE_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: BT_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C13 (Output)
     *
     * Primary function out of reset: GP_AP_C13
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C13
     */
    { AP_GPIO_AP_C13, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT1
     * SCM-A11 Reference P1A Wingboard Signal: BT_HOST_WAKE_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_HOST_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C19 (Input)
     * Selected Secondary Function: ED_INT1 (Input)
     *
     * Primary function out of reset: ED_INT1
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C19
     */
    { AP_ED_INT1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Secondary display IOMUX settings.
 */
static struct iomux_initialization cli_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CONTR
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CLI_SER_RS (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CLI_SER_RS (CLI)
     * Selected Primary Function: IPU_SER_RS (Output)
     *
     * Primary function out of reset: IPU_D3_CONTR
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A25
     */
    { AP_IPU_D3_CONTR, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D2_CS
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D2_CS_B (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D2_CS_B (CLI)
     * Selected Primary Function: IPU_D2_CS (Output)
     *
     * Primary function out of reset: IPU_D2_CS
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A28
     */
    { AP_IPU_D2_CS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CLS
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D3_CLS (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D3_CLS (CLI)
     * Selected Primary Function: IPU_SD_CLK (Output)
     *
     * Primary function out of reset: IPU_D3_CLS
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A31
     */
    { AP_IPU_D3_CLS, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_RD
     * SCM-A11 Reference P1A Wingboard Signal: IPU_RD_SDD (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_RD_SDD (CLI)
     * Selected Primary Function: IPU_SD_D_INOUT (Input/Output)
     *
     * Primary function out of reset: IPU_RD
     * Out of Reset State: High
     * Mux0 Function: GP_AP_B0
     */
    { AP_IPU_RD, OUTPUTCONFIG_FUNC3, INPUTCONFIG_FUNC3 },
    /*
     * SCM-A11 Package Pin Name: GP_AP_B18
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_RST_B (CLI)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_RST_B (CLI)
     * Selected Primary Function: GP_AP_B18 (Output)
     *
     * Primary function out of reset: GP_AP_B18
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B18
     */
    { AP_GPIO_AP_B18, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * CSI - camera sensor interface IOMUX settings.
 */
static struct iomux_initialization camera_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: CSI_D0
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CAM_RST_B (camera)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CAM_RST_B (camera)
     * Selected Primary Function: GP_AP_B24 (Output)
     *
     * Primary function out of reset: CSI_D0
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B24
     */
    { AP_CSI_D0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D1
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CAM_PD (camera)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CAM_PD (camera)
     * Selected Primary Function: GP_AP_B25 (Output)
     *
     * Primary function out of reset: CSI_D1
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B25
     */
    { AP_CSI_D1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D2
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(0) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(0) (camera)
     * Selected Primary Function: CSI_D2 (Input)
     *
     * Primary function out of reset: CSI_D2
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B26
     */
    { AP_CSI_D2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D3
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(1) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(1) (camera)
     * Selected Primary Function: CSI_D3 (Input)
     *
     * Primary function out of reset: CSI_D3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B27
     */
    { AP_CSI_D3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D4
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(2) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(2) (camera)
     * Selected Primary Function: CSI_D4 (Input)
     *
     * Primary function out of reset: CSI_D4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B28
     */
    { AP_CSI_D4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D5
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(3) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(3) (camera)
     * Selected Primary Function: CSI_D5 (Input)
     *
     * Primary function out of reset: CSI_D5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B29
     */
    { AP_CSI_D5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D6
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(4) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(4) (camera)
     * Selected Primary Function: CSI_D6 (Input)
     *
     * Primary function out of reset: CSI_D6
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B30
     */
    { AP_CSI_D6, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D7
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(5) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(5) (camera)
     * Selected Primary Function: CSI_D7 (Input)
     *
     * Primary function out of reset: CSI_D7
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B31
     */
    { AP_CSI_D7, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D8
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(6) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(6) (camera)
     * Selected Primary Function: CSI_D8 (Input)
     *
     * Primary function out of reset: CSI_D8
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C0
     */
    { AP_CSI_D8, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D9
     * SCM-A11 Reference P1A Wingboard Signal: CSI_D(7) (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_D(7) (camera)
     * Selected Primary Function: CSI_D9 (Input)
     *
     * Primary function out of reset: CSI_D9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C1
     */
    { AP_CSI_D9, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_MCLK
     * SCM-A11 Reference P1A Wingboard Signal: CSI_MCLK (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_MCLK (camera)
     * Selected Primary Function: CSI_MCLK (Output)
     *
     * Primary function out of reset: CSI_MCLK
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_C2
     */
    { AP_CSI_MCLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_VSYNC
     * SCM-A11 Reference P1A Wingboard Signal: CSI_VSYNC (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_VSYNC (camera)
     * Selected Primary Function: CSI_VSYNC (Input)
     *
     * Primary function out of reset: CSI_VSYNC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C3
     */
    { AP_CSI_VSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_HSYNC
     * SCM-A11 Reference P1A Wingboard Signal: CSI_HSYNC (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_HSYNC (camera)
     * Selected Primary Function: CSI_HSYNC (Input)
     *
     * Primary function out of reset: CSI_HSYNC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C4
     */
    { AP_CSI_HSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_PIXCLK
     * SCM-A11 Reference P1A Wingboard Signal: CSI_PIXCLK (camera)
     * SCM-A11 Reference P1D Wingboard Signal: CSI_PIXCLK (camera)
     * Selected Primary Function: CSI_PIXCLK (Input)
     *
     * Primary function out of reset: CSI_PIXCLK
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C5
     */
    { AP_CSI_PIXCLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Camera flash IOMUX settings.
 */
static struct iomux_initialization camera_flash_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * SCM-A11 Reference P1A Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P1D Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P2B Wingboard Signal: LIN_VIB_AMP_EN (Linear Vibrator)
     * Selected Primary Function: GP_SP_A10 (Output)
     *
     * Primary function out of reset: UH2_SUSPEND
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A10
     */
    { SP_UH2_SUSPEND, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SPI1_SS0
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_CAM_FLASH_STROBE (Camera Flash)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_CAM_FLASH_STROBE (Camera Flash)
     * Selected Primary Function: GP_SP_A30 (Output)
     *
     * Primary function out of reset: SPI1_SS0
     * Out of Reset State: Output
     * Mux0 Function: GP_SP_A30
     */
    { SP_SPI1_SS0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Main display IOMUX settings.
 */
static struct iomux_initialization display_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: IPU_BE1_LD17
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D0_VSYNC_LD17 (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D0_VSYNC_LD17 (Display)
     * Selected Primary Function: IPU_BE1_LD17 (Output)
     *
     * Primary function out of reset: IPU_LD17
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A20
     */
    { AP_IPU_LD17, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_VSYNC
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D3_VSYNC (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D3_VSYNC (Display)
     * Selected Primary Function: IPU_D3_VSYNC (Output)
     *
     * Primary function out of reset: IPU_D3_VSYNC
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A21
     */
    { AP_IPU_D3_VSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_HSYNC
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D3_HSYNC (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D3_HSYNC (Display)
     * Selected Primary Function: IPU_D3_HSYNC (Output)
     *
     * Primary function out of reset: IPU_D3_HSYNC
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A22
     */
    { AP_IPU_D3_HSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CLK
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D3_CLK (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D3_CLK (Display)
     * Selected Primary Function: IPU_D3_CLK (Output)
     *
     * Primary function out of reset: IPU_D3_CLK
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A23
     */
    { AP_IPU_D3_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_DRDY
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D3_DRDY (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D3_DRDY (Display)
     * Selected Primary Function: IPU_D3_DRDY (Output)
     *
     * Primary function out of reset: IPU_D3_DRDY
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A24
     */
    { AP_IPU_D3_DRDY, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D0_CS
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_SD (Display)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_SD (Display)
     * Selected Primary Function: GP_AP_A26 (Output)
     *
     * Primary function out of reset: IPU_D0_CS
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A26
     */
    { AP_IPU_D0_CS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_BE0_LD16
     * SCM-A11 Reference P1A Wingboard Signal: IPU_D1CS_LD16 (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_D1CS_LD16 (Display)
     * Selected Primary Function: IPU_BE0_LD16 (Output)
     *
     * Primary function out of reset: IPU_LD16
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A27
     */
    { AP_IPU_LD16, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_PAR_RS
     * SCM-A11 Reference P1A Wingboard Signal: IPU_PAR_RS (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_PAR_RS (Display)
     * SCM-A11 Reference P2B Wingboard Signal: SER_EN (Display)
     * Selected Primary Function: GP_AP_A29 (Output)
     *
     * Primary function out of reset: IPU_PAR_RS
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A29
     */
    { AP_IPU_PAR_RS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_WR
     * SCM-A11 Reference P1A Wingboard Signal: IPU_WR (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_WR (Display)
     * Selected Primary Function: GP_AP_B1 (Output)
     *
     * Primary function out of reset: IPU_WR
     * Out of Reset State: High
     * Mux0 Function: GP_AP_B1
     */
    { AP_IPU_WR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD0
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(0) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(0) (Display)
     * Selected Primary Function: IPU_LD0 (Output)
     *
     * Primary function out of reset: IPU_LD0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD1
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(1) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(1) (Display)
     * Selected Primary Function: IPU_LD1 (Output)
     *
     * Primary function out of reset: IPU_LD1
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD2
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(2) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(2) (Display)
     * Selected Primary Function: IPU_LD2 (Output)
     *
     * Primary function out of reset: IPU_LD2
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD3
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(3) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(3) (Display)
     * Selected Primary Function: IPU_LD3 (Output)
     *
     * Primary function out of reset: IPU_LD3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B2
     */
    { AP_IPU_LD3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD4
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(4) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(4) (Display)
     * Selected Primary Function: IPU_LD4 (Output)
     *
     * Primary function out of reset: IPU_LD4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B3
     */
    { AP_IPU_LD4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD5
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(5) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(5) (Display)
     * Selected Primary Function: IPU_LD5 (Output)
     *
     * Primary function out of reset: IPU_LD5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B4
     */
    { AP_IPU_LD5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD6
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(6) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(6) (Display)
     * Selected Primary Function: IPU_LD6 (Output)
     *
     * Primary function out of reset: IPU_LD6
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B5
     */
    { AP_IPU_LD6, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD7
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(7) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(7) (Display)
     * Selected Primary Function: IPU_LD7 (Output)
     *
     * Primary function out of reset: IPU_LD7
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B6
     */
    { AP_IPU_LD7, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD8
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(8) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(8) (Display)
     * Selected Primary Function: IPU_LD8 (Output)
     *
     * Primary function out of reset: IPU_LD8
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B7
     */
    { AP_IPU_LD8, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD9
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(9) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(9) (Display)
     * Selected Primary Function: IPU_LD9 (Output)
     *
     * Primary function out of reset: IPU_LD9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B8
     */
    { AP_IPU_LD9, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD10
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(10) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(10) (Display)
     * Selected Primary Function: IPU_LD10 (Output)
     *
     * Primary function out of reset: IPU_LD10
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B9
     */
    { AP_IPU_LD10, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD11
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(11) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(11) (Display)
     * Selected Primary Function: IPU_LD11 (Output)
     *
     * Primary function out of reset: IPU_LD11
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B10
     */
    { AP_IPU_LD11, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD12
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(12) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(12) (Display)
     * Selected Primary Function: IPU_LD12 (Output)
     *
     * Primary function out of reset: IPU_LD12
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B11
     */
    { AP_IPU_LD12, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD13
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(13) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(13) (Display)
     * Selected Primary Function: IPU_LD13 (Output)
     *
     * Primary function out of reset: IPU_LD13
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B12
     */
    { AP_IPU_LD13, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD14
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(14) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(14) (Display)
     * Selected Primary Function: IPU_LD14 (Output)
     *
     * Primary function out of reset: IPU_LD14
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B13
     */
    { AP_IPU_LD14, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD15
     * SCM-A11 Reference P1A Wingboard Signal: IPU_LD(15) (Display)
     * SCM-A11 Reference P1D Wingboard Signal: IPU_LD(15) (Display)
     * Selected Primary Function: IPU_LD15 (Output)
     *
     * Primary function out of reset: IPU_LD15
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B14
     */
    { AP_IPU_LD15, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U2_RI_B
     * SCM-A11 Reference P1A Wingboard Signal: GPIO_DISP_CM/DAI_CLK (Display)
     * SCM-A11 Reference P1D Wingboard Signal: GPIO_DISP_CM/DAI_CLK (Display)
     * Selected Primary Function: GP_AP_C27 (Output)
     *
     * Primary function out of reset: U2_RI_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C27
     */
    { AP_U2_RI_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Ethernet IOMUX settings.
 */
static struct iomux_initialization ethernet_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: ED_INT3
     * SCM-A11 Reference P1A Wingboard Signal: CPLD_IO_ENET_INT (Ethernet)
     * SCM-A11 Reference P1D Wingboard Signal: CPLD_IO_ENET_INT/(hall effect)
     *                                          (Ethernet)
     * Selected Primary Function: GP_AP_C21 (Input)
     * Selected Secondary Function: ED_INT3 (Input)
     *
     * Primary function out of reset: ED_INT3
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C21
     */
    { AP_ED_INT3, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * FM Radio IOMUX settings.
 */
static struct iomux_initialization fm_radio_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: GP_AP_C11
     * SCM-A11 Reference P1A Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: FM_INTERRUPT (FM Radio)
     * Selected Primary Function: GP_AP_C11 (Input)
     *
     * Primary function out of reset: GP_AP_C11
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C11
     */
    { AP_GPIO_AP_C11, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: VFUSE_SELECT (Misc)
     * SCM-A11 Reference P3A Wingboard Signal: VFUSE_SELECT/FM_RESET
     *                                                      (Misc/FM Radio)
     * Selected Primary Function: GP_SP_A9 (Output)
     *
     * Primary function out of reset: UH2_SPEED
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A9
     */
    { SP_UH2_SPEED, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * GPS iomux settings.
 */
static struct iomux_initialization gps_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: U3_TXD
     * SCM-A11 Reference P1A Wingboard Signal: GPS_U3_TX (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_U3_TX (GPS)
     * Selected Primary Function: U3_TXD (Output)
     *
     * Primary function out of reset: U3_TXD
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A0
     */
    { SP_U3_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U3_RXD
     * SCM-A11 Reference P1A Wingboard Signal: GPS_U3_RX (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_U3_RX (GPS)
     * Selected Primary Function: U3_RXD (Input)
     *
     * Primary function out of reset: U3_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A1
     */
    { SP_U3_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_U3_RTS_B (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P3C Wingboard Signal: GPS_RESET (GPS)/LIN_VIB_AMP_EN
     *                                           (Linear Vibrator)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Primary function out of reset: U3_RTS_B
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A2
     */
    { SP_U3_RTS_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * I2C IOMUX settings.
 */
static struct iomux_initialization i2c_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: I2CLK
     * SCM-A11 Reference P1A Wingboard Signal: I2C_CLK (camera)
     * SCM-A11 Reference P1D Wingboard Signal: I2C_CLK (camera)
     * Selected Primary Function: I2CLK (Input/Output)
     *
     * Primary function out of reset: I2CLK
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C6
     */
    { AP_I2CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: I2DAT
     * SCM-A11 Reference P1A Wingboard Signal: I2C_DAT (camera)
     * SCM-A11 Reference P1D Wingboard Signal: I2C_DAT (camera)
     * Selected Primary Function: I2DAT (Input/Output)
     *
     * Primary function out of reset: I2DAT
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C7
     */
    { AP_I2DAT, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Keypad IOMUX settings.
 */
static struct iomux_initialization keypad_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: KPROW4
     * SCM-A11 Reference P1A Wingboard Signal: KPP_ROW(4) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_ROW(4) (Keypad)
     * Selected Primary Function: KPROW4 (Input/Output)
     *
     * Primary function out of reset: KPROW4
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL3
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(3) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(3) (Keypad)
     * Selected Primary Function: KPCOL3 (Input/Output)
     *
     * Primary function out of reset: KPCOL3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B19
     */
    { AP_KPCOL3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL4
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(4) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(4) (Keypad)
     * Selected Primary Function: KPCOL4 (Input/Output)
     *
     * Primary function out of reset: KPCOL4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B20
     */
    { AP_KPCOL4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL5
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(5) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(5) (Keypad)
     * Selected Primary Function: KPCOL5 (Input/Output)
     *
     * Primary function out of reset: KPCOL5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B21
     */
    { AP_KPCOL5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW0
     * SCM-A11 Reference P1A Wingboard Signal: KPP_ROW(0) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_ROW(0) (Keypad)
     * Selected Primary Function: KPROW0 (Input/Output)
     *
     * Primary function out of reset: KPROW0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW1
     * SCM-A11 Reference P1A Wingboard Signal: KPP_ROW(1) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_ROW(1) (Keypad)
     * Selected Primary Function: KPROW1 (Input/Output)
     *
     * Primary function out of reset: KPROW1
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B15
     */
    { AP_KPROW1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW2
     * SCM-A11 Reference P1A Wingboard Signal: KPP_ROW(2) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_ROW(2) (Keypad)
     * Selected Primary Function: KPROW2 (Input/Output)
     *
     * Primary function out of reset: KPROW2
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW3
     * SCM-A11 Reference P1A Wingboard Signal: KPP_ROW(3) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_ROW(3) (Keypad)
     * Selected Primary Function: KPROW3 (Input/Output)
     *
     * Primary function out of reset: KPROW3
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL0
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(0) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(0) (Keypad)
     * Selected Primary Function: KPCOL0 (Input/Output)
     *
     * Primary function out of reset: KPCOL0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPCOL0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL1
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(1) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(1) (Keypad)
     * Selected Primary Function: KPCOL1 (Input/Output)
     *
     * Primary function out of reset: KPCOL1
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPCOL1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL2
     * SCM-A11 Reference P1A Wingboard Signal: KPP_COL(2) (Keypad)
     * SCM-A11 Reference P1D Wingboard Signal: KPP_COL(2) (Keypad)
     * Selected Primary Function: KPCOL2 (Input/Output)
     *
     * Primary function out of reset: KPCOL2
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPCOL2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Multi-media card IOMUX settings.
 */
static struct iomux_initialization mmc_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: UH2_RXDM
     * SCM-A11 Reference P1A Wingboard Signal: TF_ENABLE (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: TF_ENABLE (MMC)
     * SCM-A11 Reference P2B Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * Selected Primary Function: GP_SP_A13 (Output)
     *
     * Primary function out of reset: UH2_RXDM
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A13
     */
    { SP_UH2_RXDM, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT0
     * SCM-A11 Reference P1A Wingboard Signal: SD1_DATA(0) (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_DATA(0) (MMC)
     * Selected Primary Function: SD1_DAT0 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT0
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A16
     */
    { SP_SD1_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT1
     * SCM-A11 Reference P1A Wingboard Signal: SD1_DATA(1) (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_DATA(1) (MMC)
     * Selected Primary Function: SD1_DAT1 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT1
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A17
     */
    { SP_SD1_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT2
     * SCM-A11 Reference P1A Wingboard Signal: SD1_DATA(2) (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_DATA(2) (MMC)
     * Selected Primary Function: SD1_DAT2 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT2
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A18
     */
    { SP_SD1_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT3
     * SCM-A11 Reference P1A Wingboard Signal: SD1_DATA(3) (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_DATA(3) (MMC)
     * Selected Primary Function: SD1_DAT3 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT3
     * Out of Reset State: Hi-Z
     * Mux0 Function: GP_SP_A19
     */
    { SP_SD1_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_CMD
     * SCM-A11 Reference P1A Wingboard Signal: SD1_CMD (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_CMD (MMC)
     * Selected Primary Function: SD1_CMD (Input/Output)
     *
     * Primary function out of reset: SD1_CMD
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A20
     */
    { SP_SD1_CMD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: SD1_CLK (MMC)
     * SCM-A11 Reference P1D Wingboard Signal: SD1_CLK (MMC)
     * Selected Primary Function: SD1_CLK (Output)
     *
     * Primary function out of reset: SD1_CLK
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A21
     */
    { SP_SD1_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Miscellaneous IOMUX settings.
 */
static struct iomux_initialization misc_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: OWDAT
     * SCM-A11 Reference P1A Wingboard Signal: BATT_DAT (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: BATT_DAT (Misc)
     * Selected Primary Function: OWDAT (Input/Output)
     *
     * Primary function out of reset: OWDAT
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A19
     */
    { AP_OWDAT, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C9
     * SCM-A11 Reference P1A Wingboard Signal: BOOT_RS232_USB (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: 10K Pulldown (Misc)
     * Selected Primary Function: GP_AP_C9 (Output)
     *
     * Primary function out of reset: GP_AP_C9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C9
     */
    { AP_GPIO_AP_C9, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT2
     * SCM-A11 Reference P1A Wingboard Signal: WDOG_BP (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: WDOG_BP (Misc)
     * Selected Primary Function: GP_AP_C20 (Input)
     * Selected Secondary Function: ED_INT2 (Input)
     *
     * Primary function out of reset: ED_INT2
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C20
     */
    { AP_ED_INT2, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: U2_DSR_B
     * SCM-A11 Reference P1A Wingboard Signal: DAI_TX (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: DAI_TX (Misc)
     * Selected Primary Function: AD4_TXD (Output)
     *
     * Primary function out of reset: U2_DSR_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C26
     */
    { AP_U2_DSR_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U2_CTS_B
     * SCM-A11 Reference P1A Wingboard Signal: DAI_FS (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: DAI_FS (Misc)
     * Selected Primary Function: AD4_TXFS (Input/Output)
     *
     * Primary function out of reset: U2_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C28
     */
    { AP_U2_CTS_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_FUNC2 },
    /*
     * SCM-A11 Package Pin Name: U2_DTR_B
     * SCM-A11 Reference P1A Wingboard Signal: DAI_RX (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: DAI_RX (Misc)
     * Selected Primary Function: AD4_RXD (Input)
     *
     * Primary function out of reset: U2_DTR_B
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C29
     */
    { AP_U2_DTR_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_TXDP
     * SCM-A11 Reference P1A Wingboard Signal: IO_LED_DBG (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: IO_LED_DBG (Misc)
     * SCM-A11 Reference P2B Wingboard Signal: USB_HS_SWITCH (Wing Connector)
     * Selected Primary Function: GP_SP_A11 (Output)
     *
     * Primary function out of reset: UH2_TXDP
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A11
     */
    { SP_UH2_TXDP, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Morphing IOMUX settings.
 */
static struct iomux_initialization morphing_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_RXDP (Saipan Connector)
     * SCM-A11 Reference P3A Wingboard Signal: UH2_RXDP (INTERRUPT) (Morphing)
     * SCM-A11 Reference P3C Wingboard Signal: TNLC_KCHG_INT (Morphing)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Primary function out of reset: UH2_RXDP
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A12
     */
    { SP_UH2_RXDP, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_PWR
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_PWR (Saipan Connector)
     * SCM-A11 Reference P3A Wingboard Signal: UH2_RXDM (RESET) (Morphing)
     * Selected Primary Function: GP_SP_A15 (Output)
     *
     * Primary function out of reset: UH2_PWR
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A15
     */
    { SP_UH2_PWR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SPI1_SS1
     * SCM-A11 Reference P1A Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: SPI1_SS1 (2nd MORPH RESET)
     *                                                  (Morphing)
     * SCM-A11 Reference P3C Wingboard Signal: FSS_HYST (Morphing)
     * Selected Primary Function: GP_SP_A31 (Output)
     *
     * Primary function out of reset: SPI1_SS1
     * Out of Reset State: Output
     * Mux0 Function: GP_SP_A31
     */
    { SP_SPI1_SS1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: OMEGA_INTERRUPT (Omega Wheel)
     * SCM-A11 Reference P3C Wingboard Signal: TNLC_RCHG (Morphing)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Primary function out of reset: UH2_TXOE_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A8
     */
    { SP_UH2_TXOE_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * NAND IOMUX settings.
 */
static struct iomux_initialization nand_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: CLE
     * SCM-A11 Reference P1A Wingboard Signal: NAND_CLE (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_CLE (Memory)
     * Selected Primary Function: CLE (Output)
     *
     * Primary function out of reset: CLE
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A0
     */
    { AP_CLE, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ALE
     * SCM-A11 Reference P1A Wingboard Signal: NAND_ALE (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_ALE (Memory)
     * Selected Primary Function: ALE (Output)
     *
     * Primary function out of reset: ALE
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A1
     */
    { AP_ALE, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CE_B
     * SCM-A11 Reference P1A Wingboard Signal: NAND_CS_B (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_CS_B (Memory)
     * Selected Primary Function: CE_B (Output)
     *
     * Primary function out of reset: CE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A2
     */
    { AP_CE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: RE_B
     * SCM-A11 Reference P1A Wingboard Signal: NAND_RE_B (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_RE_B (Memory)
     * Selected Primary Function: RE_B (Output)
     *
     * Primary function out of reset: RE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A3
     */
    { AP_RE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: WE_B
     * SCM-A11 Reference P1A Wingboard Signal: NAND_WE_B (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_WE_B (Memory)
     * Selected Primary Function: WE_B (Output)
     *
     * Primary function out of reset: WE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A4
     */
    { AP_WE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: WP_B
     * SCM-A11 Reference P1A Wingboard Signal: NAND_WP_B (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_WP_B (Memory)
     * Selected Primary Function: WP_B (Output)
     *
     * Primary function out of reset: WP_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A5
     */
    { AP_WP_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: BSY_B
     * SCM-A11 Reference P1A Wingboard Signal: NAND_R_B (Memory)
     * SCM-A11 Reference P1D Wingboard Signal: NAND_R_B (Memory)
     * Selected Primary Function: BSY_B (Input)
     *
     * Primary function out of reset: BSY_B
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A6
     */
    { AP_BSY_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * IOMUX settings for unconnected pins.
 */
static struct iomux_initialization nc_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: SPI1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: EL_NUM_EN (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     *
     * Primary function out of reset: SPI1_CLK
     * Out of Reset State: High-Z
     * Mux0 Function: GP_SP_A27
     */
    { SP_SPI1_CLK, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_TXDM
     * SCM-A11 Reference P1A Wingboard Signal: TP (NC)
     * SCM-A11 Reference P1D Wingboard Signal: TP (NC)
     * Selected Primary Function: Unassigned (Input)
     *
     * Primary function out of reset: UH2_TXDM
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_UH2_TXDM, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_RXD
     * SCM-A11 Reference P1A Wingboard Signal: TP (NC)
     * SCM-A11 Reference P1D Wingboard Signal: TP (NC)
     * Selected Primary Function: Unassigned (Input)
     *
     * Primary function out of reset: UH2_RXD
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { SP_UH2_RXD, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * IOMUX settings for SIM controller.
 */
static struct iomux_initialization sim_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: SIM1_RST_B
     * SCM-A11 Reference P1A Wingboard Signal: SIM_RST (SIM)
     * SCM-A11 Reference P1D Wingboard Signal: SIM_RST (SIM)
     * Selected Primary Function: SIM1_RST_B (Output)
     *
     * Primary function out of reset: SIM1_RST_B
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_C30
     */
    { SP_SIM1_RST_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: SIM_CLK (SIM)
     * SCM-A11 Reference P1D Wingboard Signal: SIM_CLK (SIM)
     * Selected Primary Function: SIM1_CLK (Output)
     *
     * Primary function out of reset: SIM1_CLK
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SIM1_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_TRXD
     * SCM-A11 Reference P1A Wingboard Signal: SIM_IO (SIM)
     * SCM-A11 Reference P1D Wingboard Signal: SIM_IO (SIM)
     * Selected Primary Function: SIM1_TRXD (Input/Output)
     *
     * Primary function out of reset: SIM1_TRXD
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SIM1_TRXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * IOMUX settings for wireless LAN controller.
 */
static struct iomux_initialization wlan_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: ED_INT0
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_HOST_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_HOST_WAKE_B (WLAN)
     * Selected Primary Function: GP_AP_C18 (Input)
     * Selected Secondary Function: ED_INT0 (Input)
     *
     * Primary function out of reset: ED_INT0
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C18
     */
    { AP_ED_INT0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: ED_INT4
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DET_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DET_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: TF_DET (MMC)
     * Selected Primary Function: GP_AP_C22 (Input)
     * Selected Secondary Function: ED_INT4 (Input)
     *
     * Primary function out of reset: ED_INT4
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C22
     */
    { AP_ED_INT4, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT0
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(0) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(0) (WLAN)
     * Selected Primary Function: SD2_DAT0 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT0
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A22
     */
    { SP_SD2_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT1
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(1) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(1) (WLAN)
     * Selected Primary Function: SD2_DAT1 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT1
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A23
     */
    { SP_SD2_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT2
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(2) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(2) (WLAN)
     * Selected Primary Function: SD2_DAT2 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT2
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A24
     */
    { SP_SD2_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT3
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(3) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(3) (WLAN)
     * Selected Primary Function: SD2_DAT3 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT3
     * Out of Reset State: Hi-Z
     * Mux0 Function: GP_SP_A25
     */
    { SP_SD2_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: GP_SP_A26
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLK_EN_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLK_EN_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * Selected Primary Function: GP_SP_A26 (Input)
     *
     * Primary function out of reset: GP_SP_A26
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A26
     */
    { SP_GPIO_Shared26, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SD2_CMD
     * SCM-A11 Reference P1A Wingboard Signal: SD2_CMD (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_CMD (WLAN)
     * Selected Primary Function: SD2_CMD (Input/Output)
     *
     * Primary function out of reset: SD2_CMD
     * Out of Reset State: High
     * Mux0 Function: (not defined)
     */
    { SP_SD2_CMD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_CLK
     * SCM-A11 Reference P1A Wingboard Signal: SD2_CLK (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_CLK (WLAN)
     * Selected Primary Function: SD2_CLK (Output)
     *
     * Primary function out of reset: SD2_CLK
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SD2_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * IOMUX settings for high speed USB and any other wing connector devices.
 */
static struct iomux_initialization wing_connector_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: GP_AP_B22
     * SCM-A11 Reference P1A Wingboard Signal: GP_AP_B22 (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: GP_AP_B22 (Wing Connector)
     * Selected Primary Function: GP_AP_B22 (Input)
     *
     * This setting varies from the pin list. The Func2 mux setting for
     * this pin is DMAREQ0.
     *
     * Primary function out of reset: GP_AP_B22
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B22
     */
    { AP_GPIO_AP_B22, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C10
     * SCM-A11 Reference P1A Wingboard Signal: USB_HS_DMA_ACK (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: USB_HS_DMA_ACK (Wing Connector)
     * Selected Primary Function: GP_AP_C10 (Input)
     *
     * Primary function out of reset: GP_AP_C10
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C10
     */
    { AP_GPIO_AP_C10, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT6
     * SCM-A11 Reference P1A Wingboard Signal: USB_HS_INT (Wing Connector)
     * SCM-A11 Reference P1D Wingboard Signal: USB_HS_INT (Wing Connector)
     * Selected Primary Function: GP_AP_C24 (Input)
     * Selected Secondary Function: ED_INT6 (Input)
     *
     * Primary function out of reset: ED_INT6
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C24
     */
    { AP_ED_INT6, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SPI1_MOSI
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: SPI1_MOSI (Wing Connector)
     * SCM-A11 Reference P2A Wingboard Signal: USB_HS_RESET (Wing Connector)
     * Selected Primary Function: GP_SP_A28 (Output)
     *
     * Primary function out of reset: SPI1_MOSI
     * Out of Reset State: Output
     * Mux0 Function: GP_SP_A28
     */
    { SP_SPI1_MOSI, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};


/**
 * Global IOMUX settings
 */
static struct iomux_initialization *initial_iomux_settings[] __initdata = {
    atlas_iomux_settings,
    backlight_iomux_settings,
    bluetooth_iomux_settings,
    cli_iomux_settings,
    camera_iomux_settings,
    camera_flash_iomux_settings,
    display_iomux_settings,
    ethernet_iomux_settings,
    fm_radio_iomux_settings,
    gps_iomux_settings,
    i2c_iomux_settings,
    keypad_iomux_settings,
    mmc_iomux_settings,
    misc_iomux_settings,
    morphing_iomux_settings,
    nand_iomux_settings,
    nc_iomux_settings,
    sim_iomux_settings,
    wlan_iomux_settings,
    wing_connector_iomux_settings,
    /* list terminator */
    NULL
};


/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by fixup_scma11ref()
 * during system startup. This function is board specific.
 */
void __init scma11ref_gpio_init(void)
{
    int i, j;

    /* set iomux pad registers to the prescribed state */
    for(i = IOMUX_PAD_SETTING_START; i <= IOMUX_PAD_SETTING_STOP; i++) {
        gpio_tracemsg("Setting pad register 0x%08x to: 0x%08x",
                iomux_pad_register_settings[i].grp,
                iomux_pad_register_settings[i].config);


        iomux_set_pad(iomux_pad_register_settings[i].grp,
                iomux_pad_register_settings[i].config);
    }

#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P1A) || (boardrev() == BOARDREV_UNKNOWN) ) {
        gpio_signal_fixup_p0c();
    } else if( boardrev() < BOARDREV_P2AW) {
        gpio_signal_fixup_p1a();
    } else if( boardrev() < BOARDREV_P2BW) {
        gpio_signal_fixup_p2aw();
    } else if( boardrev() < BOARDREV_P3AW) {
        gpio_signal_fixup_p2bw();
    } else if( boardrev() < BOARDREV_P3CW) {
        gpio_signal_fixup_p3aw();
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

            /* 
             * We set the data for a signal and then configure the direction
             * of the signal.  Section 52.3.1.1 (GPIO Data Register) of
             * the SCM-A11 DTS indicates that this is a reasonable thing to do.
             */
            if(initial_gpio_settings[i].data != GPIO_DATA_INVALID) {
                gpio_set_data(initial_gpio_settings[i].port,
                        initial_gpio_settings[i].sig_no,
                        initial_gpio_settings[i].data);
            }

            gpio_config(initial_gpio_settings[i].port,
                    initial_gpio_settings[i].sig_no,
                    initial_gpio_settings[i].out,
                    GPIO_INT_NONE); /* setup interrupts later */
        }
    }
    
    /* configure IOMUX settings to their prescribed initial state */
    for(i = 0; initial_iomux_settings[i] != NULL; i++) {
        for(j = 0; initial_iomux_settings[i][j].pin != IOMUX_INVALID_PIN; j++) {
            gpio_tracemsg("IOMUX pin: 0x%08x output: 0x%02x input: 0x%02x",
                    initial_iomux_settings[i][j].pin,
                    initial_iomux_settings[i][j].output_config,
                    initial_iomux_settings[i][j].input_config);

            iomux_config_mux(initial_iomux_settings[i][j].pin,
                    initial_iomux_settings[i][j].output_config,
                    initial_iomux_settings[i][j].input_config);
        }           
    }               

#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P1A) || (boardrev() == BOARDREV_UNKNOWN) ) {
        iomux_setting_fixup_p0c();
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /* disable UART1 for Bluetooth current drain improvement */
    gpio_bluetooth_power_set_data(0);
}


#ifdef CONFIG_MOT_FEAT_BRDREV
/**
 * Adjust GPIO settings to reflect those needed to support P0C+ and
 * earlier wingboards.
 */
void __init gpio_signal_fixup_p0c(void)
{
    /*
     * SCM-A11 Package Pin Name: GP_AP_B17
     * SCM-A11 Reference P1A Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: PWM_BKL (Backlight)
     * Selected Primary Function: GP_AP_B17 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_PWM_BKL].port     = GPIO_AP_B_PORT;
    initial_gpio_settings[GPIO_SIGNAL_PWM_BKL].sig_no   = 17;
    initial_gpio_settings[GPIO_SIGNAL_PWM_BKL].out      = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_PWM_BKL].data     = GPIO_DATA_INVALID;

    /*
     * SCM-A11 Package Pin Name: U2_DTR_B
     * SCM-A11 Reference P1A Wingboard Signal: DAI_RX (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: DAI_RX (Misc)
     * Selected Primary Function: AD4_RXD (Input)
     *
     * This must be IOMUXed to GPIO mode before turning on the display. See
     * the function iomux_setting_fixup_p0c().
     */
    initial_gpio_settings[GPIO_SIGNAL_DISP_SD].port     = GPIO_AP_C_PORT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_SD].sig_no   = 29;
    initial_gpio_settings[GPIO_SIGNAL_DISP_SD].out      = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_DISP_SD].data     = GPIO_DATA_INVALID;
}


/**
 * Adjust the GPIO settings to reflect the P1 series of wing boards.
 */
void __init gpio_signal_fixup_p1a(void)
{
    /* revert to P2A wingboard state first */
    gpio_signal_fixup_p2aw();
    
    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * SCM-A11 Reference P1A Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P1D Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P2B Wingboard Signal: LIN_VIB_AMP_EN (Linear Vibrator)
     * Selected Primary Function: GP_SP_A10 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].port  = GPIO_INVALID_PORT;
    
    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Power off GPS at boot to prevent conflict with UART3.
     */
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].sig_no = 8;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_GPS_RESET].data   = GPIO_DATA_LOW;
    
    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * SCM-A11 Reference P1A Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_RESET_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
     *
     * Bring Bluetooth out of reset at boot. Power it down using BT_REG_CTL
     * signal (GPIO_SIGNAL_BT_POWER).
     */
    initial_gpio_settings[GPIO_SIGNAL_BT_RESET_B].port   = GPIO_AP_C_PORT;
    initial_gpio_settings[GPIO_SIGNAL_BT_RESET_B].sig_no = 8;
    initial_gpio_settings[GPIO_SIGNAL_BT_RESET_B].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_BT_RESET_B].data   = GPIO_DATA_HIGH;

    /*
     * SCM-A11 Package Pin Name: GP_AP_C12
     * SCM-A11 Reference P1A Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_REG_CTL (Bluetooth)
     * SCM-A11 Reference P2A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: WLAN_RESET (WLAN)
     * Selected Primary Function: GP_AP_C12 (Output)
     *
     * Power off Bluetooth at boot.
     */
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].port   = GPIO_AP_C_PORT;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].sig_no = 12;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_BT_POWER].data   = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_RXDP (Saipan Connector)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Power off WLAN at boot.
     */
    initial_gpio_settings[GPIO_SIGNAL_WLAN_CLIENT_WAKE_B].port
        = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_CLIENT_WAKE_B].sig_no = 12;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_CLIENT_WAKE_B].out
        = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_CLIENT_WAKE_B].data
        = GPIO_DATA_HIGH;

    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * SCM-A11 Reference P1A Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P1D Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P2B Wingboard Signal: LIN_VIB_AMP_EN (Linear Vibrator)
     * Selected Primary Function: GP_SP_A10 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_CAM_TORCH_EN].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_CAM_TORCH_EN].sig_no = 10;
    initial_gpio_settings[GPIO_SIGNAL_CAM_TORCH_EN].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_CAM_TORCH_EN].data   = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: UH2_PWR
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_PWR_DWN_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_PWR (Saipan Connector)
     * Selected Primary Function: GP_SP_A15 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_WLAN_PWR_DWN_B].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_PWR_DWN_B].sig_no = 15;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_PWR_DWN_B].out    = GPIO_GDIR_INPUT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_PWR_DWN_B].data
        = GPIO_DATA_INVALID;

    /*
     * SCM-A11 Package Pin Name: SPI1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: EL_NUM_EN (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_EL_NUM_EN].port = GPIO_INVALID_PORT;

    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: SPI1_MISO (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_EL_NAV_EN].port = GPIO_INVALID_PORT;

    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: VFUSE_SELECT (Misc)
     * Selected Primary Function: GP_SP_A9 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].sig_no = 9;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].data   = GPIO_DATA_LOW;
}


/**
 * Adjust GPIO settings to reflect P3A Wingboard configuration.
 */
void __init gpio_signal_fixup_p3aw(void)
{
    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * SCM-A11 Reference P1A Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P1D Wingboard Signal: GPS_RESET (GPS)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: OMEGA_INTERRUPT (Omega Wheel)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Array index: 34  GPIO_SIGNAL_TNLC_KCHG_INT
     */
    initial_gpio_settings[GPIO_SIGNAL_TNLC_KCHG_INT].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_KCHG_INT].sig_no = 8;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_KCHG_INT].out    = GPIO_GDIR_INPUT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_KCHG_INT].data   = GPIO_DATA_INVALID;

    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_CLIENT_WAKE_B (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: UH2_RXDP (Saipan Connector)
     * SCM-A11 Reference P3A Wingboard Signal: UH2_RXDP (INTERRUPT) (Morphing)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Array index: 37  GPIO_SIGNAL_TNLC_RCHG
     */
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RCHG].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RCHG].sig_no = 12;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RCHG].out    = GPIO_GDIR_INPUT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RCHG].data   = GPIO_DATA_INVALID;

    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * SCM-A11 Reference P1A Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P1D Wingboard Signal: CAM_TORCH_EN (Camera Flash)
     * SCM-A11 Reference P2B Wingboard Signal: LIN_VIB_AMP_EN (Linear Vibrator)
     * Selected Primary Function: GP_SP_A10 (Output)
     *
     * Array index: 29  GPIO_SIGNAL_LIN_VIB_AMP_EN
     *
     * Set high to enable vibrator.
     */
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].sig_no = 10;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].data   = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: SPI1_SS1
     * SCM-A11 Reference P1A Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P1D Wingboard Signal: USER_OFF (Atlas)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: SPI1_SS1 (2nd MORPH RESET)
     *                                                  (Morphing)
     * Selected Primary Function: GP_SP_A31 (Output)
     *
     * Array index: 35  GPIO_SIGNAL_TNLC_RESET
     */
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RESET].port      = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RESET].sig_no    = 31;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RESET].out       = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RESET].data      = GPIO_DATA_LOW;

    /* mark signals first used on P3C wingboard as invalid */
    initial_gpio_settings[GPIO_SIGNAL_UI_IC_DBG].port   = GPIO_INVALID_PORT;
    initial_gpio_settings[GPIO_SIGNAL_FSS_HYST].port    = GPIO_INVALID_PORT;
    initial_gpio_settings[GPIO_SIGNAL_SER_RST_B].port   = GPIO_INVALID_PORT;
}


/**
 * Adjust GPIO settings to reflect P2B Wingboard configuration.
 */
void __init gpio_signal_fixup_p2bw(void)
{
    /* revert table to P3A wingboard settings first */
    gpio_signal_fixup_p3aw();

    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: VFUSE_SELECT (Misc)
     * SCM-A11 Reference P3A Wingboard Signal: FM_RESET (FM Radio)
     * Selected Primary Function: GP_SP_A9 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_FM_RESET].port    = GPIO_INVALID_PORT;

    /*
     * SCM-A11 Package Pin Name: GP_AP_C11
     * SCM-A11 Reference P1A Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P1D Wingboard Signal: BT_CLK_EN_B (Bluetooth)
     * SCM-A11 Reference P2B Wingboard Signal: NC (NC)
     * SCM-A11 Reference P3A Wingboard Signal: FM_INTERRUPT (FM Radio)
     * Selected Primary Function: GP_AP_C11 (Input)
     */
    initial_gpio_settings[GPIO_SIGNAL_FM_INTERRUPT].port    = GPIO_INVALID_PORT;

    /* p2b wingboard doesn't support morphing */
    initial_gpio_settings[GPIO_SIGNAL_TNLC_KCHG_INT].port   = GPIO_INVALID_PORT;
    initial_gpio_settings[GPIO_SIGNAL_CAP_RESET].port       = GPIO_INVALID_PORT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RESET].port      = GPIO_INVALID_PORT;
    initial_gpio_settings[GPIO_SIGNAL_TNLC_RCHG].port       = GPIO_INVALID_PORT;
    
    /*
     * SCM-A11 Package Pin Name: SPI1_CLK
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: EL_NUM_EN (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_EL_NUM_EN].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_EL_NUM_EN].sig_no = 27;
    initial_gpio_settings[GPIO_SIGNAL_EL_NUM_EN].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_EL_NUM_EN].data   = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * SCM-A11 Reference P1A Wingboard Signal: NC (NC)
     * SCM-A11 Reference P1D Wingboard Signal: NC (NC)
     * SCM-A11 Reference P2B Wingboard Signal: SPI1_MISO (Backlight)
     * SCM-A11 Reference P3A Wingboard Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_EL_NAV_EN].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_EL_NAV_EN].sig_no = 29;
    initial_gpio_settings[GPIO_SIGNAL_EL_NAV_EN].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_EL_NAV_EN].data   = GPIO_DATA_LOW;

    /* p2b wingboard doesn't have EL_EN signal */
    initial_gpio_settings[GPIO_SIGNAL_EL_EN].port       = GPIO_INVALID_PORT;
}


/**
 * Adjust GPIO settings to reflect P2A Wingboard configuration.
 */
void __init gpio_signal_fixup_p2aw(void)
{
    /* revert to p2bw state before reverting to p2aw */
    gpio_signal_fixup_p2bw();

    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * SCM-A11 Reference P1A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2A Wingboard Signal: WLAN_RESET (WLAN)
     * SCM-A11 Reference P2B Wingboard Signal: VFUSE_SELECT (Misc)
     * Selected Primary Function: GP_SP_A9 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].sig_no = 9;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_WLAN_RESET].data   = GPIO_DATA_LOW;
}

/**
 * Adjust IOMUX settings to reflect those needed to support P0C+ and
 * earlier wingboards.
 */
void __init iomux_setting_fixup_p0c(void)
{
    /*
     * SCM-A11 Package Pin Name: GP_AP_B17
     * SCM-A11 Reference P1A Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P1D Wingboard Signal: PWM_BKL (Backlight)
     * SCM-A11 Reference P2A Wingboard Signal: PWM_BKL (Linear Vibrator)
     * SCM-A11 Reference P2B Wingboard Signal: PWM_BKL (Linear Vibrator)
     * Selected Primary Function: GP_AP_B17 (Output)
     */
    iomux_config_mux(AP_GPIO_AP_B17, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);

    
    /*
     * SCM-A11 Package Pin Name: U2_DTR_B
     * SCM-A11 Reference P1A Wingboard Signal: DAI_RX (Misc)
     * SCM-A11 Reference P1D Wingboard Signal: DAI_RX (Misc)
     * Selected Primary Function: GP_AP_C29 (Output)
     */
    iomux_config_mux(AP_U2_DTR_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);


    /*
     * SPI1 IO/Pads are shared between UART1 and SPI1, we must adjust the
     * IOMUX settings for the SPI1 pins so as not to interfere with UART1.
     *
     * This only applies to P0 wingboards.
     */
    iomux_config_mux(SP_SPI1_SS0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    iomux_config_mux(SP_SPI1_MISO, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    iomux_config_mux(SP_SPI1_MOSI, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    iomux_config_mux(SP_SPI1_CLK, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
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
    /* UART IOMUX settings configured at boot. */
}


/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
    /* UART IOMUX settings configured at boot. */
}


/**
 * The function is stubbed out and does not apply for SCM-A11
 *
 * @param   port        a UART port index
 */
void config_uartdma_event(int port)
{
    return;
}


/*!
 *  Setup GPIO for keypad to be active
 */
void gpio_keypad_active(void)
{
    /* All keypad IOMUX pins are configured to their desired state at boot. */
}


/*!
 * Setup GPIO for keypad to be inactive
 */
void gpio_keypad_inactive(void)
{
    /* All keypad pins are configured at boot. */
}


/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
    /* 
     * SPI1 is not used on the SCM-A11 Wingboards; it conflicts with the 
     * UART1 GPIO settings on P0 Wingboards.
     *
     * SPI2 is setup by the BP. This is setup is initiated by the MBM at
     * boot time.
     */
}


/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
    /* No SPI IOMUX changes are required. */
}


/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
    /* I2C pins are configured at boot. */
}


/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
    /* I2C pins are configured at boot. */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API */


#ifdef CONFIG_MOT_FEAT_GPIO_API_MC13783
/*!
 * This function configures the IOMux block for Atlas standard operations.
 *
 */
void gpio_mc13783_active(void) 
{
}


/*!
 * This function clears the Atlas intrrupt.
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
    /* SCMA11 -> SPI2 = 1 */
    return 1;
}


/*!
 * This function return the SPI smave select for Atlas.
 *
 */
int gpio_mc13783_get_ss(void) 
{
    /* SCMA11BB -> SS = 2 */
    return 2;
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_MC13783 */


#if defined(CONFIG_MOT_FEAT_GPIO_API_SDHC)
/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module) 
{
    switch(module) {
	/* SDHC module 2 is the WLAN controller */
	case 1:
	    gpio_signal_set_data(GPIO_SIGNAL_WLAN_PWR_DWN_B, GPIO_DATA_HIGH);
	    gpio_signal_set_data(GPIO_SIGNAL_WLAN_RESET, GPIO_DATA_HIGH);
	    break;

	default:
	    break;
    }
}


/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module) 
{
    switch(module) {
	/* SDHC module 2 is the WLAN controller */
	case 1:
	    gpio_signal_set_data(GPIO_SIGNAL_WLAN_PWR_DWN_B, GPIO_DATA_LOW);
	    gpio_signal_set_data(GPIO_SIGNAL_WLAN_RESET, GPIO_DATA_LOW);
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
 *
 * @return The function returns 0 on success and -1 on failure.
 **/
int sdhc_intr_setup(void *host, gpio_irq_handler handler)
{
    int ret;

    /*
     * SCM-A11 Package Pin Name: SD2_DAT1
     * SCM-A11 Reference P1A Wingboard Signal: SD2_DATA(1) (WLAN)
     * SCM-A11 Reference P1D Wingboard Signal: SD2_DATA(1) (WLAN)
     * Selected Primary Function: GP_SP_A23 (Input)
     *
     * Use SD2_DAT1 as GPIO for SD1_DET. This varies from the initial
     * IOMUX setting for SD2_DAT1.
     */
    iomux_config_mux(SP_SD2_DAT1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT);

    ret = gpio_signal_request_irq(GPIO_SIGNAL_SD1_DET, GPIO_HIGH_PRIO, handler,
                        0, "MXCMMC", host);

    if (ret != 0) {
        gpio_signal_free_irq(GPIO_SIGNAL_SD1_DET, GPIO_HIGH_PRIO);
    } else {
        gpio_signal_config(GPIO_SIGNAL_SD1_DET, GPIO_GDIR_INPUT,
                GPIO_INT_FALL_EDGE);
    }

    return ret;
}


/*!
 * Free the interrupt request for SD1_DET.
 *
 * @param  host Pointer to MMC/SD host structure.
 */
void sdhc_intr_destroy(void *host)
{
    gpio_signal_free_irq(GPIO_SIGNAL_SD1_DET, GPIO_HIGH_PRIO);
}


/*!
 * Clear the IOMUX/GPIO for SDHC1 SD1_DET.
 * 
 * @param flag Flag represents whether the card is inserted/removed.
 *             Using this sensitive level of GPIO signal is changed.
 *
 **/
void sdhc_intr_clear(int *flag)
{
    if(*flag) {
        gpio_signal_config(GPIO_SIGNAL_SD1_DET, GPIO_GDIR_INPUT,
                GPIO_INT_FALL_EDGE);
        *flag = 0;
    } else {
        gpio_signal_config(GPIO_SIGNAL_SD1_DET, GPIO_GDIR_INPUT,
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
    return (mxc_get_clocks(SDHC1_CLK) / 9) / 32 ;
}


/**
 * Find the maximum clock for SDHC.
 * @param clk SDHC module number.
 * @return Returns the maximum SDHC clock.
 */
unsigned int sdhc_get_max_clock(enum mxc_clocks clk)
{
    return mxc_get_clocks(SDHC1_CLK) / 2;
}


/**
 * Probe for the card. If present the GPIO data would be set.
 *
 * @return  Status of the SD1_DET signal.
 */
int sdhc_find_card(int id)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_SD1_DET);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_SDHC */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LCD)
/*!
 * Setup GPIO for LCD to be active
 *
 */
void gpio_lcd_active(void)
{
    /* enable the serializer */
    gpio_signal_set_data(GPIO_SIGNAL_SER_EN, GPIO_DATA_HIGH);

    /* enable the display */
    gpio_signal_set_data(GPIO_SIGNAL_DISP_RST_B, GPIO_DATA_HIGH);
    gpio_signal_set_data(GPIO_SIGNAL_DISP_CM, GPIO_DATA_LOW);
    gpio_signal_set_data(GPIO_SIGNAL_DISP_SD, GPIO_DATA_LOW);
}


/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
    /* shut down the display */
    gpio_signal_set_data(GPIO_SIGNAL_DISP_SD, GPIO_DATA_HIGH);
    gpio_signal_set_data(GPIO_SIGNAL_DISP_RST_B, GPIO_DATA_LOW);

    /* disable the serializer */
    gpio_signal_set_data(GPIO_SIGNAL_SER_EN, GPIO_DATA_LOW);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
/**
 * Enable or disable the main display's backlight.
 *
 * @param   enable  Set high to enable the backlight; zero to disable it.
 */
void gpio_lcd_backlight_enable(bool enable)
{
    /* PWM_BKL (GP_AP_B17) is no longer connected to the backlight driver
     * on P1A and P1D wingboards. It is only connected to the ETM connectors.
     */
    gpio_signal_set_data(GPIO_SIGNAL_PWM_BKL,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);

    gpio_signal_set_data(GPIO_SIGNAL_MAIN_BKL,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}


/**
 * Get status of lcd backlight gpio signals.
 *
 * @return  Status of the GPIO_SIGNAL_PWM_BKL and GPIO_SIGNAL_MAIN_BKL signals.
 */
int gpio_get_lcd_backlight(void)
{

    __u32 main_bkl;
#ifdef CONFIG_MOT_FEAT_BRDREV
    int data = 0;
    __u32 pwm_bkl;
    
    if( (boardrev() < BOARDREV_P1A) || (boardrev() == BOARDREV_UNKNOWN) ) {
        pwm_bkl = gpio_signal_get_data_check(GPIO_SIGNAL_PWM_BKL);
        main_bkl = gpio_signal_get_data_check(GPIO_SIGNAL_MAIN_BKL);
       
        if( ( pwm_bkl == GPIO_DATA_HIGH ) && ( main_bkl == GPIO_DATA_HIGH ) ) {
	   data = 1;
       }
       
       return data;
    }
    else
    {
#endif /* CONFIG_MOT_FEAT_BRDREV */
        return gpio_signal_get_data_check(GPIO_SIGNAL_MAIN_BKL);
#ifdef CONFIG_MOT_FEAT_BRDREV
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_TORCH)
/**
 * Camera Torch Enable; set high to illuminate
 * 
 * @param   enable  Non-zero enables the camera torch.
 */
void gpio_camera_torch_enable(int enable)
{
    gpio_signal_set_data(GPIO_SIGNAL_CAM_TORCH_EN,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_TORCH */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL)
/**
 * Turn the backlight for the number keys on or off.
 *
 * @param   enable  Set to non-zero to enable the backlight.
 */
void gpio_backlight_numbers_enable(int enable)
{
    if(GPIO_SIGNAL_IS_VALID(GPIO_SIGNAL_EL_NUM_EN)) {
        gpio_signal_set_data(GPIO_SIGNAL_EL_NUM_EN,
                enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
    } else {
        gpio_signal_set_data(GPIO_SIGNAL_EL_EN,
                enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
    }
}


/**
 * Turn the backlight for the navigation keys on or off.
 *
 * @param   enable  Set to non-zero to enable the backlight.
 */
void gpio_backlight_navigation_enable(int enable)
{
    if(GPIO_SIGNAL_IS_VALID(GPIO_SIGNAL_EL_NAV_EN)) {
        gpio_signal_set_data(GPIO_SIGNAL_EL_NAV_EN,
                enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
    } else {
        gpio_signal_set_data(GPIO_SIGNAL_EL_EN,
                enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
    }
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL */


#if defined(CONFIG_MOT_FEAT_GPIO_API_CSI)
/*!
 * Setup GPIO for sensor to be active
 */
void gpio_sensor_active(void)
{
    /* Camera is pulled out of reset at boot, but powered down. */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_PD, GPIO_DATA_LOW);
}


/**
 * Disable camera sensor.
 */
void gpio_sensor_inactive(void)
{
    gpio_signal_set_data(GPIO_SIGNAL_CAM_PD, GPIO_DATA_HIGH);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_CSI */


#if defined(CONFIG_MOT_FEAT_GPIO_API_ETHERNET)
/**
 * Install handler for Ethernet interrupts.
 *
 * @param   handler     Handler for the interrupt.
 * @param   irq_flags   Interrupt request flags.
 * @param   devname     Name associated with device driver requesting interrupt.
 * @param   dev_id      Unique identifier for the device driver.
 *
 * @return  Zero on success; non-zero on failure.
 */
int enet_request_irq(irqreturn_t (*handler)(int, void *, struct pt_regs *),
        unsigned long irq_flags, const char * devname, void *dev_id)
{
    set_irq_type(INT_EXT_INT3, IRQT_RISING);
    return request_irq(INT_EXT_INT3, handler, irq_flags, devname, dev_id);
}


/**
 * Remove handler for Ethernet interrupt.
 *
 * @param   dev_id      Unique identifier for the device driver.
 *
 * @return  Zero.
 */
int enet_free_irq(void *dev_id)
{
    free_irq(INT_EXT_INT3, dev_id);
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


#if defined(CONFIG_MOT_FEAT_GPIO_API_DAI)
/**
 * Multiplexing Use Case 3 -- DAI on UART2 pins
 */
void gpio_dai_enable(void)
{
    /*
     *  Pin U2_DSR_B and pin DAI_FS are configured for DAI at boot.
     *
     *  Pin U2_RI_B is usually in GPIO mode for the GPIO_DISP_SD signal.
     *
     *  On P0C, pin U2_DTR_B is usually in GPIO mode for the
     *  GPIO_DISP_CM signal.
     */

    /*
     * SCM-A11 Reference Design Signal Name: DAI_CLK
     * SCM-A11 Package Pin Name: U2_RI_B
     * Selected Function Name: AD4_TXC
     * Selected Direction: Input/Output
     *
     * Primary function out of reset: U2_RI_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C27
     */
    iomux_config_mux(AP_U2_RI_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_FUNC2);

#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P1D) || (boardrev() == BOARDREV_UNKNOWN) ) {
        /*
         * SCM-A11 Reference Design Signal Name: GPIO_DISP_CM/DAI_RX
         * SCM-A11 Package Pin Name: U2_DTR_B
         * Selected Function Name: AD4_RXD
         * Selected Direction: Input/Output
         *
         * Primary function out of reset: U2_DTR_B
         * Out of Reset State: Input
         * Mux0 Function: GP_AP_C29
         */
        iomux_config_mux(AP_U2_DTR_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE);
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}


/**
 * Return DAI/UART2 to default state.
 */
void gpio_dai_disable(void)
{
    iomux_config_mux(AP_U2_RI_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);

#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P1D) || (boardrev() == BOARDREV_UNKNOWN) ) {
        iomux_config_mux(AP_U2_DTR_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_DAI */


#if defined(CONFIG_MOT_FEAT_GPIO_API_USBHS)
/**
 * Enable high speed USB controller.
 *
 * @param   enable  Set high to enable controller or zero to put controller
 *                  into reset.
 */
void gpio_usb_hs_reset_set_data(__u32 enable)
{
    gpio_signal_set_data(GPIO_SIGNAL_USB_HS_RESET,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}


/**
 * Wakeup high speed USB controller.
 *
 * @param   wakeup  Set high to take high speed USB controller out of sleep;
 *                  set to zero to put USB controller to sleep.
 */
void gpio_usb_hs_wakeup_set_data(__u32 wakeup)
{
    gpio_signal_set_data(GPIO_SIGNAL_USB_HS_WAKEUP,
            wakeup ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}


/**
 * High speed USB switch.
 *
 * @param   swtch   Set to zero for USB connected to Atlas.
 */
void gpio_usb_hs_switch_set_data(__u32 swtch)
{
    gpio_signal_set_data(GPIO_SIGNAL_USB_HS_SWITCH,
            swtch ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}


/**
 * Install an interrupt handler for the FLAGC signal.
 *
 * @param   handler     Function to be called when interrupt arives.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Name of device driver.
 * @param   dev_id      Device identifier to pass to request_irq.
 *
 * @return  Zero on success; non-zero on failure.
 */
int gpio_usb_hs_flagc_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id)
{
    int retval;

    retval = gpio_signal_request_irq(GPIO_SIGNAL_USB_HS_FLAGC,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);

    if(retval == 0) {
        /* only configure signal for interrupts if request_irq was successful */
    gpio_signal_config(GPIO_SIGNAL_USB_HS_FLAGC, GPIO_GDIR_INPUT,
            GPIO_INT_HIGH_LEV);
    }

    return retval;
}


/**
 * Remove the interrupt handler for the FLAGC signal.
 *
 * @param   dev_id      Device identifier to pass to request_irq.
 */
void gpio_usb_hs_flagc_free_irq(void *dev_id)
{
    gpio_signal_free_irq(GPIO_SIGNAL_USB_HS_FLAGC, GPIO_HIGH_PRIO);
}


/**
 * Clear pending FLAGC interrupt in GPIO controller.
 */
void gpio_usb_hs_flagc_clear_int(void)
{
    gpio_signal_clear_int(GPIO_SIGNAL_USB_HS_FLAGC);
}


/**
 * Get value of USB HS FLAGC signal.
 *
 * @return  Status of USB_HS_FLAGC signal.
 */
__u32 gpio_usb_hs_flagc_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_USB_HS_FLAGC);
}


/**
 * Configure high speed USB interrupt.
 *
 * @param   handler     Function to be called when interrupt arives.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Name of device driver.
 * @param   dev_id      Device identifier to pass to request_irq.
 *
 * @return  Zero on success; non-zero on failure.
 */
int gpio_usb_hs_int_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id)
{
#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P1D) || (boardrev() == BOARDREV_UNKNOWN) ) {
        return -ENODEV;
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    set_irq_type(INT_EXT_INT6, IRQT_LOW);
    return request_irq(INT_EXT_INT6, handler, irq_flags, devname, dev_id);    
}


/**
 * Remove the high speed USB interrupt handler.
 *
 * @param   dev_id      Device identifier passed to request_irq.
 *
 * @return  Zero on success.
 */
void gpio_usb_hs_int_free_irq(void *dev_id)
{
    free_irq(INT_EXT_INT6, dev_id);
}


/**
 * Clear the high speed USB interrupt signal.
 */
void gpio_usb_hs_int_clear_int(void)
{
    /* NO OP */
}


/**
 * Get that status of the USB HS interrupt signal.
 */
__u32 gpio_usb_hs_int_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_USB_HS_INT);
}


/**
 * Configure pin GP_AP_B22 (signal USB_HS_DMA_REQ) for GPIO mode.
 */
void gpio_usb_hs_dma_req_config_gpio_mode(void)
{
    iomux_config_mux(AP_GPIO_AP_B22, OUTPUTCONFIG_DEFAULT,
            INPUTCONFIG_NONE);
}


/**
 * Configure pin GP_AP_B22 (signal USB_HS_DMA_REQ) for DMAREQ0 mode.
 */
void gpio_usb_hs_dma_req_config_sdma_mode(void)
{
    iomux_config_mux(AP_GPIO_AP_B22, OUTPUTCONFIG_FUNC2,
            INPUTCONFIG_NONE);
}


/**
 * Configure pin GP_AP_B22 (signal USB_HS_DMA_REQ) for GPIO primary
 * and DMAREQ0 secondary mode.
 */
void gpio_usb_hs_dma_req_config_dual_mode(void)
{
    iomux_config_mux(AP_GPIO_AP_B22, OUTPUTCONFIG_DEFAULT,
            INPUTCONFIG_FUNC2);
}


/**
 * Install handler for USB_HS_DMA_REQ interrupt interrupt when in GPIO
 * mode.
 *
 * @param   handler     Function to be called when interrupt arives.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Name of device driver.
 * @param   dev_id      Device identifier to pass to request_irq.
 *
 * @return  Zero on success; non-zero on failure.
 */
int gpio_usb_hs_dma_req_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id)
{
    int retval;

    return gpio_signal_request_irq(GPIO_SIGNAL_USB_HS_DMA_REQ,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);
}


/**
 * Adjust the interrupt trigger level for the USB_HS_DMA_REQ interrupt when
 * in GPIO mode.
 *
 * @param   edge    Trigger level for interrupt.
 */
void gpio_usb_hs_dma_req_set_irq_type(gpio_edge_t edge)
{
    gpio_signal_config(GPIO_SIGNAL_USB_HS_DMA_REQ, GPIO_GDIR_INPUT,
            edge);
}


/**
 * Remove interrupt handler for USB_HS_DMA_REQ.
 *
 * @param   dev_id      Device identifier to pass to request_irq.
 */
void gpio_usb_hs_dma_req_free_irq(void *dev_id)
{
    gpio_signal_free_irq(GPIO_SIGNAL_USB_HS_DMA_REQ, GPIO_HIGH_PRIO);
}


/**
 * Clear pending GPIO interrupt for USB_HS_DMA_REQ.
 */
void gpio_usb_hs_dma_req_clear_int(void)
{
    gpio_signal_clear_int(GPIO_SIGNAL_USB_HS_DMA_REQ);
}


/**
 * Get the current status of the USB_HS_DMA_REQ signal.
 */
__u32 gpio_usb_hs_dma_req_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_USB_HS_DMA_REQ);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_USBHS */


#if defined(CONFIG_MOT_FEAT_GPIO_API_WLAN)
/**
 * Set the state of the WLAN_RESET signal.
 *
 * @param   data    Desired state of the signal.
 */
void gpio_wlan_reset_set_data(__u32 data)
{
    gpio_signal_set_data(GPIO_SIGNAL_WLAN_RESET, data);
}


/**
 * Set the state of the WLAN_CLIENT_WAKE_B signal.
 *
 * @param   data    Desired state of the signal.
 */
void gpio_wlan_clientwake_set_data(__u32 data)
{
    gpio_signal_set_data(GPIO_SIGNAL_WLAN_CLIENT_WAKE_B, data);
}


/**
 * Get the current status of the WLAN_PWR_DWN_B signal.
 *
 * @return  Zero if WLAN_PWR_DWN_B is low; non-zero if it is high.
 */
__u32 gpio_wlan_powerdown_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_WLAN_PWR_DWN_B);
}


/**
 * Set the current status of the WLAN_PWR_DWN_B signal.
 *
 * @return  Zero if WLAN_PWR_DWN_B is low; non-zero if it is high.
 */
void gpio_wlan_powerdown_set_data(__u32 data)
{
    gpio_signal_set_data(GPIO_SIGNAL_WLAN_PWR_DWN_B, data);
}


/**
 * Install handler for WLAN_HOST_WAKE_B interrupt.
 *
 * @param   handler     Function to be called when interrupt arives.
 * @param   irq_flags   Flags to pass to request_irq.
 * @param   devname     Name of device driver.
 * @param   dev_id      Device identifier to pass to request_irq.
 *
 * @return  Zero on success; non-zero on failure.
 */
int gpio_wlan_hostwake_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id)
{
    set_irq_type(INT_EXT_INT0, IRQT_BOTHEDGE);
    return request_irq(INT_EXT_INT0, handler, irq_flags, devname, dev_id);    
}


/**
 * Remove handler for WLAN_HOST_WAKE_B interrupt.
 *
 * @param   dev_id      Device identifier to pass to free_irq.
 */
void gpio_wlan_hostwake_free_irq(void *dev_id)
{
    free_irq(INT_EXT_INT0, dev_id);
}


/**
 * Clear the WLAN_HOST_WAKE_B interrupt.
 */
void gpio_wlan_hostwake_clear_int(void)
{
    /* NO OP */
}


/**
 * Get the current status of WLAN_HOST_WAKE_B.
 *
 * @return  Zero if signal is low; non-zero if signal is high.
 */
__u32 gpio_wlan_hostwake_get_data(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_WLAN_HOST_WAKE_B);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_WLAN */


#if defined(CONFIG_MOT_FEAT_GPIO_API_MMCSD)
/*!
* Clear the IOMUX/GPIO for MMC/SD.
*/
void mmcsd_intr_destroy(void *host)
{
	free_irq(INT_EXT_INT4, host);
}


/*!
 * Setup GPIO for MMC/SD
 *
 * @param id 	mmc/sd host id
 */
void gpio_mmcsd_active(int id) 
{
}


/*!
 * Setup GPIO for MMC/SD
 *
 * @param id 	mmc/sd host id
 */
void gpio_mmcsd_inactive(int id) 
{
}


/*!
 * Setup the EDIO pin for MMC/SD.
 * 
 * @param  host Pointer to MMC/SD host structure.
 * @param  handler      GPIO ISR function pointer for the GPIO signal.
 *
 * @return The function returns 0 on success and -1 on failure.
 **/
int mmcsd_intr_setup(void *host, gpio_irq_handler handler)
{
    int ret = 0;

    set_irq_type(INT_EXT_INT4, IRQT_BOTHEDGE);
    ret = request_irq(INT_EXT_INT4, handler, SA_INTERRUPT | SA_SAMPLE_RANDOM, 
			"mmc card detect", (void *)host);
    
    if (ret) {
        printk(KERN_ERR "MMC Unable to initialize card detect interrupt.\n");
    }
    
    return ret;
}


/*!
 * Clear the EDIO for MMC/SD.
 * 
 * @param flag Flag represents whether the card is inserted/removed.
 *             Using this sensitive level of GPIO signal is changed.
 *
 **/
void mmcsd_intr_clear(int *flag)
{
    /* reserved interface */
}


/**
 * Find the minimum clock for MMC/SD.
 *
 * @param clk SDHC module number.
 * @return Returns the minimum SDHC clock.
 */
unsigned int mmcsd_get_min_clock(enum mxc_clocks clk)
{
    return (mxc_get_clocks(clk) / 9) / 32 ;
}


/**
 * Find the maximum clock for MMC/SD.
 * @param clk SDHC module number.
 * @return Returns the maximum SDHC clock.
 */
unsigned int mmcsd_get_max_clock(enum mxc_clocks clk)
{
    return mxc_get_clocks(clk) / 2;
}


/**
 * Probe for the card. If present the EDIO data would be set.
 *
 * @param id 	mmc/sd host id
 * @return  Status of the EDIO signal.
 */
int mmcsd_find_card(int id)
{
    __u32 data = 0;

    switch (id) {
        case 0:
            data = edio_get_data(ED_INT4);
            break;
        case 1:
            data = 0;
            break;
        default:
            printk("%s: we should not be here!\n", __FUNCTION__);
    }
    return data;
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_MMCSD */
