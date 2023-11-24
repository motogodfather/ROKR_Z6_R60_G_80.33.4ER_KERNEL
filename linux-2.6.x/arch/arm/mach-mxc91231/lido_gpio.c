/*
 * linux/arch/arm/mach-mxc91231/lido_gpio.c
 *
 * Copyright 2006-2007 Motorola, Inc.
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
 * 02-Nov-2006  Motorola        Changed IOMUX pad group 22 setting.
 * 10-Nov-2006  Motorola        Update high speed USB API.
 * 10-Nov-2006  Motorola        Added support for Lido P2.
 * 13-Nov-2006  Motorola        Change pad group 25 settings.
 * 30-Nov-2006  Motorola        Added support for toggling AP_IPU_D3_CLK between
 *                              default output function and function1
 * 15-Dec-2006  Motorola        IOMUX pad 12 current drain improvement
 * 26-Jan-2007  Motorola        Bluetooth current drain improvements.
 * 20-Feb-2007  Motorola        Add SDHC API.
 * 20-Mar-2007  Motorola        Adjust GPIO settings for IPU_D3_CLK.
 */


#include <linux/module.h>
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
 * @file lido_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */

/*
 * Local functions.
 */
#ifdef CONFIG_MOT_FEAT_BRDREV
static void gpio_setting_fixup_p1(void);
#endif /* CONFIG_MOT_FEAT_BRDREV */


/**
 * Initial GPIO register settings.
 */
struct gpio_signal_settings initial_gpio_settings[MAX_GPIO_SIGNAL] = {
    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * Lido P1 Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
     *
     * Array index: 0   GPIO_SIGNAL_BT_POWER
     *
     * Power off Bluetooth at boot time. (Signal is connected to Bluetooth's
     * VREG_CTL.
     *
     */
    { GPIO_AP_C_PORT,    8, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: SPI1_MOSI
     * Lido P1 Signal: USB_HS_RESET (USB HS)
     * Selected Primary Function: GP_SP_A28 (Output)
     *
     * Array index: 1   GPIO_SIGNAL_USB_HS_RESET
     * 
     * Disable high speed USB controller at boot.
     */
    { GPIO_SP_A_PORT,   28, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_B22
     * Lido P1 Signal: GP_AP_B22 (Wing Connector)
     * Selected Primary Function: GP_AP_B22 (Input)
     *
     * Array index: 2   GPIO_SIGNAL_USB_HS_DMA_REQ
     */
    { GPIO_AP_B_PORT,   22, GPIO_GDIR_INPUT,  GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C9
     * Lido P1 Signal: USB_HS_WAKEUP/10K PD (USB HS)
     * Selected Primary Function: GP_AP_C9 (Output)
     *
     * Array index: 3   GPIO_SIGNAL_USB_HS_WAKEUP
     *
     * Put highspeed USB into sleep mode.
     */
    { GPIO_AP_C_PORT,    9, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C10
     * Lido P1 Signal: USB_HS_FLAGC (USB HS)
     * Selected Primary Function: GP_AP_C10 (Input)
     *
     * Array index: 4   GPIO_SIGNAL_USB_HS_FLAGC
     */
    { GPIO_AP_C_PORT,   10, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: ED_INT6
     * Lido P1 Signal: USB_HS_INT (USB HS)
     * Selected Primary Function: GP_AP_C24 (Input)
     * Selected Secondary Function: ED_INT6 (Input)
     *
     * Array index: 5   GPIO_SIGNAL_USB_HS_INT
     *
     * High speed USB interrupt.
     */
    { GPIO_AP_C_PORT,   24, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: UH2_TXDP
     * Lido P1 Signal: USB_HS_SWITCH (USB HS)
     * Selected Primary Function: GP_SP_A11 (Output)
     *
     * Array index: 6   GPIO_SIGNAL_USB_HS_SWITCH
     *
     * Low setting means USB HS is connected to Atlas.
     */
    { GPIO_SP_A_PORT,   11, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: IPU_PAR_RS
     * Lido P1 Signal: SER_EN (Display)
     * Selected Primary Function: GP_AP_A29 (Output)
     *
     * Array index: 7   GPIO_SIGNAL_SER_EN
     *
     * MBM will disable/enable the serializer at boot as appropriate.
     */
    { GPIO_AP_A_PORT,   29, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: IPU_D0_CS
     * Lido P1 Signal: GPIO_DISP_SD (Display)
     * Selected Primary Function: GP_AP_A26 (Output)
     *
     * Array index: 8   GPIO_SIGNAL_DISP_SD
     *
     * MBM will disable/enable the display at boot as appropriate.
     */
    { GPIO_AP_A_PORT,   26, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: CSI_D0
     * Lido P1 Signal: GPIO_CAM_RST_B (camera)
     * Selected Primary Function: GP_AP_B24 (Output)
     *
     * Array index: 9   GPIO_SIGNAL_CAM_RST_B
     *
     * Take camera out of reset at boot.
     */
    { GPIO_AP_B_PORT,   24, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: CSI_D1
     * Lido P1 Signal: GPIO_CAM_PD (camera)
     * Selected Primary Function: GP_AP_B25 (Output)
     *
     * Array index: 10  GPIO_SIGNAL_CAM_PD
     *
     * Power down camera at boot.
     */
    { GPIO_AP_B_PORT,   25, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: UH2_RXDM
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A13 (Output)
     *
     * Array index: 11  GPIO_SIGNAL_SP_A13
     */
    { GPIO_SP_A_PORT,   13, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: SPI1_SS0
     * Lido P1 Signal: CAM_FLASH_EN (Camera Flash)
     * Selected Primary Function: GP_SP_A30 (Output)
     *
     * Array index: 12  GPIO_SIGNAL_CAM_FLASH_EN
     */
    { GPIO_SP_A_PORT,   30, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },
    
    /*
     * SCM-A11 Package Pin Name: ED_INT3
     * Lido P1 Signal: FLIP_OPEN (Misc)
     * Selected Primary Function: GP_AP_C21 (Input)
     * Selected Secondary Function: ED_INT3 (Input)
     *
     * Array index: 13  GPIO_SIGNAL_FLIP_OPEN
     */
    { GPIO_AP_C_PORT,   21, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },
    
    /*
     * SCM-A11 Package Pin Name: SPI1_CLK
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     *
     * Array index: 14  GPIO_SIGNAL_SP_A27
     */
    { GPIO_SP_A_PORT,   27, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },
    
    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * Lido P1 Signal: EL_EN (Backlight)
     * Selected Primary Function: GP_SP_A29 (Output)
     *
     * Array index: 15  GPIO_SIGNAL_EL_EN
     */
    { GPIO_SP_A_PORT,   29, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_B18
     * Lido P1 Signal: GPIO_DISP_RST_B (Display)
     * Selected Primary Function: GP_AP_B18 (Output)
     *
     * Array index: 16  GPIO_SIGNAL_DISP_RST_B
     *
     * MBM will set this at boot.
     */
    { GPIO_AP_B_PORT,   18, GPIO_GDIR_OUTPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C12
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_AP_C12 (Output)
     *
     * Array index: 17  GPIO_SIGNAL_AP_C12
     */
    { GPIO_AP_C_PORT,   12, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: GP_AP_C13
     * Lido P1 Signal: BT_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C13 (Output)
     *
     * Array index: 18  GPIO_SIGNAL_BT_WAKE_B
     */
    { GPIO_AP_C_PORT,   13, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: LIN_VIB_AMP_EN (Haptics)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Array index: 19  GPIO_SIGNAL_SP_A2
     */
    { GPIO_INVALID_PORT,     2, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: U3_CTS_B
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A3 (Output)
     *
     * Array index: 20  GPIO_SIGNAL_SP_A3
     */
    { GPIO_SP_A_PORT,    3, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Array index: 21  GPIO_SIGNAL_SP_A8
     */
    { GPIO_SP_A_PORT,    8, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A9 (Output)
     *
     * Array index: 22  GPIO_SIGNAL_SP_A9
     */
    { GPIO_SP_A_PORT,    9, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * Lido P1 Signal: TOUCH_INTB (Morphing)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Array index: 23  GPIO_SIGNAL_TOUCH_INTB
     *
     * Capacitive Touch Keys -- Interrupt
     */
    { GPIO_SP_A_PORT,   12, GPIO_GDIR_INPUT,   GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: UH2_OVR
     * Lido P1 Signal: USB_XCVR_EN (Atlas)
     * Selected Primary Function: GP_SP_A14 (Output)
     *
     * Array index: 24  GPIO_SIGNAL_USB_XCVR_EN
     */
    { GPIO_SP_A_PORT,   14, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: UH2_PWR
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A15 (Output)
     *
     * Array index: 25  GPIO_SIGNAL_SP_A15
     */
    { GPIO_SP_A_PORT,   15, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: LIN_VIB_AMP_EN (Haptics)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Array index: 26  GPIO_SIGNAL_LIN_VIB_AMP_EN
     *
     * Dispable vibrator amplifier at boot.
     */
    { GPIO_SP_A_PORT,    2, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: ED_INT1
     * Ascension P1 Signal: BT_HOST_WAKE_B (Bluetooth)
     * Ascension P2 Signal: BT_HOST_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C19 (Input)
     * Selected Secondary Function: ED_INT1 (Input)
     *
     * Array index: 27  GPIO_SIGNAL_BT_HOST_WAKE_B
     *
     * Host wake interrupt from bluetooth controller.
     */
    { GPIO_AP_C_PORT,   19, GPIO_GDIR_INPUT,    GPIO_DATA_INVALID },

    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * Lido P1 Signal: LIN_VIB_AMP_EN (Haptics)
     * Lido P2 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A10 (Output)
     *
     * Array index: 28  GPIO_SIGNAL_SP_A10
     */
    { GPIO_SP_A_PORT,   10, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: IPU_WR
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: GPIO_SER_RST_B (Display)
     * Selected Primary Function: GP_AP_B1 (Output)
     *
     * Array index: 29  GPIO_SIGNAL_SER_RST_B
     *
     * Low means serializer is in reset.
     */
    { GPIO_AP_B_PORT,    1, GPIO_GDIR_OUTPUT,   GPIO_DATA_HIGH },

    /*
     * SCM-A11 Package Pin Name: U1_TXD
     * Lido P1 Signal: BT_RX_AP_TX (Bluetooth)
     * Selected Primary Function: U1_TXD (Output)
     * Mux0 Function: GP_AP_A7
     *
     * Lido uses Broadcom Bluetooth controller with internal pull downs,
     * so drive BT pins low when inactive to decrease current drain.
     *
     * Array index: 30  GPIO_SIGNAL_U1_TXD
     */
    { GPIO_AP_A_PORT,        7, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: U1_CTS_B
     * Lido P1 Signal: BT_CTS_B (Bluetooth)
     * Selected Primary Function: U1_CTS_B (Output)
     * Mux0 Function: GP_AP_A10
     *
     * Lido uses Broadcom Bluetooth controller with internal pull downs,
     * so drive BT pins low when inactive to decrease current drain.
     *
     * Array index: 31  GPIO_SIGNAL_U1_CTS_B
     */
    { GPIO_AP_A_PORT,       10, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },

    /*
     * SCM-A11 Package Pin Name: IPU_D3_CLK
     * Lido P1 Signal: IPU_D3_CLK (Display)
     * Selected Primary Function: IPU_D3_CLK (Output)
     *
     * Array index: 32  GPIO_SIGNAL_IPU_D3_CLK
     *
     * Prevent IPU_D3_CLK from floating when the display clock is 
     * disabled.
     */
    { GPIO_AP_A_PORT,   23, GPIO_GDIR_OUTPUT,   GPIO_DATA_LOW },
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

    /* SDRAM -- DSE_HIGH */
    { IOPAD_GROUP3, 0x0002 },

    /* SDRAM -- DSE_HIGH | PKE_ENABLE */
    { IOPAD_GROUP4, 0x0082 },

    /* SDRAM -- SRE_FAST | DSE_HIGH */
    { IOPAD_GROUP5, 0x0003 },

    /* SDRAM -- SRE_FAST | DSE_HIGH | PUS_100K_PULLUP | PKE_ENABLE */
    { IOPAD_GROUP6, 0x00C3 },

    /* SDRAM -- SRE_FAST | DSE_MIN | PKE_ENABLE */
    { IOPAD_GROUP7, 0x0087 },

    /* SDRAM -- SRE_FAST */
    { IOPAD_GROUP8, 0x0001 },

    /* SDRAM -- SRE_FAST | DSE_MIN | PKE_ENABLE */
    { IOPAD_GROUP9, 0x0087 },

    /* Audio & AP GPIOs -- PUS_22K_PULLUP */
    { IOPAD_GROUP10, 0x0060 },

    /* Keypad -- PUS_47K_PULLUP */
    { IOPAD_GROUP11, 0x0020 },

    /* Keypad -- DSE_MAX | DDR_MODE_DDR | PUS_100K_PULLUP */
    { IOPAD_GROUP12, 0x0244 },

    /* I2C -- DDR_MODE_DDR | PUS_100K_PULLUP */
    { IOPAD_GROUP13, 0x0240 },

    /* CSPI1 -- 0 */
    { IOPAD_GROUP14, 0x0000 },

    /* CKO & CKOH -- DSE_HIGH */
    { IOPAD_GROUP15, 0x0002 },

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

    /* CSI (Camera) -- DSE_MAX */
    { IOPAD_GROUP22, 0x0004 },

    /* EDIO -- PUS_100K_PULLUP */
    { IOPAD_GROUP23, 0x0040 },

    /* EDIO -- PUS_100K_PULLUP */
    { IOPAD_GROUP24, 0x0040 },

    /* DIG --  HYS_SCHMITZ | PKE_ENABLE | PUS_22K_PULLUP | SRE_FAST*/
    { IOPAD_GROUP25, 0x01E1 },

    /* SDHC -- PUS_100K_PULLUP | PKE_ENABLE */
    { IOPAD_GROUP26, 0x00C0 },

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
     * Lido P1 Signal: ASAP_TX (Atlas)
     * Selected Primary Function: AD1_TXD (Output)
     *
     * Primary function out of reset: AD1_TXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A11
     */
    { AP_AD1_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD1_RXD
     * Lido P1 Signal: ASAP_RX (Atlas)
     * Selected Primary Function: AD1_RXD (Input)
     *
     * Primary function out of reset: AD1_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A12
     */
    { AP_AD1_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD1_TXC
     * Lido P1 Signal: ASAP_CLK (Atlas)
     * Selected Primary Function: AD1_TXC (Input/Output)
     *
     * Primary function out of reset: AD1_TXC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A13
     */
    { AP_AD1_TXC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD1_TXFS
     * Lido P1 Signal: ASAP_FS (Atlas)
     * Selected Primary Function: AD1_TXFS (Input/Output)
     *
     * Primary function out of reset: AD1_TXFS
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A14
     */
    { AP_AD1_TXFS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD2_TXD
     * Lido P1 Signal: MMSAP_TX (Atlas)
     * Selected Primary Function: AD2_TXD (Output)
     *
     * Primary function out of reset: AD2_TXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A15
     */
    { AP_AD2_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD2_RXD
     * Lido P1 Signal: MMSAP_RX (Atlas)
     * Selected Primary Function: AD2_RXD (Input)
     *
     * Primary function out of reset: AD2_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A16
     */
    { AP_AD2_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: AD2_TXC
     * Lido P1 Signal: MMSAP_CLK (Atlas)
     * Selected Primary Function: AD2_TXC (Input/Output)
     *
     * Primary function out of reset: AD2_TXC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A17
     */
    { AP_AD2_TXC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: AD2_TXFS
     * Lido P1 Signal: MMSAP_FS (Atlas)
     * Selected Primary Function: AD2_TXFS (Input/Output)
     *
     * Primary function out of reset: AD2_TXFS
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A18
     */
    { AP_AD2_TXFS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW5
     * Lido P1 Signal: LOBAT_B (Atlas)
     * Selected Primary Function: GP_AP_B16 (Input)
     *
     * Primary function out of reset: KPROW5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B16
     */
    { AP_KPROW5, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_B23
     * Lido P1 Signal: POWER_RDY (Atlas)
     * Selected Primary Function: GP_AP_B23 (Input)
     *
     * Primary function out of reset: GP_AP_B23
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B23
     */
    { AP_GPIO_AP_B23, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C15
     * Lido P1 Signal: WDOG_AP (Atlas)
     * Selected Primary Function: WDOG_AP (Output)
     *
     * Primary function out of reset: GP_AP_C15
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C15
     */
    { AP_GPIO_AP_C15, OUTPUTCONFIG_FUNC4, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C16
     * Lido P1 Signal: USB_VPIN (Atlas)
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
     * Lido P1 Signal: USB_VMIN (Atlas)
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
     * Lido P1 Signal: PM_INT (Atlas)
     * Selected Primary Function: GP_AP_C23 (Input)
     * Selected Secondary Function: ED_INT5 (Input)
     *
     * Primary function out of reset: ED_INT5
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C23
     */
    { AP_ED_INT5, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: USB_TXOE_B
     * Lido P1 Signal: USB_TXEN_B (Atlas)
     * Selected Primary Function: USB_TXOE_B (Output)
     *
     * Primary function out of reset: USB_TXOE_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A4
     */
    { SP_USB_TXOE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: USB_DAT_VP
     * Lido P1 Signal: USB_VPOUT (Atlas)
     * Selected Primary Function: USB_DAT_VP (Input/Output)
     *
     * Primary function out of reset: USB_DAT_VP
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A5
     */
    { SP_USB_DAT_VP, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: USB_SE0_VM
     * Lido P1 Signal: USB_VMOUT (Atlas)
     * Selected Primary Function: USB_SE0_VM (Input/Output)
     *
     * Primary function out of reset: USB_SE0_VM
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A6
     */
    { SP_USB_SE0_VM, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: USB_RXD
     * Lido P1 Signal: USB_XRXD (Atlas)
     * Selected Primary Function: USB_RXD (Input)
     *
     * Primary function out of reset: USB_RXD
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A7
     */
    { SP_USB_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_OVR
     * Lido P1 Signal: USB_XCVR_EN (Atlas)
     * Selected Primary Function: GP_SP_A14 (Output)
     *
     * Primary function out of reset: UH2_OVR
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A14
     */
    { SP_UH2_OVR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_SVEN
     * Lido P1 Signal: VSIM_EN (Atlas)
     * Selected Primary Function: SIM1_SVEN (Output)
     *
     * Primary function out of reset: SIM1_SVEN
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SIM1_SVEN, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_PD
     * Lido P1 Signal: BAT_DETB (Atlas)
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
 * Keypad backlight IOMUX settings.
 */
static struct iomux_initialization backlight_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: SPI1_MISO
     * Lido P1 Signal: EL_EN (Backlight)
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
     * Lido P1 Signal: BT_RX_AP_TX (Bluetooth)
     * Selected Primary Function: U1_TXD (Output)
     *
     * Primary function out of reset: U1_TXD
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A7
     */
    { AP_U1_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_RXD
     * Lido P1 Signal: BT_TX_AP_RX (Bluetooth)
     * Selected Primary Function: U1_RXD (Input)
     *
     * Primary function out of reset: U1_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A8
     */
    { AP_U1_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_RTS_B
     * Lido P1 Signal: BT_RTS_B (Bluetooth)
     * Selected Primary Function: U1_RTS_B (Input)
     *
     * Primary function out of reset: U1_RTS_B
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A9
     */
    { AP_U1_RTS_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U1_CTS_B
     * Lido P1 Signal: BT_CTS_B (Bluetooth)
     * Selected Primary Function: U1_CTS_B (Output)
     *
     * Primary function out of reset: U1_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A10
     */
    { AP_U1_CTS_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C8
     * Lido P1 Signal: BT_RESET_B (Bluetooth)
     * Selected Primary Function: GP_AP_C8 (Output)
         *
         * Primary function out of reset: GP_AP_C8
         * Out of Reset State: Input
         * Mux0 Function: GP_AP_C8
     */
    { AP_GPIO_AP_C8, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C11
     * Lido P1 Signal: BT_CLK_EN (Bluetooth)
     * Selected Primary Function: GP_AP_C11 (Input)
     *
     * Primary function out of reset: GP_AP_C11
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C11
     */
    { AP_GPIO_AP_C11, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C13
     * Lido P1 Signal: BT_WAKE_B (Bluetooth)
     * Selected Primary Function: GP_AP_C13 (Output)
     *
     * Primary function out of reset: GP_AP_C13
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C13
     */
    { AP_GPIO_AP_C13, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT1
     * Lido P1 Signal: BT_HOST_WAKE_B (Bluetooth)
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
 * CSI - camera sensor interface IOMUX settings
 */
static struct iomux_initialization camera_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: CSI_D0
     * Lido P1 Signal: GPIO_CAM_RST_B (camera)
     * Selected Primary Function: GP_AP_B24 (Output)
     *
     * Primary function out of reset: CSI_D0
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B24
     */
    { AP_CSI_D0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D1
     * Lido P1 Signal: GPIO_CAM_PD (camera)
     * Selected Primary Function: GP_AP_B25 (Output)
     *
     * Primary function out of reset: CSI_D1
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B25
     */
    { AP_CSI_D1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D2
     * Lido P1 Signal: CSI_D(0) (camera)
     * Selected Primary Function: CSI_D2 (Input)
     *
     * Primary function out of reset: CSI_D2
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B26
     */
    { AP_CSI_D2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D3
     * Lido P1 Signal: CSI_D(1) (camera)
     * Selected Primary Function: CSI_D3 (Input)
     *
     * Primary function out of reset: CSI_D3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B27
     */
    { AP_CSI_D3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D4
     * Lido P1 Signal: CSI_D(2) (camera)
     * Selected Primary Function: CSI_D4 (Input)
     *
     * Primary function out of reset: CSI_D4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B28
     */
    { AP_CSI_D4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D5
     * Lido P1 Signal: CSI_D(3) (camera)
     * Selected Primary Function: CSI_D5 (Input)
     *
     * Primary function out of reset: CSI_D5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B29
     */
    { AP_CSI_D5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D6
     * Lido P1 Signal: CSI_D(4) (camera)
     * Selected Primary Function: CSI_D6 (Input)
     *
     * Primary function out of reset: CSI_D6
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B30
     */
    { AP_CSI_D6, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D7
     * Lido P1 Signal: CSI_D(5) (camera)
     * Selected Primary Function: CSI_D7 (Input)
     *
     * Primary function out of reset: CSI_D7
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B31
     */
    { AP_CSI_D7, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D8
     * Lido P1 Signal: CSI_D(6) (camera)
     * Selected Primary Function: CSI_D8 (Input)
     *
     * Primary function out of reset: CSI_D8
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C0
     */
    { AP_CSI_D8, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_D9
     * Lido P1 Signal: CSI_D(7) (camera)
     * Selected Primary Function: CSI_D9 (Input)
     *
     * Primary function out of reset: CSI_D9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C1
     */
    { AP_CSI_D9, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_MCLK
     * Lido P1 Signal: CSI_MCLK (camera)
     * Selected Primary Function: CSI_MCLK (Output)
     *
     * Primary function out of reset: CSI_MCLK
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_C2
     */
    { AP_CSI_MCLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_VSYNC
     * Lido P1 Signal: CSI_VSYNC (camera)
     * Selected Primary Function: CSI_VSYNC (Input)
     *
     * Primary function out of reset: CSI_VSYNC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C3
     */
    { AP_CSI_VSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_HSYNC
     * Lido P1 Signal: CSI_HSYNC (camera)
     * Selected Primary Function: CSI_HSYNC (Input)
     *
     * Primary function out of reset: CSI_HSYNC
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C4
     */
    { AP_CSI_HSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CSI_PIXCLK
     * Lido P1 Signal: CSI_PIXCLK (camera)
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
     * SCM-A11 Package Pin Name: SPI1_SS0
     * Lido P1 Signal: CAM_FLASH_EN (Camera Flash)
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
     * Lido P1 Signal: IPU_D0_VSYNC_LD17 (Display)
     * Selected Primary Function: IPU_BE1_LD17 (Output)
     *
     * Primary function out of reset: IPU_LD17
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A20
     */
    { AP_IPU_LD17, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_VSYNC
     * Lido P1 Signal: IPU_D3_VSYNC (Display)
     * Selected Primary Function: IPU_D3_VSYNC (Output)
     *
     * Primary function out of reset: IPU_D3_VSYNC
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A21
     */
    { AP_IPU_D3_VSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_HSYNC
     * Lido P1 Signal: IPU_D3_HSYNC (Display)
     * Selected Primary Function: IPU_D3_HSYNC (Output)
     *
     * Primary function out of reset: IPU_D3_HSYNC
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A22
     */
    { AP_IPU_D3_HSYNC, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CLK
     * Lido P1 Signal: IPU_D3_CLK (Display)
     * Selected Primary Function: IPU_D3_CLK (Output)
     *
     * Primary function out of reset: IPU_D3_CLK
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A23
     */
    { AP_IPU_D3_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_DRDY
     * Lido P1 Signal: IPU_D3_DRDY (Display)
     * Selected Primary Function: IPU_D3_DRDY (Output)
     *
     * Primary function out of reset: IPU_D3_DRDY
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A24
     */
    { AP_IPU_D3_DRDY, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D0_CS
     * Lido P1 Signal: GPIO_DISP_SD (Display)
     * Selected Primary Function: GP_AP_A26 (Output)
     *
     * Primary function out of reset: IPU_D0_CS
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A26
     */
    { AP_IPU_D0_CS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_BE0_LD16
     * Lido P1 Signal: IPU_D1CS_LD16 (Display)
     * Selected Primary Function: IPU_BE0_LD16 (Output)
     *
     * Primary function out of reset: IPU_LD16
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_A27
     */
    { AP_IPU_LD16, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_PAR_RS
     * Lido P1 Signal: SER_EN (Display)
     * Selected Primary Function: GP_AP_A29 (Output)
     *
     * Primary function out of reset: IPU_PAR_RS
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A29
     */
    { AP_IPU_PAR_RS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_SPL
     * Lido P1 Signal: IPU_SD_MISO (Display)
     * Selected Primary Function: IPU_SD_D_INPUT (Input)
     *
     * Primary function out of reset: IPU_D3_SPL
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A30
     */
    { AP_IPU_D3_PS, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CLS
     * Lido P1 Signal: IPU_SD_CLK (Display)
     * Selected Primary Function: IPU_SD_CLK (Output)
     *
     * Primary function out of reset: IPU_D3_CLS
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A31
     */
    { AP_IPU_D3_CLS, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_RD
     * Lido P1 Signal: IPU_SD_MOSI (Display)
     * Selected Primary Function: IPU_SD_D_INOUT (Input/Output)
     *
     * Primary function out of reset: IPU_RD
     * Out of Reset State: High
     * Mux0 Function: GP_AP_B0
     */
    { AP_IPU_RD, OUTPUTCONFIG_FUNC3, INPUTCONFIG_FUNC3 },
    /*
     * SCM-A11 Package Pin Name: IPU_LD0
     * Lido P1 Signal: IPU_LD(0) (Display)
     * Selected Primary Function: IPU_LD0 (Output)
     *
     * Primary function out of reset: IPU_LD0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD1
     * Lido P1 Signal: IPU_LD(1) (Display)
     * Selected Primary Function: IPU_LD1 (Output)
     *
     * Primary function out of reset: IPU_LD1
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD2
     * Lido P1 Signal: IPU_LD(2) (Display)
     * Selected Primary Function: IPU_LD2 (Output)
     *
     * Primary function out of reset: IPU_LD2
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_IPU_LD2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD3
     * Lido P1 Signal: IPU_LD(3) (Display)
     * Selected Primary Function: IPU_LD3 (Output)
     *
     * Primary function out of reset: IPU_LD3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B2
     */
    { AP_IPU_LD3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD4
     * Lido P1 Signal: IPU_LD(4) (Display)
     * Selected Primary Function: IPU_LD4 (Output)
     *
     * Primary function out of reset: IPU_LD4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B3
     */
    { AP_IPU_LD4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD5
     * Lido P1 Signal: IPU_LD(5) (Display)
     * Selected Primary Function: IPU_LD5 (Output)
     *
     * Primary function out of reset: IPU_LD5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B4
     */
    { AP_IPU_LD5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD6
     * Lido P1 Signal: IPU_LD(6) (Display)
     * Selected Primary Function: IPU_LD6 (Output)
     *
     * Primary function out of reset: IPU_LD6
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B5
     */
    { AP_IPU_LD6, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD7
     * Lido P1 Signal: IPU_LD(7) (Display)
     * Selected Primary Function: IPU_LD7 (Output)
     *
     * Primary function out of reset: IPU_LD7
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B6
     */
    { AP_IPU_LD7, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD8
     * Lido P1 Signal: IPU_LD(8) (Display)
     * Selected Primary Function: IPU_LD8 (Output)
     *
     * Primary function out of reset: IPU_LD8
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B7
     */
    { AP_IPU_LD8, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD9
     * Lido P1 Signal: IPU_LD(9) (Display)
     * Selected Primary Function: IPU_LD9 (Output)
     *
     * Primary function out of reset: IPU_LD9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B8
     */
    { AP_IPU_LD9, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD10
     * Lido P1 Signal: IPU_LD(10) (Display)
     * Selected Primary Function: IPU_LD10 (Output)
     *
     * Primary function out of reset: IPU_LD10
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B9
     */
    { AP_IPU_LD10, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD11
     * Lido P1 Signal: IPU_LD(11) (Display)
     * Selected Primary Function: IPU_LD11 (Output)
     *
     * Primary function out of reset: IPU_LD11
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B10
     */
    { AP_IPU_LD11, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD12
     * Lido P1 Signal: IPU_LD(12) (Display)
     * Selected Primary Function: IPU_LD12 (Output)
     *
     * Primary function out of reset: IPU_LD12
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B11
     */
    { AP_IPU_LD12, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD13
     * Lido P1 Signal: IPU_LD(13) (Display)
     * Selected Primary Function: IPU_LD13 (Output)
     *
     * Primary function out of reset: IPU_LD13
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B12
     */
    { AP_IPU_LD13, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD14
     * Lido P1 Signal: IPU_LD(14) (Display)
     * Selected Primary Function: IPU_LD14 (Output)
     *
     * Primary function out of reset: IPU_LD14
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B13
     */
    { AP_IPU_LD14, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_LD15
     * Lido P1 Signal: IPU_LD(15) (Display)
     * Selected Primary Function: IPU_LD15 (Output)
     *
     * Primary function out of reset: IPU_LD15
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B14
     */
    { AP_IPU_LD15, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_B18
     * Lido P1 Signal: GPIO_DISP_RST_B (Display)
     * Selected Primary Function: GP_AP_B18 (Output)
     *
     * Primary function out of reset: GP_AP_B18
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B18
     */
    { AP_GPIO_AP_B18, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C14
     * Lido P1 Signal: IPU_SD_D1_CS (Display)
     * Selected Primary Function: IPU_D1_CS (Output)
     *
     * Primary function out of reset: GP_AP_C14
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C14
     */
    { AP_GPIO_AP_C14, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Haptics IOMUX settings.
 */
static struct iomux_initialization haptics_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: GP_AP_B17
     * Lido P1 Signal: HAPTIC_PULSE (NC)
     * Selected Primary Function: AP_PWM (Output)
     *
     * Primary function out of reset: GP_AP_B17
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B17
     */
    { AP_GPIO_AP_B17, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * Lido P1 Signal: LIN_VIB_AMP_EN (Haptics)
     * Selected Primary Function: GP_SP_A10 (Output)
     *
     * Primary function out of reset: UH2_SUSPEND
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A10
     */
    { SP_UH2_SUSPEND, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * I2C IOMUX settings.
 */
static struct iomux_initialization i2c_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: I2CLK
     * Lido P1 Signal: I2C_CLK (Camera/USB_HS)
     * Selected Primary Function: I2CLK (Input/Output)
     *
     * Primary function out of reset: I2CLK
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C6
     */
    { AP_I2CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: I2DAT
     * Lido P1 Signal: I2C_DAT (Camera/USB_HS)
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
     * Lido P1 Signal: KPP_ROW(4) (Keypad)
     * Selected Primary Function: KPROW4 (Input/Output)
     *
     * Primary function out of reset: KPROW4
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL3
     * Lido P1 Signal: KPP_COL(3) (Keypad)
     * Selected Primary Function: KPCOL3 (Input/Output)
     *
     * Primary function out of reset: KPCOL3
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B19
     */
    { AP_KPCOL3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL4
     * Lido P1 Signal: KPP_COL(4) (Keypad)
     * Selected Primary Function: KPCOL4 (Input/Output)
     *
     * Primary function out of reset: KPCOL4
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B20
     */
    { AP_KPCOL4, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL5
     * Lido P1 Signal: KPP_COL(5) (Keypad)
     * Selected Primary Function: KPCOL5 (Input/Output)
     *
     * Primary function out of reset: KPCOL5
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B21
     */
    { AP_KPCOL5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW0
     * Lido P1 Signal: KPP_ROW(0) (Keypad)
     * Selected Primary Function: KPROW0 (Input/Output)
     *
     * Primary function out of reset: KPROW0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW1
     * Lido P1 Signal: KPP_ROW(1) (Keypad)
     * Selected Primary Function: KPROW1 (Input/Output)
     *
     * Primary function out of reset: KPROW1
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B15
     */
    { AP_KPROW1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW2
     * Lido P1 Signal: KPP_ROW(2) (Keypad)
     * Selected Primary Function: KPROW2 (Input/Output)
     *
     * Primary function out of reset: KPROW2
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPROW3
     * Lido P1 Signal: KPP_ROW(3) (Keypad)
     * Selected Primary Function: KPROW3 (Input/Output)
     *
     * Primary function out of reset: KPROW3
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPROW3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL0
     * Lido P1 Signal: KPP_COL(0) (Keypad)
     * Selected Primary Function: KPCOL0 (Input/Output)
     *
     * Primary function out of reset: KPCOL0
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPCOL0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL1
     * Lido P1 Signal: KPP_COL(1) (Keypad)
     * Selected Primary Function: KPCOL1 (Input/Output)
     *
     * Primary function out of reset: KPCOL1
     * Out of Reset State: Input
     * Mux0 Function: (not defined)
     */
    { AP_KPCOL1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: KPCOL2
     * Lido P1 Signal: KPP_COL(2) (Keypad)
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
 * Miscellaneous peripheral IOMUX settings.
 */
static struct iomux_initialization misc_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: OWDAT
     * Lido P1 Signal: BATT_DAT (Misc)
     * Selected Primary Function: OWDAT (Input/Output)
     *
     * Primary function out of reset: OWDAT
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A19
     */
    { AP_OWDAT, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: ED_INT2
     * Lido P1 Signal: WDOG_BP (Misc)
     * Selected Primary Function: GP_AP_C20 (Input)
     * Selected Secondary Function: ED_INT2 (Input)
     *
     * Primary function out of reset: ED_INT2
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C20
     */
    { AP_ED_INT2, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: ED_INT3
     * Lido P1 Signal: FLIP_OPEN (Misc)
     * Selected Primary Function: GP_AP_C21 (Input)
     * Selected Secondary Function: ED_INT3 (Input)
     *
     * Primary function out of reset: ED_INT3
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C21
     */
    { AP_ED_INT3, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: U2_DSR_B
     * Lido P1 Signal: DAI_TX (Misc)
     * Selected Primary Function: AD4_TXD (Output)
     *
     * Primary function out of reset: U2_DSR_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C26
     */
    { AP_U2_DSR_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U2_RI_B
     * Lido P1 Signal: DAI_CLK (Misc)
     * Selected Primary Function: AD4_TXC (Input/Output)
     *
     * Primary function out of reset: U2_RI_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C27
     */
    { AP_U2_RI_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_FUNC2 },
    /*
     * SCM-A11 Package Pin Name: U2_CTS_B
     * Lido P1 Signal: DAI_FS (Misc)
     * Selected Primary Function: AD4_TXFS (Input/Output)
     *
     * Primary function out of reset: U2_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C28
     */
    { AP_U2_CTS_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_FUNC2 },
    /*
     * SCM-A11 Package Pin Name: U2_DTR_B
     * Lido P1 Signal: DAI_RX (Misc)
     * Selected Primary Function: AD4_RXD (Input)
     *
     * Primary function out of reset: U2_DTR_B
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C29
     */
    { AP_U2_DTR_B, OUTPUTCONFIG_FUNC2, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * Multi-media card IOMUX settings.
 */
static struct iomux_initialization mmc_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: SD1_DAT0
     * Lido P1 Signal: SD1_DATA(0) (MMC)
     * Selected Primary Function: SD1_DAT0 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT0
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A16
     */
    { SP_SD1_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT1
     * Lido P1 Signal: SD1_DATA(1) (MMC)
     * Selected Primary Function: SD1_DAT1 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT1
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A17
     */
    { SP_SD1_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT2
     * Lido P1 Signal: SD1_DATA(2) (MMC)
     * Selected Primary Function: SD1_DAT2 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT2
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A18
     */
    { SP_SD1_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_DAT3
     * Lido P1 Signal: SD1_DATA(3) (MMC)
     * Selected Primary Function: SD1_DAT3 (Input/Output)
     *
     * Primary function out of reset: SD1_DAT3
     * Out of Reset State: Hi-Z
     * Mux0 Function: GP_SP_A19
     */
    { SP_SD1_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_CMD
     * Lido P1 Signal: SD1_CMD (MMC)
     * Selected Primary Function: SD1_CMD (Input/Output)
     *
     * Primary function out of reset: SD1_CMD
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A20
     */
    { SP_SD1_CMD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD1_CLK
     * Lido P1 Signal: SD1_CLK (MMC)
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
 * Morphing IOMUX settings.
 */
static struct iomux_initialization morphing_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: UH2_RXDP
     * Lido P1 Signal: TOUCH_INTB (Morphing)
     * Selected Primary Function: GP_SP_A12 (Output)
     *
     * Primary function out of reset: UH2_RXDP
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A12
     */
    { SP_UH2_RXDP, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * NAND controller IOMUX settings.
 */
static struct iomux_initialization nand_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: CLE
     * Lido P1 Signal: NAND_CLE (NAND)
     * Selected Primary Function: CLE (Output)
     *
     * Primary function out of reset: CLE
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A0
     */
    { AP_CLE, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ALE
     * Lido P1 Signal: NAND_ALE (NAND)
     * Selected Primary Function: ALE (Output)
     *
     * Primary function out of reset: ALE
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A1
     */
    { AP_ALE, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: CE_B
     * Lido P1 Signal: NAND_CS_B (NAND)
     * Selected Primary Function: CE_B (Output)
     *
     * Primary function out of reset: CE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A2
     */
    { AP_CE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: RE_B
     * Lido P1 Signal: NAND_RE_B (NAND)
     * Selected Primary Function: RE_B (Output)
     *
     * Primary function out of reset: RE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A3
     */
    { AP_RE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: WE_B
     * Lido P1 Signal: NAND_WE_B (NAND)
     * Selected Primary Function: WE_B (Output)
     *
     * Primary function out of reset: WE_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A4
     */
    { AP_WE_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: WP_B
     * Lido P1 Signal: NAND_WP_B (NAND)
     * Selected Primary Function: WP_B (Output)
     *
     * Primary function out of reset: WP_B
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A5
     */
    { AP_WP_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: BSY_B
     * Lido P1 Signal: NAND_R_B (NAND)
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
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A27 (Output)
     *
     * Primary function out of reset: SPI1_CLK
     * Out of Reset State: High-Z
     * Mux0 Function: GP_SP_A27
     */
    { SP_SPI1_CLK, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_PWR
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A15 (Output)
     *
     * Primary function out of reset: UH2_PWR
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A15
     */
    { SP_UH2_PWR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT4
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_AP_C22 (Input)
     * Selected Secondary Function: ED_INT4 (Input)
     *
     * Primary function out of reset: ED_INT4
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C22
     */
    { AP_ED_INT4, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: UH2_RXDM
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A13 (Output)
     *
     * Primary function out of reset: UH2_RXDM
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A13
     */
    { SP_UH2_RXDM, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D3_CONTR
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: IPU_SER_RS (Output)
     *
     * Primary function out of reset: IPU_D3_CONTR
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_A25
     */
    { AP_IPU_D3_CONTR, OUTPUTCONFIG_FUNC3, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_D2_CS
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: IPU_D2_CS (Output)
     *
     * Primary function out of reset: IPU_D2_CS
     * Out of Reset State: High
     * Mux0 Function: GP_AP_A28
     */
    { AP_IPU_D2_CS, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: IPU_WR
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: GPIO_SER_RST_B (Display)
     * Selected Primary Function: GP_AP_B1 (Output)
     *
     * Primary function out of reset: IPU_WR
     * Out of Reset State: High
     * Mux0 Function: GP_AP_B1
     */
    { AP_IPU_WR, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C12
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_AP_C12 (Output)
     *
     * Primary function out of reset: GP_AP_C12
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C12
     */
    { AP_GPIO_AP_C12, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT0
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_AP_C18 (Input)
     * Selected Secondary Function: ED_INT0 (Input)
     *
     * Primary function out of reset: ED_INT0
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C18
     */
    { AP_ED_INT0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: ED_INT7
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_AP_C25 (Input)
     * Selected Secondary Function: ED_INT7 (Input)
     *
     * Primary function out of reset: ED_INT7
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C25
     */
    { AP_ED_INT7, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: LIN_VIB_AMP_EN (Haptics)
     * Selected Primary Function: GP_SP_A2 (Output)
     *
     * Primary function out of reset: U3_RTS_B
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A2
     */
    { SP_U3_RTS_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U3_CTS_B
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A3 (Output)
     *
     * Primary function out of reset: U3_CTS_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A3
     */
    { SP_U3_CTS_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_TXOE_B
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A8 (Output)
     *
     * Primary function out of reset: UH2_TXOE_B
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A8
     */
    { SP_UH2_TXOE_B, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_SPEED
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A9 (Output)
     *
     * Primary function out of reset: UH2_SPEED
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A9
     */
    { SP_UH2_SPEED, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT0
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_DAT0 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT0
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A22
     */
    { SP_SD2_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT1
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_DAT1 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT1
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A23
     */
    { SP_SD2_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT2
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_DAT2 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT2
     * Out of Reset State: High
     * Mux0 Function: GP_SP_A24
     */
    { SP_SD2_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_DAT3
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_DAT3 (Input/Output)
     *
     * Primary function out of reset: SD2_DAT3
     * Out of Reset State: Hi-Z
     * Mux0 Function: GP_SP_A25
     */
    { SP_SD2_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: GP_SP_A26
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A26 (Input)
     *
     * Primary function out of reset: GP_SP_A26
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A26
     */
    { SP_GPIO_Shared26, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SPI1_SS1
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A31 (Output)
     *
     * Primary function out of reset: SPI1_SS1
     * Out of Reset State: Output
     * Mux0 Function: GP_SP_A31
     */
    { SP_SPI1_SS1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SD2_CMD
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_CMD (Input/Output)
     *
     * Primary function out of reset: SD2_CMD
     * Out of Reset State: High
     * Mux0 Function: (not defined)
     */
    { SP_SD2_CMD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: SD2_CLK
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: SD2_CLK (Output)
     *
     * Primary function out of reset: SD2_CLK
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SD2_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_TXDM
     * Lido P1 Signal: NC (NC)
     * Selected Primary Function: Unassigned (Input)
     *
     * Primary function out of reset: UH2_TXDM
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_UH2_TXDM, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: UH2_RXD
     * Lido P1 Signal: NC (NC)
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
 * SIM controller IOMUX settings.
 */
static struct iomux_initialization sim_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: SIM1_RST_B
     * Lido P1 Signal: SIM_RST (SIM)
     * Selected Primary Function: SIM1_RST_B (Output)
     *
     * Primary function out of reset: SIM1_RST_B
     * Out of Reset State: Low
     * Mux0 Function: GP_AP_C30
     */
    { SP_SIM1_RST_B, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_CLK
     * Lido P1 Signal: SIM_CLK (SIM)
     * Selected Primary Function: SIM1_CLK (Output)
     *
     * Primary function out of reset: SIM1_CLK
     * Out of Reset State: Low
     * Mux0 Function: (not defined)
     */
    { SP_SIM1_CLK, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SIM1_TRXD
     * Lido P1 Signal: SIM_IO (SIM)
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
 * UART3 IOMUX settings.
 */
static struct iomux_initialization uart3_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: U3_TXD
     * Lido P1 Signal: GPS_U3_TX (Linux Terminal)
     * Selected Primary Function: U3_TXD (Output)
     *
     * Primary function out of reset: U3_TXD
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A0
     */
    { SP_U3_TXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: U3_RXD
     * Lido P1 Signal: GPS_U3_RX (Linux Terminal)
     * Selected Primary Function: U3_RXD (Input)
     *
     * Primary function out of reset: U3_RXD
     * Out of Reset State: Input
     * Mux0 Function: GP_SP_A1
     */
    { SP_U3_RXD, OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE },
    /* list terminator */
    { IOMUX_INVALID_PIN, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT }
};

/**
 * High-speed USB IOMUX settings.
 */
static struct iomux_initialization usb_hs_iomux_settings[] __initdata = {
    /*
     * SCM-A11 Package Pin Name: GP_AP_B22
     * Lido P1 Signal: USB_HS_DMA_REQ (USB HS)
     * Selected Primary Function: GP_AP_B22 (Input)
     *
     * This varies from the pin list. The Func2 mux setting for this pin is
     * DMAREQ0.
     *
     * Primary function out of reset: GP_AP_B22
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_B22
     */
    { AP_GPIO_AP_B22, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C9
     * Lido P1 Signal: USB_HS_WAKEUP/10K PD (USB HS)
     * Selected Primary Function: GP_AP_C9 (Output)
     *
     * Primary function out of reset: GP_AP_C9
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C9
     */
    { AP_GPIO_AP_C9, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: GP_AP_C10
     * Lido P1 Signal: USB_HS_FLAGC (USB HS)
     * Selected Primary Function: GP_AP_C10 (Input)
     *
     * Primary function out of reset: GP_AP_C10
     * Out of Reset State: Input
     * Mux0 Function: GP_AP_C10
     */
    { AP_GPIO_AP_C10, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: ED_INT6
     * Lido P1 Signal: USB_HS_INT (USB HS)
     * Selected Primary Function: GP_AP_C24 (Input)
     * Selected Secondary Function: ED_INT6 (Input)
     *
     * Primary function out of reset: ED_INT6
     * Out of Reset State: High
     * Mux0 Function: GP_AP_C24
     */
    { AP_ED_INT6, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC1 },
    /*
     * SCM-A11 Package Pin Name: UH2_TXDP
     * Lido P1 Signal: USB_HS_SWITCH (USB HS)
     * Selected Primary Function: GP_SP_A11 (Output)
     *
     * Primary function out of reset: UH2_TXDP
     * Out of Reset State: Low
     * Mux0 Function: GP_SP_A11
     */
    { SP_UH2_TXDP, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
    /*
     * SCM-A11 Package Pin Name: SPI1_MOSI
     * Lido P1 Signal: USB_HS_RESET (USB HS)
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
    camera_iomux_settings,
    camera_flash_iomux_settings,
    display_iomux_settings,
    haptics_iomux_settings,
    i2c_iomux_settings,
    keypad_iomux_settings,
    misc_iomux_settings,
    mmc_iomux_settings,
    morphing_iomux_settings,
    nand_iomux_settings,
    nc_iomux_settings,
    sim_iomux_settings,
    uart3_iomux_settings,
    usb_hs_iomux_settings,
    /* list terminator */
    NULL
};


/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by
 * \b fixup_scma11() during system startup. This function is board specific.
 */
void __init lido_gpio_init(void)
{
    int i, j;

#ifdef CONFIG_MOT_FEAT_BRDREV
    if( (boardrev() < BOARDREV_P2A) || (boardrev() == BOARDREV_UNKNOWN) ) {
        gpio_setting_fixup_p1();
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    /* set iomux pad registers to the prescribed state */
    for(i = IOMUX_PAD_SETTING_START; i <= IOMUX_PAD_SETTING_STOP; i++) {
        gpio_tracemsg("Setting pad register 0x%08x to: 0x%08x",
                iomux_pad_register_settings[i].grp,
                iomux_pad_register_settings[i].config);

        iomux_set_pad(iomux_pad_register_settings[i].grp,
                iomux_pad_register_settings[i].config);
    }

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

    /* disable UART1 for Bluetooth current drain improvement */
    gpio_bluetooth_power_set_data(0);
}


#ifdef CONFIG_MOT_FEAT_BRDREV
void __init gpio_setting_fixup_p1(void)
{
    /*
     * SCM-A11 Package Pin Name: U3_RTS_B
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: LIN_VIB_AMP_EN (Haptics)
     * Selected Primary Function: GP_SP_A2 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_SP_A2].port    = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_SP_A2].sig_no  = 2;
    initial_gpio_settings[GPIO_SIGNAL_SP_A2].out     = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_SP_A2].data    = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: UH2_SUSPEND
     * Lido P1 Signal: LIN_VIB_AMP_EN (Haptics)
     * Lido P2 Signal: NC (NC)
     * Selected Primary Function: GP_SP_A10 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_SP_A10].port   = GPIO_INVALID_PORT;

    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].port   = GPIO_SP_A_PORT;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].sig_no = 10;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].out    = GPIO_GDIR_OUTPUT;
    initial_gpio_settings[GPIO_SIGNAL_LIN_VIB_AMP_EN].data   = GPIO_DATA_LOW;

    /*
     * SCM-A11 Package Pin Name: IPU_WR
     * Lido P1 Signal: NC (NC)
     * Lido P2 Signal: GPIO_SER_RST_B (Display)
     * Selected Primary Function: GP_AP_B1 (Output)
     */
    initial_gpio_settings[GPIO_SIGNAL_SER_RST_B].port    = GPIO_INVALID_PORT;
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
 * @param  port         a UART port
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
    /* Keypad IOMUX settings configured at boot. */
}


/*!
 * Setup GPIO for keypad to be inactive
 */
void gpio_keypad_inactive(void)
{
    /* Keypad IOMUX settings configured at boot. */
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
    /* I2C IOMUX settings configured at boot. */
}


/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
    /* I2C IOMUX settings configured at boot. */
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


#if defined(CONFIG_MOT_FEAT_GPIO_API_LCD)

void gpio_ipu_set_pixel_clk(bool enable)
{
    iomux_config_mux(AP_IPU_D3_CLK,
                    (enable==true?OUTPUTCONFIG_FUNC1:OUTPUTCONFIG_DEFAULT),
                    INPUTCONFIG_NONE);
}

#if defined(CONFIG_MOT_FEAT_GPIO_API_SERIALIZER)

/*!
 * Setup GPIO for the serializer RESET 
 */
void gpio_lcd_serializer_reset(int asserted)
{
    /* GPIO_SIGNAL_DISP_RST_B is serializer reset */
    gpio_signal_set_data(GPIO_SIGNAL_DISP_RST_B, (asserted == GPIO_SIGNAL_ASSERT) ?
	    GPIO_DATA_LOW : GPIO_DATA_HIGH);
}

/*!
 * Setup GPIO for serializer STBY
 */
void gpio_lcd_serializer_stby(int asserted)
{
    /* 
     * Pull the serializer out of STBY. Must occur at least 
     * 20us after calling gpio_lcd_serializer_reset
     */
    gpio_signal_set_data(GPIO_SIGNAL_SER_EN, (asserted == GPIO_SIGNAL_ASSERT) ? 
	    GPIO_DATA_LOW : GPIO_DATA_HIGH);
}
#endif /* defined(CONFIG_MOT_FEAT_GPIO_API_SERIALIZER) */

#endif /* CONFIG_MOT_FEAT_GPIO_API_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_FLASH)
/**
 * Turn the camera flash on or off.
 *
 * @param   eanble  Non-zero enables the camera flash.
 */
void gpio_camera_flash_enable(int enable)
{
    gpio_signal_set_data(GPIO_SIGNAL_CAM_FLASH_EN,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_FLASH */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL)
/**
 * Turn the backlight for the number keys on or off.
 *
 * @param   enable  Set to non-zero to enable the backlight.
 */
void gpio_backlight_numbers_enable(int enable)
{
    gpio_signal_set_data(GPIO_SIGNAL_EL_EN,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}


/**
 * Turn the backlight for the navigation keys on or off.
 *
 * @param   enable  Set to non-zero to enable the backlight.
 */
void gpio_backlight_navigation_enable(int enable)
{
    gpio_signal_set_data(GPIO_SIGNAL_EL_EN,
            enable ? GPIO_DATA_HIGH : GPIO_DATA_LOW);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL */


#if defined(CONFIG_MOT_FEAT_GPIO_API_CSI)
/*!
 * Setup GPIO for sensor to be active
 */
void gpio_sensor_active(void)
{
    /* camera is taken out of reset at boot, but powered down */
    gpio_signal_set_data(GPIO_SIGNAL_CAM_PD, GPIO_DATA_LOW);
}


/**
 * Power down the camera.
 */
void gpio_sensor_inactive(void)
{
    gpio_signal_set_data(GPIO_SIGNAL_CAM_PD, GPIO_DATA_HIGH);
}
#endif /* CONFIG_MOT_FEAT_GPIO_API_CSI */


#if defined(CONFIG_MOT_FEAT_FLIP) 
/*
 * Flip Interrupt Functions
 *
 * SCM-A11 Package Pin Name: ED_INT3
 * Lido P1 Signal: FLIP_OPEN (Misc)
 * Selected Primary Function: GP_AP_C21 (Input)
 * Selected Secondary Function: ED_INT3 (Input)
 */

/**
 * Register the flip interrupt request.
 */
int gpio_flip_request_irq(gpio_irq_handler handler, unsigned long irq_flags,
        const char *devname, void *dev_id)
{
    set_irq_type(INT_EXT_INT3, IRQT_BOTHEDGE);
    return request_irq(INT_EXT_INT3, handler, irq_flags, devname, dev_id);
}


/**
 * Free the flip interrupt request.
 */
int gpio_flip_free_irq(void *dev_id)
{
    free_irq(INT_EXT_INT3, dev_id);
    return 0;
}


/**
 * Clear the flip interrupt if it is set.
 */
void gpio_flip_clear_int(void)
{
    /* NOOP */
}


/**
 * Get the current state of the flip.
 *
 * @return  High value (non-zero) indicates flip is open; low value (zero)
 * indicates that flip is closed.
 */
int gpio_flip_open(void)
{
    return gpio_signal_get_data_check(GPIO_SIGNAL_FLIP_OPEN);
}

#endif /* CONFIG_MOT_FEAT_FLIP */


#if defined(CONFIG_MOT_FEAT_GPIO_API_DAI)
/**
 * Multiplexing Use Case 3 -- DAI on UART2 pins
 */
void gpio_dai_enable(void)
{
    /* DAI IOMUX settings configured at boot. */
}


/**
 * Return DAI/UART2 to default state.
 */
void gpio_dai_disable(void)
{
    /* DAI IOMUX settings configured at boot. */
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
    unsigned long saveirqs;
    int retval;

    local_irq_save(saveirqs);

    retval = gpio_signal_request_irq(GPIO_SIGNAL_USB_HS_FLAGC,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);

    if(retval == 0) {
        /* only configure signal for interrupts if request_irq was successful */
        gpio_signal_config(GPIO_SIGNAL_USB_HS_FLAGC, GPIO_GDIR_INPUT,
                GPIO_INT_HIGH_LEV);
    }

    local_irq_restore(saveirqs);

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
    unsigned long saveirqs;
    int retval;

    local_irq_save(saveirqs);

    retval = gpio_signal_request_irq(GPIO_SIGNAL_USB_HS_DMA_REQ,
            GPIO_HIGH_PRIO, handler, irq_flags, devname, dev_id);

    local_irq_restore(saveirqs);

    return retval;
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

#if defined(CONFIG_MOT_FEAT_GPIO_API_MMCSD)
/**
 * Clear the IOMUX/GPIO for MMC/SD.
 */
void mmcsd_intr_destroy(void *host)
{
    /* reserved */
}


/**
 * Setup GPIO for MMC/SD
 *
 * @param id    mmc/sd host id
 */
void gpio_mmcsd_active(int id)
{
    /* reserved */
}


/**
 * Setup GPIO for MMC/SD
 *
 * @param id    mmc/sd host id
 */
void gpio_mmcsd_inactive(int id)
{
    /* reserved */
}

/**
 * Setup the EDIO pin for MMC/SD.
 *
 * @param  host         Pointer to MMC/SD host structure.
 * @param  handler      GPIO ISR function pointer for the GPIO signal.
 *
 * @return The function returns 0 on success and -1 on failure.
 */
int mmcsd_intr_setup(void *host, gpio_irq_handler handler)
{
    return 0;
}


/**
 * Clear the EDIO for MMC/SD.
 *
 * @param flag Flag represents whether the card is inserted/removed.
 *             Using this sensitive level of GPIO signal is changed.
 *
 */
void mmcsd_intr_clear(int *flag)
{
    /* reserved */
}

/*
 * Find the minimum clock for MMC/SD.
 *
 * @param   clk SDHC module number.

 * @return  Returns the minimum SDHC clock.
 */
unsigned int mmcsd_get_min_clock(enum mxc_clocks clk)
{
    return (mxc_get_clocks(clk) / 9) / 32 ;
}


/**
 * Find the maximum clock for MMC/SD.

 * @param   clk SDHC module number.
 * 
 * @return  Returns the maximum SDHC clock.
 */
unsigned int mmcsd_get_max_clock(enum mxc_clocks clk)
{
    return mxc_get_clocks(clk) / 2;
}

/**
 * Probe for the card. Lido T1 has a card soldered to SDHC1; the Lido P-series
 * does not support MMC.
 *
 * @param   id  mmc/sd host id
 * 
 * @return  Zero if card is present; non-zero otherwise.
 */
int mmcsd_find_card(int id)
{
    int data = 1;

#ifdef CONFIG_MOT_FEAT_BRDREV
    /* boardrev P7A is used for Lido T1 */
    if( (boardrev() < BOARDREV_P7A) || (boardrev() == BOARDREV_UNKNOWN) ) {
        return 1; /* Lido P-series does not include transflash */
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */

    switch (id) {
        case 0:
            data = 0;
            break;
            
	case 1:
            data = 1;
            break;

	default:
            printk("%s: we should not be here!\n", __FUNCTION__);
    }
    return data;
}

#endif /*CONFIG_MOT_FEAT_GPIO_API_MMCSD */
