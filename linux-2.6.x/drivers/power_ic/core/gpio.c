/*
 * Copyright (C) 2006 Motorola, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2006-Nov-09 - ReConfigure GPIO Keypad backlight
 * Motorola 2006-Oct-06 - Update File
 * Motorola 2006-Sep-08 - Fix use of BOARDREV for Lido and Saipan.
 * Motorola 2006-Aug-15 - Add Elba support.
 * Motorola 2006-Jul-31 - Update comments
 * Motorola 2006-Jul-26 - Add TransFlash support to BUTE
 * Motorola 2006-Jul-14 - Add Lido & Saipan Support
 * Motorola 2006-Jun-28 - Montavista header upmerges
 * Motorola 2006-Jun-19 - Fix montavista upmerge conditionals
 * Motorola 2006-Jun-14 - Modify keypad lights for HW revisions.
 * Motorola 2006-Jun-14 - Fix TransFlash detection on Ascension
 * Motorola 2006-Jun-19 - Fix montavista upmerge conditionals
 * Motorola 2006-Jun-14 - Fix TransFlash detection on Ascension
 * Motorola 2006-May-12 - Add Zeus support for SD
 * Motorola 2006-Apr-24 - Add 3.5mm headset support
 * Motorola 2006-Apr-04 - Confine all GPIO functions to this file
 */

 /*!
 * @file gpio.c
 *
 * @ingroup poweric_core
 *
 * @brief The main interface to GPIOs.
 *
 * This file contains all the configurations, setting, and receiving of all GPIOs
 */
#include <stdbool.h>

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/power_ic_kernel.h>
/* Must include before interrupt.h since interrupt.h depends on sched.h but is
 * not included in interrupt.h */
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/boardrev.h>
#include <asm/mot-gpio.h>
#include <asm/arch/gpio.h>
#ifdef CONFIG_ARCH_MXC91321
#include <asm/arch/mxc91321_pins.h>
#elif defined(CONFIG_ARCH_MXC91231)
#include <asm/arch/mxc91231_pins.h>
#endif

#include "event.h"
#include "gpio.h"
#include "os_independent.h"

#include "sdhc_main.h"
/******************************************************************************
* Local type definitions
******************************************************************************/

/******************************************************************************
* Local constants
******************************************************************************/
/*!
 * @name Main Display Lighting
 */
/*@{*/
#define LIGHTS_FL_MAIN_BKL_PIN               30
#define LIGHTS_FL_PWM_BKL_PIN                17
/*@}*/

/*!
 * @name EL Keypad Lighting
 */
/*@{*/
#define LIGHTS_FL_EL_BASE                    29
#define LIGHTS_FL_EL_SLIDER                  27
/*@}*/

/*!
 * @name Camera Flash
 */
/*@{*/
#define LIGHTS_FL_CAMERA_EN_P2               13
#define LIGHTS_FL_TORCH_FLASH                30
/*@}*/

/*!@cond INTERNAL */
/*!
 *@brief Defines to match scma11 in order to minimize the differences in this file.
 */
#if defined(CONFIG_MACH_ARGONLVREF)
#define SP_SD1_CMD              PIN_MMC1_CMD
#define SP_SD1_CLK              PIN_MMC1_CLK
#define SP_SD1_DAT0             PIN_MMC1_DATA_0
#define SP_SD1_DAT1             PIN_MMC1_DATA_1
#define SP_SD1_DAT2             PIN_MMC1_DATA_2
#define SP_SD1_DAT3             PIN_MMC1_DATA_3
#define SP_SD2_CMD              PIN_MMC2_CMD
#define SP_SD2_CLK              PIN_MMC2_CLK
#define SP_SD2_DAT0             PIN_MMC2_DATA_0
#define SP_SD2_DAT1             PIN_MMC2_DATA_1
#define SP_SD2_DAT2             PIN_MMC2_DATA_2
#define SP_SD2_DAT3             PIN_MMC2_DATA_3
#define OUTPUTCONFIG_DEFAULT    OUTPUTCONFIG_GPIO
#define OUTPUTCONFIG_FUNC1      OUTPUTCONFIG_FUNC
#define INPUTCONFIG_FUNC1       INPUTCONFIG_FUNC
#define INPUTCONFIG_NONE        INPUTCONFIG_NONE

#elif defined(CONFIG_ARCH_MXC91131)
#define SP_SD1_CMD              SD1_CMD_PIN
#define SP_SD1_CLK              SD1_CLK_PIN
#define SP_SD1_DAT0             SD1_DAT0_PIN
#define SP_SD1_DAT1             SD1_DAT1_PIN
#define SP_SD1_DAT2             SD1_DAT2_PIN
#define SP_SD1_DAT3             SD1_DAT3_PIN
#define SP_SD2_CMD              SD2_CMD_PIN
#define SP_SD2_CLK              SD2_CLK_PIN
#define SP_SD2_DAT0             SD2_DAT0_PIN
#define SP_SD2_DAT1             SD2_DAT1_PIN
#define SP_SD2_DAT2             SD2_DAT2_PIN
#define SP_SD2_DAT3             SD2_DAT3_PIN
#define OUTPUTCONFIG_DEFAULT    MUX0_OUT
#define OUTPUTCONFIG_FUNC1      MUX0_OUT
#define INPUTCONFIG_FUNC1       MUX0_IN
#define INPUTCONFIG_NONE        NONE_IN

#endif /* CONFIG_ARCH_MXC91131 */
/*!@endcond */

/*!
 * @brief Port to which the media card is attached.
 */
#define SDHC_POLL_GPIO_PORT 3

/*!
 * @brief Bit which SDHC1 DAT[3] is connected on SDHC_POLL_GPIO_PORT.
 */
#define SDHC_1_GPIO_BIT 19

/*!
 * @brief Bit which SDHC2 DAT[3] is connected on SDHC_POLL_GPIO_PORT.
 */
#define SDHC_2_GPIO_BIT 25

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
/*!
 * @brief Initializes the power IC event GPIO lines
 *
 * This function initializes the power IC event GPIOs.  This includes:
 *     - Configuring the GPIO interrupt lines
 *     - Registering the GPIO interrupt handlers
 *
 */
void power_ic_gpio_config_event_int(void)
{
#if defined(CONFIG_ARCH_MXC91231)
    /* Configure mux */
    iomux_config_mux(AP_ED_INT5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1);
    /* Configure and register the ATLAS interrupt */
    set_irq_type(PM_INT, IRQT_RISING);
    request_irq(PM_INT, power_ic_irq_handler, SA_INTERRUPT, "Atlas irq: SCM-A11", 0);
#elif defined(CONFIG_MACH_ARGONLVREF)
    /* Configure mux */
    iomux_config_mux(PIN_PM_INT, OUTPUTCONFIG_ALT2, INPUTCONFIG_ALT2);
    /* Configure and register the ATLAS interrupt */
    set_irq_type(PM_INT, IRQT_RISING);
    request_irq(PM_INT, power_ic_irq_handler, SA_INTERRUPT, "Atlas irq: BUTE", 0);
#endif
}

/*!
 * @brief Read the power IC event GPIO lines
 *
 * This function will read the primary interrupt GPIO line
 *
 * @return UINT32 value of the GPIO data
 *
 */
int power_ic_gpio_event_read_priint(unsigned int signo)
{
    return(edio_get_data(PM_EDIO));
}

/*!
 * @brief Configure the power IC EMU GPIO lines
 *
 * This function will configure the EMU GPIO lines for D+ and D-
 */
void power_ic_gpio_emu_config(void)
{
#ifdef CONFIG_ARCH_MXC91231
    iomux_config_mux(AP_GPIO_AP_C16, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC3);
    iomux_config_mux(AP_GPIO_AP_C17, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC3);
    gpio_config(GPIO_AP_C_PORT, 16, false, GPIO_INT_NONE);
    gpio_config(GPIO_AP_C_PORT, 17, false, GPIO_INT_NONE);
#elif defined(CONFIG_MACH_ARGONLVREF)
    iomux_config_mux(PIN_USB_VPIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
    iomux_config_mux(PIN_USB_VMIN, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
    iomux_config_mux(PIN_GPIO4, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
    iomux_config_mux(PIN_GPIO6, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
    gpio_config(0, 4, false, GPIO_INT_NONE);  /* D+ for GPIO 4 (0 Base) */
    gpio_config(0, 6, false, GPIO_INT_NONE);  /* D- for GPIO 6 (0 Base) */
#endif
}
/*!
 * @brief Initializes the lighting
 *
 * The function performs any initialization required to get the lighting
 * for SCM-A11 running using GPIOs.
 */
void power_ic_gpio_lights_config(void)
{
#if defined(CONFIG_ARCH_MXC91231) && !(defined(CONFIG_MACH_ASCENSION)) \
     && !(defined(CONFIG_MACH_SAIPAN)) && !(defined(CONFIG_MACH_LIDO)) \
     && !(defined(CONFIG_MACH_ELBA))
#ifdef CONFIG_MOT_FEAT_BRDREV
    if(boardrev() < BOARDREV_P1A)
    {
        iomux_config_mux(AP_GPIO_AP_B17, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
        gpio_config(GPIO_AP_B_PORT, LIGHTS_FL_PWM_BKL_PIN, true, GPIO_INT_NONE);
        gpio_set_data(GPIO_AP_B_PORT, LIGHTS_FL_PWM_BKL_PIN, 1);
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
    iomux_config_mux(AP_IPU_D3_PS, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_config(GPIO_AP_A_PORT, LIGHTS_FL_MAIN_BKL_PIN, true, GPIO_INT_NONE);
    gpio_set_data(GPIO_AP_A_PORT, LIGHTS_FL_MAIN_BKL_PIN, 1);
#elif defined(CONFIG_MACH_ASCENSION) || defined(CONFIG_MACH_LIDO) || defined(CONFIG_MACH_SAIPAN)
    /* EL Keypad GPIOs */
    iomux_config_mux(SP_SPI1_CLK, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_EL_BASE, 0);
    gpio_config(GPIO_SP_A_PORT, LIGHTS_FL_EL_BASE, true, GPIO_INT_NONE);

    iomux_config_mux(SP_SPI1_MISO, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_EL_SLIDER, 0);
    gpio_config(GPIO_SP_A_PORT, LIGHTS_FL_EL_SLIDER, true, GPIO_INT_NONE);

    /* Camera Flash GPIOs */
    iomux_config_mux(SP_SPI1_SS0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_TORCH_FLASH, 0);
    gpio_config(GPIO_SP_A_PORT, LIGHTS_FL_TORCH_FLASH, true, GPIO_INT_NONE);

    iomux_config_mux(SP_UH2_RXDM, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_CAMERA_EN_P2, 0);
    gpio_config(GPIO_SP_A_PORT, LIGHTS_FL_CAMERA_EN_P2, true, GPIO_INT_NONE);
#endif /* SCM-A11 & ASCENSION */
}

/*!
 * @brief Set main display
 *
 *  This function will handle turning on and off the main display thru GPIO
 *
 * @param   nColor     The color to set the region to.
 */
void power_ic_gpio_lights_set_main_display(LIGHTS_FL_COLOR_T nColor)
{
#if !(defined(CONFIG_MACH_ASCENSION))  && !(defined(CONFIG_MACH_LIDO)) && !(defined(CONFIG_MACH_SAIPAN)) \
      && defined(CONFIG_ARCH_MXC91231) && !(defined(CONFIG_MACH_ELBA))
#ifdef CONFIG_MOT_FEAT_BRDREV
    if(boardrev() < BOARDREV_P1A)
    {
        gpio_set_data(GPIO_AP_B_PORT, LIGHTS_FL_PWM_BKL_PIN, (nColor != 0));
    }
#endif /* CONFIG_MOT_FEAT_BRDREV */
    gpio_set_data(GPIO_AP_A_PORT, LIGHTS_FL_MAIN_BKL_PIN, (nColor != 0));
#endif
}


/*!
 * @brief Update keypad base light
 *
 * This function will handle keypad base light update.
 *
 * @param  nColor       The color to set the region to.
 */
void power_ic_gpio_lights_set_keypad_base(LIGHTS_FL_COLOR_T nColor)
{
#ifdef CONFIG_MACH_ASCENSION
    /* Disable keypad lights for P2A and earlier */
    if (boardrev() > BOARDREV_P2A)
    {
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_EL_BASE, (nColor != 0));
    }
    else
    {
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_EL_BASE, 0);
    }
#elif defined(CONFIG_MACH_LIDO) || defined(CONFIG_MACH_SAIPAN)
    gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_EL_BASE, (nColor != 0));
#endif
}

/*!
 * @brief Update keypad slider light
 *
 * This function will handle keypad slider light update.
 *
 * @param  nColor       The color to set the region to.
 */
void power_ic_gpio_lights_set_keypad_slider(LIGHTS_FL_COLOR_T nColor)
{
    /* Currently Not Supported */
}

/*!
 * @brief Sets the Camera Flash and Torch Mode
 *
 * This function sets the camera flash and torch mode to the passed setting
 *
 * @param  nColor       The color to set the region to.
 */
void power_ic_gpio_lights_set_camera_flash(LIGHTS_FL_COLOR_T nColor)
{
#if defined(CONFIG_MACH_ASCENSION) || defined(CONFIG_MACH_LIDO) || defined(CONFIG_MACH_SAIPAN)
    if(nColor == LIGHTS_FL_CAMERA_FLASH )
    {
        /*Turn on the camera flash*/
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_CAMERA_EN_P2, 1);
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_TORCH_FLASH, 1);
    }
    else if (nColor == LIGHTS_FL_CAMERA_TORCH)
    {
        /*Turn on the camera torch mode*/
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_CAMERA_EN_P2, 1);
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_TORCH_FLASH, 0);
    }
    else
    {
        /*Turn off the camera flash*/
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_CAMERA_EN_P2, 0);
        gpio_set_data(GPIO_SP_A_PORT, LIGHTS_FL_TORCH_FLASH, 0);
    }
#endif
}

/*!
 * @brief Configures the MMC interrupts for detection.
 *
 * Configures interrupts for detecting the MMC card
 */
void power_ic_gpio_sdhc_mmc_config(void * call_back)
{
#ifdef CONFIG_ARCH_MXC91231
#ifdef CONFIG_MACH_ASCENSION
    set_irq_type(INT_EXT_INT4, IRQT_BOTHEDGE);
    if (request_irq(INT_EXT_INT4, call_back,
                    SA_INTERRUPT | SA_SAMPLE_RANDOM, "usr_blk_dev card detect", (void *)1))
    {
        printk(KERN_ERR "usr_blk_dev: Unable to initialize card detect interrupt.\n");
    }
#endif
#elif defined(CONFIG_MACH_ARGONLVREF)
    iomux_config_mux(PIN_GPIO17, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
    gpio_config(GPIO_MCU_PORT, GPIO_MCU_PIN_SD1_DET_B, false, GPIO_INT_FALL_EDGE);
    if (gpio_request_irq(GPIO_MCU_PORT, GPIO_MCU_PIN_SD1_DET_B,
            GPIO_HIGH_PRIO, call_back, 0, "usr_blk_dev card detect", (void *)1))
    {
        printk(KERN_ERR "usr_blk_dev: Unable to initialize card detect interrupt.\n");
    }
#elif defined(CONFIG_ARCH_MXC91131)
    /* Configure the card detect pin for GPIO */
    iomux_config_mux(SD1_MMC_PU_CTRL_PIN, MUX0_OUT, GPIO_MUX1_IN);

    /* Initialize the interrupt condition */
    gpio_config(1, USR_BLK_DEV_DEV1_INT_GPIO, false,
                (gpio_get_data(1, USR_BLK_DEV_DEV1_INT_GPIO)
                 ? GPIO_INT_FALL_EDGE : GPIO_INT_RISE_EDGE));

    /* Setup the interrupt handler */
    if (gpio_request_irq(1, USR_BLK_DEV_DEV1_INT_GPIO, GPIO_HIGH_PRIO,
                         call_back, 0, "MMC", (void *)1))
    {
        printk(KERN_ERR "usr_blk_dev: Unable to initialize card detect "
               "interrupt.\n");
    }
#endif
}

/*!
 * @brief Setup GPIO for SDHC
 *
 * @param module SDHC module number
 * @param enable Flag indicating whether to enable or disable the SDHC
 */
void power_ic_gpio_sdhc_gpio_config(SDHC_MODULE_T module, bool enable)
{
    unsigned int i;
    static const struct mux_struct
    {
        int pin;
        int out_config;
        int in_config;
    } mux_tb[4][6] =
    {
        {
            /* enable = 0, module = SDHC_MODULE_1 */
            { SP_SD1_CMD,  OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD1_CLK,  OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD1_DAT0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD1_DAT1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD1_DAT2, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD1_DAT3, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE }
        },
        {
            /* enable = 1, module = SDHC_MODULE_1 */
            { SP_SD1_CMD,  OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD1_CLK,  OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE  },
            { SP_SD1_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD1_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD1_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD1_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 }
        },
        {
            /* enable = 0, module = SDHC_MODULE_2 */
            { SP_SD2_CMD,  OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD2_CLK,  OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD2_DAT0, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD2_DAT1, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD2_DAT2, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE },
            { SP_SD2_DAT3, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE }
        },
        {
            /* enable = 1, module = SDHC_MODULE_2 */
            { SP_SD2_CMD,  OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD2_CLK,  OUTPUTCONFIG_FUNC1, INPUTCONFIG_NONE  },
            { SP_SD2_DAT0, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD2_DAT1, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD2_DAT2, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 },
            { SP_SD2_DAT3, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1 }
        }
    };
    struct mux_struct *mux_p = (struct mux_struct *)&mux_tb[module*2 + enable];

    for (i = 0; i < 6; i++)
    {
        iomux_config_mux(mux_p->pin, mux_p->out_config, mux_p->in_config);
        mux_p++;
    }

#ifdef CONFIG_MACH_MXC91131EVB
    /*
     * The PBC is only present on the ZAS EVB.
     */
    {
        unsigned int pbc_bctrl1_set = readw(PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
        unsigned int port_mask = ((module == 0) ? PBC_BCTRL1_MCP1
                                                : PBC_BCTRL1_MCP2);

        /* Set voltage for the MMC interface. */
        if (enable)
        {
            pbc_bctrl1_set |= port_mask;
        }
        else
        {
            pbc_bctrl1_set &= ~port_mask;
        }

        writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
    }
#endif
}

/*!
 * @brief Polls to see if card is present
 *
 * @param    module SDHC module to poll.
 *
 * @note     No range checking is done on module. It is assumed this
 *           will be done by the caller.
 *
 * @return   0 when no card present.
 *           1 if card inserted.
 */
int power_ic_gpio_sdhc_poll(SDHC_MODULE_T module)
{
    int ret = 0;

#ifdef CONFIG_MACH_SCMA11REF
    enum iomux_pins pin = SP_SD1_DAT3;
    int bit = SDHC_1_GPIO_BIT;

    if(module == SDHC_MODULE_2)
    {
        pin = SP_SD2_DAT3;
        bit = SDHC_2_GPIO_BIT;
    }

    /* Set iomux so card can be polled. */
    iomux_config_mux(pin, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_DEFAULT);
    udelay(125);

    /* Poll for card */
    ret = gpio_get_data(SDHC_POLL_GPIO_PORT, bit);

    /* Change iomux back */
    iomux_config_mux(pin, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1);

#else
    /* Driver only supports polling SDHC1 at this time. */
    if(module == SDHC_MODULE_1)
    {
        /*
         * If *_get_data returns 0, card is present.  If 1, card not present.
         * Because of this, the return of *_get_data is inverted to match what
         * this function is expected to return.
         */
#ifdef CONFIG_MACH_ASCENSION
        ret = !edio_get_data(ED_INT4);
#elif defined(CONFIG_MACH_ARGONLVREF)
        ret = !gpio_get_data(GPIO_MCU_PORT, GPIO_MCU_PIN_SD1_DET_B);
#endif
    }
#endif

    return ret;
}
